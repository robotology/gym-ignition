#include "gympp/gyms/Ignition.h"
#include "gympp/Log.h"
#include "gympp/Random.h"

#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/ServerConfig.hh>
#include <ignition/gazebo/SystemLoader.hh>
#include <ignition/plugin/SpecializedPluginPtr.hh>

#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <unistd.h>

using IgnitionServer = ignition::gazebo::v0::Server;

using namespace gympp::gyms;

struct IgnitionGuiData
{
    std::mutex mutex;
    std::condition_variable cv;
    std::unique_ptr<std::thread> thread{nullptr};

    static void spawnGui(IgnitionGuiData& gui)
    {
        {
            std::unique_lock lock(gui.mutex);
            gymppDebug << "Waiting that the server starts" << std::endl << std::flush;
            gui.cv.wait(lock);
        }

        // This blocks until the window is closed or we receive a SIGINT
        gymppDebug << "Executing the application" << std::endl;
        pid_t guiPid;
        guiPid = fork();
        if (guiPid == 0) {
            // remove client from foreground process group
            setpgid(guiPid, 0);

            // Spin up GUI process and block here
            char** argvGui = new char*[2];
            argvGui[0] = const_cast<char*>("ign-gazebo-gui");
            argvGui[1] = nullptr;
            execvp("ign-gazebo-gui", argvGui);
            ignerr << "Failed to execute GUI";
            exit(EXIT_FAILURE);
        }
    }
};

class IgnitionGazebo::Impl
{
public:
    bool firstRun = true;

    std::string sdfFile;

    uint64_t numOfIterations = 0;
    ignition::gazebo::ServerConfig serverConfig;

    IgnitionGuiData gui;

    struct PluginData
    {
        std::string libName;
        std::string pluginName;

        EnvironmentBehavior* behavior = nullptr;
        ignition::gazebo::SystemPluginPtr systemPluginPtr;
    } pluginData;

    bool loadPlugin(PluginData& pluginData);
};

bool IgnitionGazebo::Impl::loadPlugin(PluginData& pluginData)
{
    // TODO
    //    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
    //           std::string("/home/dferigo/git/gym-ignition/build/lib").c_str(),
    //           1);

    ignition::gazebo::SystemLoader sl;
    sl.AddSystemPluginPath("/home/dferigo/git/gym-ignition/build/lib"); // TODO
    auto plugin = sl.LoadPlugin("libCartPolePlugin.so", "gympp::plugins::CartPole", nullptr);

    if (!plugin.has_value()) {
        gymppError << "Failed to load plugin";
        return false;
    }

    pluginData.systemPluginPtr = plugin.value();

    // Get the child classes out of it
    pluginData.behavior =
        pluginData.systemPluginPtr->template QueryInterface<gympp::gyms::EnvironmentBehavior>();

    if (!pluginData.behavior) {
        gymppError << "Failed to cast the plugin";
        return false;
    }

    return true;
}

IgnitionGazebo::IgnitionGazebo(const std::string libName,
                               const std::string& pluginName,
                               const ActionSpacePtr aSpace,
                               const ObservationSpacePtr oSpace,
                               const std::string& sdfFile,
                               double updateRate,
                               uint64_t iterations)
    : Environment(aSpace, oSpace)
    , pImpl{new IgnitionGazebo::Impl, [](Impl* impl) { delete impl; }}
{
    setVerbosity(4);
    pImpl->sdfFile = sdfFile;
    pImpl->pluginData.libName = libName;
    pImpl->pluginData.pluginName = pluginName;
    pImpl->numOfIterations = iterations;
    pImpl->serverConfig.SetUpdateRate(updateRate);
}

gympp::gyms::IgnitionGazebo::~IgnitionGazebo() = default;

std::optional<IgnitionGazebo::State> IgnitionGazebo::step(const Action& action)
{
    if (pImpl->firstRun) {
        pImpl->firstRun = false;

        // Load the sdf file from the filesystem
        // TODO: get the absolute path with Ignition::Filesystem?
        if (!loadSDF(pImpl->sdfFile)) {
            gymppError << "Failed to load the SDF";
            return {};
        }

        // Load the plugin
        if (!pImpl->loadPlugin(pImpl->pluginData)) {
            gymppError << "Failed to load the gym plugin";
            return {};
        }

        // Create the server
        gymppDebug << "Creating the server" << std::endl << std::flush;
        std::unique_lock lock(pImpl->gui.mutex);
        m_server = std::make_unique<IgnitionServer>(pImpl->serverConfig);

        // Add the plugin system in the server
        m_server->AddSystem(pImpl->pluginData.systemPluginPtr);
    }

    if (pImpl->gui.thread) {
        // Run main window on a separate thread
        pImpl->gui.cv.notify_all();
    }

    if (!this->action_space->contains(action)) {
        gymppError << "The input action does not belong to the action space" << std::endl;
        return {};
    }

    // Set the action to the environment
    if (!pImpl->pluginData.behavior->setAction(action)) {
        gymppError << "Failed to set the action" << std::endl;
        return {};
    }

    // Run the server synchronously
    if (!m_server->Run(/*_blocking=*/true, pImpl->numOfIterations, /*_paused=*/false)) {
        gymppError << "The server couldn't execute the step" << std::endl;
        return {};
    }

    // Get the observation from the environment
    std::optional<Observation> observation = pImpl->pluginData.behavior->getObservation();

    if (!observation) {
        gymppError << "The gympp plugin didn't return the observation" << std::endl;
        return {};
    }

    if (!this->observation_space->contains(observation.value())) {
        gymppError << "The returned observation does not belong to the observation space"
                   << std::endl;
        return {};
    }

    // Get the reward from the environment
    std::optional<Reward> reward = pImpl->pluginData.behavior->computeReward();

    if (!reward) {
        gymppError << "The gympp plugin didn't return the reward" << std::endl;
        return {};
    }

    if (!this->reward_range.contains(reward.value())) {
        gymppError << "The returned reward (" << reward.value()
                   << ") does not belong to the reward space" << std::endl;
        return {};
    }

    return IgnitionGazebo::State{
        pImpl->pluginData.behavior->isDone(), {}, reward.value(), observation.value()};
}

std::vector<unsigned> IgnitionGazebo::seed(unsigned seed)
{
    if (seed != 0) {
        gympp::Random::setSeed(seed);
    }

    return {seed};
}

void IgnitionGazebo::setVerbosity(int level)
{
    ignition::common::Console::SetVerbosity(level);
}

bool IgnitionGazebo::loadSDF(std::string& sdfFile)
{
    if (sdfFile.empty()) {
        gymppError << "Passed SDF file is an empty string" << std::endl;
        return false;
    }

    if (!pImpl->serverConfig.SetSdfFile(sdfFile)) {
        std::cout << "Failed to set the SDF file " << sdfFile << std::endl;
        return false;
    }

    return true;
}

gympp::EnvironmentPtr IgnitionGazebo::env()
{
    return shared_from_this();
}

std::optional<IgnitionGazebo::Observation> IgnitionGazebo::reset()
{
    return {};
}

bool IgnitionGazebo::render(IgnitionGazebo::RenderMode mode)
{
    if (mode == IgnitionGazebo::RenderMode::HUMAN) {
        pImpl->gui.thread =
            std::make_unique<std::thread>(&IgnitionGuiData::spawnGui, std::ref(pImpl->gui));
        return true;
    }

    return false;
}
