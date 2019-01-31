#include "gympp/gyms/Ignition.h"
#include "gympp/Log.h"

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

template class gympp::gyms::IgnitionGazebo<size_t, double>;
// template class gympp::gyms::IgnitionGazebo<double, double>;
// template class gympp::gyms::IgnitionGazebo<int, float>;

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

template <typename AType, typename OType>
class IgnitionGazebo<AType, OType>::Impl
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

template <typename AType, typename OType>
bool IgnitionGazebo<AType, OType>::Impl::loadPlugin(PluginData& pluginData)
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

template <typename AType, typename OType>
IgnitionGazebo<AType, OType>::IgnitionGazebo(const std::string libName,
                                             const std::string& pluginName,
                                             const ActionSpacePtr aSpace,
                                             const ObservationSpacePtr oSpace,
                                             const std::string& sdfFile,
                                             double updateRate,
                                             uint64_t iterations)
    : Environment(aSpace, oSpace)
    , pImpl{new IgnitionGazebo::Impl}
{
    setVerbosity(4);
    pImpl->sdfFile = sdfFile;
    pImpl->pluginData.libName = libName;
    pImpl->pluginData.pluginName = pluginName;
    pImpl->numOfIterations = iterations;
    pImpl->serverConfig.SetUpdateRate(updateRate);
}

template <typename AType, typename OType>
gympp::gyms::IgnitionGazebo<AType, OType>::~IgnitionGazebo() = default;

template <typename AType, typename OType>
std::optional<typename IgnitionGazebo<AType, OType>::State>
IgnitionGazebo<AType, OType>::step(const Action& action)
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

        // TODO: Configure the plugin. The server configures only plugins
        //       loaded from the sdf file
        //        pImpl->pluginData.systemPluginPtr->Configure();

        // Add the plugin's system in the server
        m_server->AddSystem(pImpl->pluginData.systemPluginPtr);
    }

    {
        //        std::unique_lock lock(pImpl->gui.mutex);
        //        if (pImpl->gui.thread && pImpl->gui.app) {
        if (pImpl->gui.thread) {
            // TODO: improve
            //            lock.unlock();

            // Run main window on a separate thread
            pImpl->gui.cv.notify_all();
        }
    }

    // Set the action to the environment
    if (!(this->action_space->contains(action) && action.get<AType>()
          && pImpl->pluginData.behavior->setAction(action))) {
        //          && pImpl->pluginData.behavior->setAction(*action.get<AType>()))) {
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

    if (!(observation && this->observation_space->contains(observation.value()))) {
        gymppError << "Failed to get the observation" << std::endl;
        return {};
    }

    // Get the reward from the environment
    std::optional<Reward> reward = pImpl->pluginData.behavior->computeReward();

    if (!(reward && this->reward_range.contains(reward.value()))) {
        gymppError << "Failed to compute reward" << std::endl;
        return {};
    }

    return {pImpl->pluginData.behavior->isDone(), {}, reward.value(), observation.value()};
}

template <typename AType, typename OType>
void IgnitionGazebo<AType, OType>::setVerbosity(int level)
{
    ignition::common::Console::SetVerbosity(level);
}

template <typename AType, typename OType>
bool IgnitionGazebo<AType, OType>::loadSDF(std::string& sdfFile)
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

template <typename AType, typename OType>
typename IgnitionGazebo<AType, OType>::Environment* IgnitionGazebo<AType, OType>::env()
{
    // TODO return a share pointer to itself. There is a pattern for sharing the memory
    // by inheriting from a shared ptr particular class
    //    return static_cast<IgnitionGazebo<AType, OType>>(this);
    return static_cast<Environment*>(this);
}

template <typename AType, typename OType>
std::optional<typename IgnitionGazebo<AType, OType>::Observation>
IgnitionGazebo<AType, OType>::reset()
{
    return {};
}

template <typename AType, typename OType>
bool IgnitionGazebo<AType, OType>::render(IgnitionGazebo::RenderMode mode)
{
    if (mode == IgnitionGazebo::RenderMode::HUMAN) {
        pImpl->gui.thread =
            std::make_unique<std::thread>(&IgnitionGuiData::spawnGui, std::ref(pImpl->gui));
        return true;
    }

    return false;
}
