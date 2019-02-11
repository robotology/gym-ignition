#include "gympp/gyms/Ignition.h"
#include "gympp/Log.h"
#include "gympp/Random.h"
#include "process.hpp"

#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/ServerConfig.hh>
#include <ignition/gazebo/SystemLoader.hh>
#include <ignition/plugin/SpecializedPluginPtr.hh>

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <unistd.h>

using namespace gympp::gyms;

struct IgnitionGuiData
{
    bool render = false;
    std::unique_ptr<TinyProcessLib::Process> ignitionGui{nullptr};
};

struct PluginData
{
    std::string libName;
    std::string pluginName;

    EnvironmentBehavior* behavior = nullptr;
    ignition::gazebo::SystemPluginPtr systemPluginPtr;
};

class IgnitionGazebo::Impl
{
public:
    bool firstRun = true;
    std::string sdfFile;
    uint64_t numOfIterations = 0;

    IgnitionGuiData gui;
    PluginData pluginData;

    ignition::gazebo::ServerConfig serverConfig;
    std::unique_ptr<ignition::gazebo::Server> server = nullptr;

    bool initializeFirstRun();
    bool loadSDF(std::string& sdfFile);
    bool loadPlugin(PluginData& pluginData);
};

bool IgnitionGazebo::Impl::loadPlugin(PluginData& pluginData)
{
    ignition::gazebo::SystemLoader sl;
    auto plugin = sl.LoadPlugin(pluginData.libName, pluginData.pluginName, nullptr);

    if (!plugin.has_value()) {
        gymppError << "Failed to load plugin '" << pluginData.pluginName << "'" << std::endl;
        gymppError << "Make sure that the IGN_GAZEBO_SYSTEM_PLUGIN_PATH environment variable "
                   << "contains the path to the plugin '" << pluginData.libName << "'" << std::endl;
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

bool IgnitionGazebo::Impl::initializeFirstRun()
{
    firstRun = false;

    // Load the sdf file from the filesystem
    // TODO: get the absolute path with Ignition::Filesystem?
    if (!loadSDF(sdfFile)) {
        gymppError << "Failed to load the SDF";
        return false;
    }

    // Load the plugin
    if (!loadPlugin(pluginData)) {
        gymppError << "Failed to load the gym plugin";
        return false;
    }

    // Create the server
    gymppDebug << "Creating the server" << std::endl << std::flush;
    server = std::make_unique<ignition::gazebo::Server>(serverConfig);

    // Add the plugin system in the server
    auto ok = server->AddSystem(pluginData.systemPluginPtr);

    if (!(ok && ok.value())) {
        gymppError << "Failed to add the system in the gazebo server" << std::endl;
        return false;
    }

    // Initialize the server by running 1 iteration. The GUI needs the server already up.
    if (!server->Run(/*_blocking=*/true, 1, /*_paused=*/false)) {
        gymppError << "The gazebo server failed to run the first iteration" << std::endl;
        return false;
    }

    if (gui.render) {
        gui.ignitionGui = std::make_unique<TinyProcessLib::Process>("ign-gazebo-gui");
        // TODO: find a best way to wait that the gui is open
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    return true;
}

IgnitionGazebo::IgnitionGazebo(const ActionSpacePtr aSpace,
                               const ObservationSpacePtr oSpace,
                               const std::string& sdfFile,
                               double updateRate,
                               uint64_t iterations)
    : Environment(aSpace, oSpace)
    , pImpl{new IgnitionGazebo::Impl, [](Impl* impl) { delete impl; }}
{
    setVerbosity(4);
    pImpl->sdfFile = sdfFile;
    pImpl->numOfIterations = iterations;
    pImpl->serverConfig.SetUpdateRate(updateRate);
}

void IgnitionGazebo::setupIgnitionPlugin(const std::string& libName, const std::string& pluginName)
{
    pImpl->pluginData.libName = libName;
    pImpl->pluginData.pluginName = pluginName;
}

gympp::gyms::IgnitionGazebo::~IgnitionGazebo()
{
    if (pImpl->gui.ignitionGui) {
#if defined(WIN32) || defined(_WIN32)
        bool force = false;
#else
        bool force = true;
#endif
        pImpl->gui.ignitionGui->kill(force);
    }

    // TODO: for some reason when the main process is terminated after a SIGTERM,
    //       the server hangs forever during its destruction.
}

std::optional<IgnitionGazebo::State> IgnitionGazebo::step(const Action& action)
{
    if (pImpl->firstRun) {
        if (!pImpl->initializeFirstRun()) {
            gymppError << "Failed to initialize ignition gazebo" << std::endl;
            return {};
        }
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

    if (!pImpl->server->Run(/*_blocking=*/true, pImpl->numOfIterations, /*_paused=*/false)) {
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

bool IgnitionGazebo::Impl::loadSDF(std::string& sdfFile)
{
    if (sdfFile.empty()) {
        gymppError << "Passed SDF file is an empty string" << std::endl;
        return false;
    }

    if (!serverConfig.SetSdfFile(sdfFile)) {
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
    if (!pImpl->pluginData.behavior) {
        return {};
    }

    if (!pImpl->pluginData.behavior->reset()) {
        return {};
    }

    return pImpl->pluginData.behavior->getObservation();
}

bool IgnitionGazebo::render(IgnitionGazebo::RenderMode mode)
{
    if (mode == IgnitionGazebo::RenderMode::HUMAN) {
        pImpl->gui.render = true;
        return true;
    }

    return false;
}
