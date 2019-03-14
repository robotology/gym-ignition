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
    std::string sdfFile;
    uint64_t numOfIterations = 0;

    std::unique_ptr<TinyProcessLib::Process> ignitionGui;
    PluginData pluginData;

    ignition::gazebo::ServerConfig serverConfig;
    std::shared_ptr<ignition::gazebo::Server> server;
    std::shared_ptr<ignition::gazebo::Server> getServer();

    bool loadSDF(std::string& sdfFile);
    bool loadPlugin(PluginData& pluginData);
};

std::shared_ptr<ignition::gazebo::Server> IgnitionGazebo::Impl::getServer()
{
    // Lazy initialization of the server
    if (!server) {
        // Load the sdf file from the filesystem
        if (!loadSDF(sdfFile)) {
            gymppError << "Failed to load the SDF";
            return nullptr;
        }

        // Load the plugin
        gymppDebug << "Loading the plugin with the environment behavior" << std::endl;
        if (!loadPlugin(pluginData)) {
            gymppError << "Failed to load the gym plugin";
            return nullptr;
        }

        // Create the server
        gymppDebug << "Creating the server" << std::endl << std::flush;
        server = std::make_unique<ignition::gazebo::Server>(serverConfig);
        assert(server);

        // Add the plugin system in the server
        // TODO: Configure() is not called in this way
        auto ok = server->AddSystem(pluginData.systemPluginPtr);

        if (!(ok && ok.value())) {
            gymppError << "Failed to add the system in the gazebo server" << std::endl;
            return {};
        }

        // The GUI needs the server already up. Warming up the first iteration and pausing the
        // server. It will be unpaused at the step() call.
        gymppDebug << "Starting the server as paused" << std::endl;
        if (!server->Run(/*blocking=*/false, 1, /*paused=*/true)) {
            gymppError << "Failed to warm up the gazebo server in paused state" << std::endl;
            return nullptr;
        }
    }

    return server;
}

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

    // Get the environment behavior interface out of it
    pluginData.behavior =
        pluginData.systemPluginPtr->template QueryInterface<gympp::gyms::EnvironmentBehavior>();

    if (!pluginData.behavior) {
        gymppError << "Failed to cast the plugin";
        return false;
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
    if (pImpl->ignitionGui) {
#if defined(WIN32) || defined(_WIN32)
        bool force = false;
#else
        bool force = true;
#endif
        pImpl->ignitionGui->kill(force);
    }
}

std::optional<IgnitionGazebo::State> IgnitionGazebo::step(const Action& action)
{
    auto server = pImpl->getServer();
    if (!server) {
        gymppError << "Failed to get the ignition server" << std::endl;
        return {};
    }

    assert(pImpl->server);
    assert(action_space);
    assert(observation_space);
    assert(pImpl->pluginData.behavior);

    if (!this->action_space->contains(action)) {
        gymppError << "The input action does not belong to the action space" << std::endl;
        return {};
    }

    // Set the action to the environment
    if (!pImpl->pluginData.behavior->setAction(action)) {
        gymppError << "Failed to set the action" << std::endl;
        return {};
    }

    if (server->Running()) {
        gymppDebug << "Unpausing the server. Running the first simulation run." << std::endl;
        server->SetPaused(false);

        // Since the server was started in non-blocking mode, we have to wait that this first
        // iteration finishes
        while (server->Running()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    else {
        if (!server->Run(/*_blocking=*/true, pImpl->numOfIterations, /*_paused=*/false)) {
            gymppError << "The server couldn't execute the step" << std::endl;
            return {};
        }
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
        gymppError << "Failed to set the SDF file " << sdfFile << std::endl;
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
        gymppError << "The plugin has not been initialized" << std::endl;
        return {};
    }

    if (!pImpl->pluginData.behavior->reset()) {
        gymppError << "Failed to reset plugin" << std::endl;
        return {};
    }

    return pImpl->pluginData.behavior->getObservation();
}

bool IgnitionGazebo::render(RenderMode mode)
{
    if (mode == RenderMode::HUMAN) {
        // The GUI needs the ignition server running. Initialize it.
        if (!pImpl->getServer()) {
            gymppError << "Failed to get the ignition server" << std::endl;
            return false;
        }

        // Spawn a new process with the GUI
        pImpl->ignitionGui = std::make_unique<TinyProcessLib::Process>("ign-gazebo-gui");

        return true;
    }

    return false;
}
