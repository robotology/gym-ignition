#include "gympp/gazebo/IgnitionEnvironment.h"
#include "gympp/Log.h"
#include "gympp/Random.h"
#include "process.hpp"

#include <ignition/common/SystemPaths.hh>
#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/ServerConfig.hh>
#include <ignition/gazebo/SystemLoader.hh>
#include <ignition/plugin/SpecializedPluginPtr.hh>
#include <sdf/Element.hh>
//#include <sdf/Root.hh>

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace gympp::gazebo;

struct PluginData
{
    std::string libName;
    std::string pluginName;

    EnvironmentBehavior* behavior = nullptr;
    ignition::gazebo::SystemPluginPtr systemPluginPtr;
};

class IgnitionEnvironment::Impl
{
public:
    uint64_t numOfIterations = 0;

    PluginData pluginData;
    std::unique_ptr<TinyProcessLib::Process> ignitionGui;

    ignition::gazebo::ServerConfig serverConfig;
    std::shared_ptr<ignition::gazebo::Server> server;
    std::shared_ptr<ignition::gazebo::Server> getServer();

    std::vector<std::string> modelsNamesInSdf;
};

std::shared_ptr<ignition::gazebo::Server> IgnitionEnvironment::Impl::getServer()
{
    // Lazy initialization of the server
    if (!server) {

        if (serverConfig.SdfFile().empty() && serverConfig.SdfString().empty()) {
            gymppError << "The sdf file was not configured" << std::endl;
            return nullptr;
        }

        // Check that the ignition plugin was configured and loaded
        if (!pluginData.behavior || !pluginData.systemPluginPtr) {
            gymppError << "The ignition plugin has not been correctly loaded" << std::endl;
            return nullptr;
        }

        // Create the server
        gymppDebug << "Creating the server" << std::endl << std::flush;
        serverConfig.SetUseLevels(false);
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
        if (!server->Run(/*blocking=*/false, numOfIterations, /*paused=*/true)) {
            gymppError << "Failed to warm up the gazebo server in paused state" << std::endl;
            return nullptr;
        }
    }

    return server;
}

// ===============
// IGNITION GAZEBO
// ===============

IgnitionEnvironment::IgnitionEnvironment(const ActionSpacePtr aSpace,
                                         const ObservationSpacePtr oSpace,
                                         double updateRate,
                                         uint64_t iterations)
    : Environment(aSpace, oSpace)
    , pImpl{new IgnitionEnvironment::Impl, [](Impl* impl) { delete impl; }}
{
    setVerbosity(4);
    //    pImpl->sdfFile = sdfFile;
    pImpl->numOfIterations = iterations;
    pImpl->serverConfig.SetUpdateRate(updateRate);
}

bool IgnitionEnvironment::setupIgnitionPlugin(const std::string& libName,
                                              const std::string& pluginName)
{
    pImpl->pluginData.libName = libName;
    pImpl->pluginData.pluginName = pluginName;

    auto& pluginData = pImpl->pluginData;

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
        pluginData.systemPluginPtr->template QueryInterface<EnvironmentBehavior>();

    if (!pluginData.behavior) {
        gymppError << "Failed to cast the plugin '" << pluginName
                   << "'to get the environment behavior interface" << std::endl;
        return false;
    }

    return true;
}

IgnitionEnvironment::~IgnitionEnvironment()
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

std::optional<IgnitionEnvironment::State> IgnitionEnvironment::step(const Action& action)
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

    return IgnitionEnvironment::State{
        pImpl->pluginData.behavior->isDone(), {}, reward.value(), observation.value()};
}

std::vector<size_t> IgnitionEnvironment::seed(size_t seed)
{
    if (seed != 0) {
        gympp::Random::setSeed(seed);
    }

    return {seed};
}

void IgnitionEnvironment::setVerbosity(int level)
{
    ignition::common::Console::SetVerbosity(level);
}

bool IgnitionEnvironment::setupSdf(const std::string& sdfFile,
                                   const std::vector<std::string>& modelNames)
{
    // =================
    // LOAD THE SDF FILE
    // =================

    if (sdfFile.empty()) {
        gymppError << "Passed SDF file argument is an empty string" << std::endl;
        return false;
    }

    // Find the file
    // TODO: add install directory of our world files
    ignition::common::SystemPaths systemPaths;
    systemPaths.SetFilePathEnv("IGN_GAZEBO_RESOURCE_PATH");
    systemPaths.AddFilePaths(IGN_GAZEBO_WORLD_INSTALL_DIR);
    std::string filePath = systemPaths.FindFile(sdfFile);

    if (filePath.empty()) {
        gymppError << "Failed to find '" << sdfFile << "'. "
                   << "Check that it's contained in the paths defined in IGN_GAZEBO_RESOURCE_PATH."
                   << std::endl;
        gymppError << "If you use the <include> element, make sure to add the parent folder of the "
                   << "<uri> in the SDF_PATH variable." << std::endl;
        return false;
    }

    if (!pImpl->serverConfig.SetSdfFile(filePath)) {
        gymppError << "Failed to set the SDF file " << sdfFile << std::endl;
        return false;
    }

    // ======================================
    // LOAD A ROBOT PLUGIN FOR EACH SDF MODEL
    // ======================================

    // TODO: In the future we might want to parse here the sdf file and get automatically
    //       all the names of the contained models. Right now we could make it work only
    //       if models are not included externally using <include><uri> sdf elements.
    //       Using the passed vector of names waiting this support.
    if (modelNames.empty()) {
        gymppError << "The sdf world must contain at least one model" << std::endl;
        return false;
    }

    // Add an IgnitionRobot plugin for each model in the sdf file
    // TODO: the lib name works only in linux!
    for (const auto& modelName : modelNames) {
        sdf::ElementPtr sdf(new sdf::Element);
        sdf->SetName("plugin");
        sdf->AddAttribute("name", "string", "gympp::gazebo::IgnitionRobot", true);
        sdf->AddAttribute("filename", "string", "libIgnitionRobot.so", true);

        ignition::gazebo::ServerConfig::PluginInfo pluginInfo{
            modelName, "model", "libIgnitionRobot.so", "gympp::gazebo::IgnitionRobot", sdf};
        pImpl->serverConfig.AddPlugin(pluginInfo);
    }

    //    // Get the models names included in the sdf file
    //    sdf::Root root;
    //    auto errors = root.Load(pImpl->serverConfig.SdfFile());

    //    if (!errors.empty()) {
    //        gymppError << "Failed to load sdf file '" << sdfFile << "." << std::endl;
    //        for (const auto& error : errors) {
    //            gymppError << error << std::endl;
    //        }
    //        return false;
    //    }

    //    for (unsigned i = 0; i < root.ModelCount(); ++i) {
    //        std::string modelName = root.ModelByIndex(i)->Name();
    //        gymppDebug << "Found model '" << modelName << "' in the sdf file" << std::endl;
    //        pImpl->modelsNamesInSdf.push_back(modelName);
    //    }

    return true;
}

gympp::EnvironmentPtr IgnitionEnvironment::env()
{
    return shared_from_this();
}

std::optional<IgnitionEnvironment::Observation> IgnitionEnvironment::reset()
{
    gymppDebug << "Resetting the environment" << std::endl;

    // The plugin must be loaded in order to call its reset() method
    if (!pImpl->getServer()) {
        gymppError << "Failed to get the ignition server" << std::endl;
        return {};
    }

    if (!pImpl->pluginData.behavior) {
        gymppError << "The plugin has not been initialized" << std::endl;
        return {};
    }

    if (!pImpl->pluginData.behavior->reset()) {
        gymppError << "Failed to reset plugin" << std::endl;
        return {};
    }

    gymppDebug << "Retrieving the initial observation after reset" << std::endl;
    return pImpl->pluginData.behavior->getObservation();
}

bool IgnitionEnvironment::render(RenderMode mode)
{
    gymppDebug << "Rendering the environment" << std::endl;

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
