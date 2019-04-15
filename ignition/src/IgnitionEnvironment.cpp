/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "gympp/gazebo/IgnitionEnvironment.h"
#include "gympp/Log.h"
#include "gympp/Random.h"
#include "gympp/gazebo/EnvironmentCallbacks.h"
#include "gympp/gazebo/EnvironmentCallbacksSingleton.h"
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
#include <vector>

using namespace gympp::gazebo;

class IgnitionEnvironment::Impl
{
public:
    std::unique_ptr<TinyProcessLib::Process> ignitionGui;

    uint64_t numOfIterations = 0;
    ignition::gazebo::ServerConfig serverConfig;
    std::shared_ptr<ignition::gazebo::Server> server;
    std::shared_ptr<ignition::gazebo::Server> getServer();

    gympp::gazebo::EnvironmentCallbacks* envCallbacks()
    {
        auto ecSingleton = EnvironmentCallbacksSingleton::Instance();
        auto* callbacks = ecSingleton->get(scopedModelName);
        assert(callbacks);
        return callbacks;
    }

    std::string scopedModelName;
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

        // Create the server
        gymppDebug << "Creating the server" << std::endl << std::flush;
        serverConfig.SetUseLevels(false);
        server = std::make_unique<ignition::gazebo::Server>(serverConfig);
        assert(server);

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

static size_t counter = 0;
bool IgnitionEnvironment::setupIgnitionPlugin(const std::string& libName,
                                              const std::string& className)
{
    auto uniqueID = counter++;
    pImpl->scopedModelName = pImpl->modelsNamesInSdf.front() + std::to_string(uniqueID);

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
    // Get the gazebo server
    auto server = pImpl->getServer();
    if (!server) {
        gymppError << "Failed to get the ignition server" << std::endl;
        return {};
    }

    assert(pImpl->server);
    assert(action_space);
    assert(observation_space);

    // Get the environment callbacks
    auto* envCallbacks = pImpl->envCallbacks();
    if (!envCallbacks) {
        gymppError << "Failed to get the environment callbacks from the plugin" << std::endl;
        return {};
    }

    if (!this->action_space->contains(action)) {
        gymppError << "The input action does not belong to the action space" << std::endl;
        return {};
    }

    // Set the action to the environment
    if (!envCallbacks->setAction(action)) {
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
    std::optional<Observation> observation = envCallbacks->getObservation();

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
    std::optional<Reward> reward = envCallbacks->computeReward();

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
        envCallbacks->isDone(), {}, reward.value(), observation.value()};
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

bool IgnitionEnvironment::setupGazeboWorld(const std::string& worldFile,
                                           const std::vector<std::string>& modelNames)
{
    // =================
    // LOAD THE SDF FILE
    // =================

    if (worldFile.empty()) {
        gymppError << "Passed SDF file argument is an empty string" << std::endl;
        return false;
    }

    // Find the file
    // TODO: add install directory of our world files
    ignition::common::SystemPaths systemPaths;
    systemPaths.SetFilePathEnv("IGN_GAZEBO_RESOURCE_PATH");
    systemPaths.AddFilePaths(IGN_GAZEBO_WORLD_INSTALL_DIR);
    std::string filePath = systemPaths.FindFile(worldFile);

    if (filePath.empty()) {
        gymppError << "Failed to find '" << worldFile << "'. "
                   << "Check that it's contained in the paths defined in IGN_GAZEBO_RESOURCE_PATH."
                   << std::endl;
        gymppError << "If you use the <include> element, make sure to add the parent folder of the "
                   << "<uri> in the SDF_PATH variable." << std::endl;
        return false;
    }

    if (!pImpl->serverConfig.SetSdfFile(filePath)) {
        gymppError << "Failed to set the SDF file " << worldFile << std::endl;
        return false;
    }

    // ======================================
    // LOAD A ROBOT PLUGIN FOR EACH SDF MODEL
    // ======================================

    // Store the model names
    // TODO: We should separate robot and environment. In this way we would have an sdf file for
    //       the robot that we can parse to get automatically the model names.
    pImpl->modelsNamesInSdf = modelNames;

    return true;
}

gympp::EnvironmentPtr IgnitionEnvironment::env()
{
    return shared_from_this();
}

std::optional<IgnitionEnvironment::Observation> IgnitionEnvironment::reset()
{
    gymppMessage << "Resetting the environment" << std::endl;

    // The plugin must be loaded in order to call its reset() method
    if (!pImpl->getServer()) {
        gymppError << "Failed to get the ignition server" << std::endl;
        return {};
    }

    // Get the environment callbacks
    auto* envCallbacks = pImpl->envCallbacks();
    if (!envCallbacks) {
        gymppError << "Failed to get the environment callbacks from the plugin" << std::endl;
        return {};
    }

    if (!envCallbacks->reset()) {
        gymppError << "Failed to reset plugin" << std::endl;
        return {};
    }

    gymppDebug << "Retrieving the initial observation after reset" << std::endl;
    return envCallbacks->getObservation();
}

bool IgnitionEnvironment::render(RenderMode mode)
{
    // If ign-gazebo-gui is already running, return without doing anything.
    int exit_status;
    if (pImpl->ignitionGui && !pImpl->ignitionGui->try_get_exit_status(exit_status)) {
        return true;
    }

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
