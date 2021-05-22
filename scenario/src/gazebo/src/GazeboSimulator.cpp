/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This project is dual licensed under LGPL v2.1+ or Apache License.
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "scenario/gazebo/GazeboSimulator.h"
#include "process.hpp"
#include "scenario/core/utils/signals.h"
#include "scenario/gazebo/Log.h"
#include "scenario/gazebo/World.h"
#include "scenario/gazebo/components/SimulatedTime.h"
#include "scenario/gazebo/components/Timestamp.h"
#include "scenario/gazebo/helpers.h"
#include "scenario/gazebo/utils.h"
#include "scenario/plugins/gazebo/ECMSingleton.h"

#include <ignition/fuel_tools.hh>
#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/ServerConfig.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Physics.hh>
#include <ignition/gazebo/components/PhysicsCmd.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/Publisher.hh>
#include <sdf/sdf.hh>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <csignal>
#include <limits>
#include <optional>
#include <thread>
#include <unordered_map>

using namespace scenario::gazebo;

namespace scenario::gazebo::detail {
    struct PhysicsData;
} // namespace scenario::gazebo::detail

struct detail::PhysicsData
{
    double rtf = -1;
    double maxStepSize = -1;
    double realTimeUpdateRate = -1;

    bool valid() const { return this->rtf > 0 && this->maxStepSize > 0; }
};

// ==============
// Implementation
// ==============

class GazeboSimulator::Impl
{
public: // attributes
    sdf::ElementPtr sdfElement = nullptr;

    struct
    {
        detail::PhysicsData physics;
        uint64_t numOfIterations = 0;
        std::unique_ptr<TinyProcessLib::Process> gui;
        std::shared_ptr<ignition::gazebo::Server> server;
    } gazebo;

    using WorldName = std::string;
    using GazeboWorldPtr = std::shared_ptr<scenario::gazebo::World>;
    std::unordered_map<WorldName, GazeboWorldPtr> worlds;

public: // methods
    bool insertSDFWorld(const sdf::World& world);
    std::shared_ptr<ignition::gazebo::Server> getServer();

    static std::shared_ptr<World>
    CreateGazeboWorld(const std::string& worldName);

    bool sceneBroadcasterActive(const std::string& worldName);
};

// ===============
// GazeboSimulator
// ===============

GazeboSimulator::GazeboSimulator(const double stepSize,
                                 const double rtf,
                                 const size_t stepsPerRun)
    : pImpl{std::make_unique<GazeboSimulator::Impl>()}
{
    // Configure gazebo
    pImpl->gazebo.numOfIterations = stepsPerRun;

    // Configure the physics profile
    pImpl->gazebo.physics.rtf = rtf;
    pImpl->gazebo.physics.maxStepSize = stepSize;

    // Configure Fuel Callback
    sdf::setFindCallback([](const std::string& uri) -> std::string {
        auto fuelClient = ignition::fuel_tools::FuelClient();
        const auto path =
            ignition::fuel_tools::fetchResourceWithClient(uri, fuelClient);
        return path;
    });
}

GazeboSimulator::~GazeboSimulator()
{
    this->close();
}

double GazeboSimulator::stepSize() const
{
    if (!this->initialized()) {
        return pImpl->gazebo.physics.maxStepSize;
    }

    // Get the first world
    const auto world = this->getWorld(this->worldNames().front());

    // Get the active physics parameters
    const auto& physics = utils::getExistingComponentData< //
        ignition::gazebo::components::Physics>(world->ecm(), world->entity());

    return physics.MaxStepSize();
}

double GazeboSimulator::realTimeFactor() const
{
    if (!this->initialized()) {
        return pImpl->gazebo.physics.rtf;
    }

    // Get the first world
    const auto world = this->getWorld(this->worldNames().front());

    // Get the active physics parameters
    const auto& physics = utils::getExistingComponentData< //
        ignition::gazebo::components::Physics>(world->ecm(), world->entity());

    return physics.RealTimeFactor();
}

size_t GazeboSimulator::stepsPerRun() const
{
    return pImpl->gazebo.numOfIterations;
}

bool GazeboSimulator::initialize()
{
    if (this->initialized()) {
        sMessage << "The simulator is already initialized" << std::endl;
        return true;
    }

    // Initialize the server
    if (!pImpl->getServer()) {
        sError << "Failed to get the Gazebo server" << std::endl;
        return false;
    }

    auto cb = [&](int sig) {
        this->close();
        exit(sig);
    };

    // Setup signals callbacks.
    // It must be done after the creation of the simulator since
    // we override their callbacks.
    core::utils::SignalManager::Instance().setCallback(SIGINT, cb);
    core::utils::SignalManager::Instance().setCallback(SIGTERM, cb);
    core::utils::SignalManager::Instance().setCallback(SIGABRT, cb);

    return true;
}

bool GazeboSimulator::initialized() const
{
    return bool(pImpl->gazebo.server);
}

bool GazeboSimulator::run(const bool paused)
{
    if (!this->initialized()) {
        sError << "The simulator was not initialized" << std::endl;
        return false;
    }

    // Get the gazebo server
    auto server = pImpl->getServer();
    if (!server) {
        sError << "Failed to get the ignition server" << std::endl;
        return false;
    }

    // If the server was configured to run in background (iterations = 0)
    // only the first call to this run method should trigger the start of
    // the simulation in non-blocking mode.
    // NOTE: non-blocking implementation is partial and not supported
    bool deterministic = pImpl->gazebo.numOfIterations != 0 ? true : false;

    if (!deterministic && server->Running()) {
        sWarning << "The server is already running in background" << std::endl;
        return true;
    }

    size_t iterations = pImpl->gazebo.numOfIterations;

    // Allow executing a single paused step in non-blocking mode,
    // allowing to refresh the visualized world state
    if (!deterministic && !server->Running() && paused) {
        deterministic = true;
        iterations = 1;
    }

    // Recent versions of Ignition Gazebo optimize the streaming of pose updates
    // in order to reduce the bandwidth between server and client.
    // However, it does not take into account paused steps.
    // Here below we force all the Pose components to be streamed by manually
    // setting them as changed.
    if (paused) {
        // Get the singleton
        auto& ecmSingleton =
            scenario::plugins::gazebo::ECMSingleton::Instance();

        // Process all worlds
        for (const auto& worldName : this->worldNames()) {
            assert(ecmSingleton.hasWorld(worldName));
            assert(ecmSingleton.valid(worldName));
            assert(ecmSingleton.getECM(worldName));

            // Get the ECM
            auto* ecm = ecmSingleton.getECM(worldName);

            // Mark all all entities with Pose component as Changed
            ecm->Each<ignition::gazebo::components::Pose>(
                [&](const ignition::gazebo::Entity& entity,
                    ignition::gazebo::components::Pose*) -> bool {
                    ecm->SetChanged(
                        entity,
                        ignition::gazebo::components::Pose::typeId,
                        ignition::gazebo::ComponentState::OneTimeChange);
                    return true;
                });
        }
    }

    // Paused simulation run
    if (paused && !server->RunOnce(/*paused=*/true)) {
        sError << "The server couldn't execute the paused step" << std::endl;
        return false;
    }

    // Unpaused simulation run
    if (!paused
        && !server->Run(/*blocking=*/deterministic,
                        /*iterations=*/iterations,
                        /*paused=*/false)) {
        sError << "The server couldn't execute the step" << std::endl;
        return false;
    }

    return true;
}

bool GazeboSimulator::gui(const int verbosity)
{
    if (!this->initialized()) {
        sError << "The simulator was not initialized" << std::endl;
        return false;
    }

    // If ign-gazebo-gui is already running, return without doing anything
    int exit_status;
    if (pImpl->gazebo.gui
        && !pImpl->gazebo.gui->try_get_exit_status(exit_status)) {
        return true;
    }

    const std::vector<std::string>& worldNames = this->worldNames();

    if (worldNames.empty()) {
        sError << "Failed to find any world in the simulator" << std::endl;
        return false;
    }

    // NOTE: we connect to the first world
    const std::string& worldName = worldNames[0];

    if (!pImpl->sceneBroadcasterActive(worldName)) {
        sDebug << "Starting the SceneBroadcaster plugin" << std::endl;
        auto world = this->getWorld(worldName);
        if (!std::static_pointer_cast<World>(world)->insertWorldPlugin(
                "ignition-gazebo-scene-broadcaster-system",
                "ignition::gazebo::systems::SceneBroadcaster")) {
            sError << "Failed to load SceneBroadcaster plugin" << std::endl;
            return false;
        }
    }

    // Allow specifying a GUI verbosity different than the server verbosity
    int guiVerbosity = verbosity;

    if (guiVerbosity < 0) {
        // Get the verbosity level
        guiVerbosity = ignition::common::Console::Verbosity();
    }

#if defined(WIN32) || defined(_WIN32)
    const std::string redirect = "";
#else
    // Suppress GUI stderr.
    // Recent versions of the GUI segfault printing an annoying stacktrace.
    const std::string redirect = guiVerbosity >= 4 ? "" : " 2>/dev/null";
#endif

    // Spawn a new process with the GUI
    pImpl->gazebo.gui = std::make_unique<TinyProcessLib::Process>(
        "ign gazebo -g -v " + std::to_string(guiVerbosity) + redirect);

    bool guiServiceExists = false;
    ignition::transport::Node node;
    std::vector<std::string> serviceList;

    do {
        sDebug << "Waiting GUI to show up... " << std::endl;
        node.ServiceList(serviceList);

        for (const auto& serviceName : serviceList) {
            if (serviceName.find("/gui/") == 0) {
                guiServiceExists = true;
                break;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } while (!guiServiceExists);

    sDebug << "GUI up and running" << std::endl;
    return true;
}

bool GazeboSimulator::close()
{
    if (pImpl->gazebo.gui) {
#if defined(WIN32) || defined(_WIN32)
        const bool force = false;
#else
        const bool force = true;
#endif
        pImpl->gazebo.gui->kill(force);
    }

    // Pause the simulator before tearing it down
    if (pImpl->gazebo.server && this->running()) {
        this->pause();
    }

    // Remove the resources of the handled worlds from the singleton
    if (this->initialized()) {
        try {
            for (const auto& worldName : this->worldNames()) {
                plugins::gazebo::ECMSingleton::Instance().clean(worldName);
            }
        }
        // This happens while tearing down everything. The ECMProvider plugin
        // sometimes is destroyed before the simulator.
        catch (std::runtime_error) {
            sWarning << "Failed to clean the singleton from the worlds"
                     << std::endl;
        }
    }

    // Delete the simulator
    pImpl->gazebo.server.reset();

    return true;
}

bool GazeboSimulator::pause()
{
    if (!this->initialized()) {
        sMessage << "Couldn't pause the simulator, it was never initialized"
                 << std::endl;
        return true;
    }

    if (!this->running()) {
        sMessage << "The simulation is already paused" << std::endl;
        return true;
    }

    const size_t numOfWorlds = this->worldNames().size();

    for (unsigned worldIdx = 0; worldIdx < numOfWorlds; ++worldIdx) {
        pImpl->getServer()->SetPaused(true, worldIdx);
    }

    return !this->running();
}

bool GazeboSimulator::running() const
{
    if (!this->initialized()) {
        sMessage << "The simulator was not initialized" << std::endl;
        return false;
    }

    return pImpl->gazebo.server->Running();
}

bool GazeboSimulator::insertWorldFromSDF(const std::string& worldFile,
                                         const std::string& worldName)
{
    if (this->initialized()) {
        sError << "Worlds must be inserted before the initialization"
               << std::endl;
        return false;
    }

    std::shared_ptr<sdf::Root> sdfRoot = nullptr;

    if (!worldFile.empty()) {
        // Load the world file
        sdfRoot = utils::getSdfRootFromFile(worldFile);
    }
    else {
        sMessage << "No world file passed, using the default empty world"
                 << std::endl;
        sdfRoot = utils::getSdfRootFromString(utils::getEmptyWorld());
    }

    if (!sdfRoot) {
        // Error printed in the function call
        return false;
    }

    if (sdfRoot->WorldCount() != 1) {
        sError << "The world file has more than one world" << std::endl;
        return false;
    }

    sdf::World world = *const_cast<sdf::World*>(sdfRoot->WorldByIndex(0));

    if (!worldName.empty()) {
        // Rename the world and get the new World pointer
        world = utils::renameSDFWorld(world, worldName);

        if (world.Name() != worldName) {
            return false;
        }
    }

    return pImpl->insertSDFWorld(world);
}

bool GazeboSimulator::insertWorldsFromSDF(
    const std::string& worldFile,
    const std::vector<std::string>& worldNames)
{
    if (this->initialized()) {
        sMessage << "Worlds must be inserted before the initialization"
                 << std::endl;
        return false;
    }

    // Load the world file
    auto sdfRoot = utils::getSdfRootFromFile(worldFile);

    if (!sdfRoot) {
        // Error printed in the function call
        return false;
    }

    if (sdfRoot->WorldCount() == 0) {
        sError << "Failed to find any world in the SDF file" << std::endl;
        return false;
    }

    if (!worldNames.empty() && sdfRoot->WorldCount() != worldNames.size()) {
        sError << "The number of world names does not match the number of "
               << "worlds found in the SDF file" << std::endl;
        return false;
    }

    for (size_t worldIdx = 0; worldIdx < sdfRoot->WorldCount(); ++worldIdx) {

        // Get the world
        sdf::World thisWorld =
            *const_cast<sdf::World*>(sdfRoot->WorldByIndex(worldIdx));

        // Optionally rename it
        if (!worldNames.empty()) {
            const std::string& worldName = worldNames[worldIdx];
            thisWorld = utils::renameSDFWorld(thisWorld, worldName);

            if (thisWorld.Name() != worldName) {
                return false;
            }
        }

        if (!pImpl->insertSDFWorld(thisWorld)) {
            sError << "Failed to insert world " << thisWorld.Name()
                   << std::endl;
            return false;
        }
    }

    return true;
}

std::vector<std::string> GazeboSimulator::worldNames() const
{
    if (!this->initialized()) {
        sError << "The simulator was not initialized" << std::endl;
        return {};
    }

    if (!scenario::plugins::gazebo::ECMSingleton::Instance().valid()) {
        throw std::runtime_error("The ECM singleton is not valid");
    }

    return scenario::plugins::gazebo::ECMSingleton::Instance().worldNames();
}

std::shared_ptr<scenario::gazebo::World>
GazeboSimulator::getWorld(const std::string& worldName) const
{
    if (!this->initialized()) {
        sError << "The simulator was not initialized" << std::endl;
        return nullptr;
    }

    // Get the existing world names.
    // In most cases there will be only one world. In this case we allow
    // omitting to specify the world name and automatically get the default.
    const std::vector<std::string>& worldNames = this->worldNames();

    if (worldName.empty() && worldNames.size() == 0) {
        sError << "The simulator does not have any world" << std::endl;
        return nullptr;
    }

    if (worldName.empty() && worldNames.size() > 1) {
        sError << "Found multiple worlds. You must specify the world name."
               << std::endl;
        return nullptr;
    }

    // Either use the method argument or the default world
    const std::string nameOfReturnedWorld =
        worldName.empty() ? worldNames[0] : worldName;

    // All the worlds objects are created and cached during the simulator
    // initialization. If there is no world, something has gone wrong.
    if (pImpl->worlds.find(nameOfReturnedWorld) == pImpl->worlds.end()) {
        sError << "World " << nameOfReturnedWorld << " not found" << std::endl;
        return nullptr;
    }

    assert(pImpl->worlds.at(nameOfReturnedWorld));
    return pImpl->worlds.at(nameOfReturnedWorld);
}

// ==============
// Implementation
// ==============

bool GazeboSimulator::Impl::insertSDFWorld(const sdf::World& world)
{
    if (!sdfElement) {
        sdfElement = sdf::SDF::WrapInRoot(world.Element()->Clone());
    }
    else {
        // Check that there are no worlds with the same name already stored
        const auto root = utils::getSdfRootFromString(sdfElement->ToString(""));

        if (!root) {
            return false;
        }

        if (root->WorldNameExists(world.Name())) {
            sError << "Another world with name " << world.Name()
                   << " already exists" << std::endl;
            return false;
        }

        // Insert the new world in the DOM
        sdfElement->InsertElement(world.Element()->Clone());
    }

    return true;
}

std::shared_ptr<ignition::gazebo::Server> GazeboSimulator::Impl::getServer()
{
    // Lazy initialization of the server
    if (!gazebo.server) {

        if (gazebo.numOfIterations == 0) {
            sError << "Non-deterministic mode (iterations=0) is not "
                   << "currently supported" << std::endl;
            return nullptr;
        }

        sdf::Root root;

        if (!sdfElement) {
            sMessage << "Using default empty world" << std::endl;
            auto errors = root.LoadSdfString(utils::getEmptyWorld());
            assert(errors.empty()); // TODO
        }
        else {
            auto errors = root.LoadSdfString(sdfElement->ToString(""));
            assert(errors.empty()); // TODO
        }

        if (root.WorldCount() == 0) {
            sError << "Failed to find a world in the SDF root" << std::endl;
            return nullptr;
        }

        // Get the plugin info of the ECM provider
        auto getECMPluginInfo = [](const std::string& worldName) {
            ignition::gazebo::ServerConfig::PluginInfo pluginInfo;
            pluginInfo.SetFilename("ECMProvider");
            pluginInfo.SetName("scenario::plugins::gazebo::ECMProvider");
            pluginInfo.SetEntityType("world");
            pluginInfo.SetEntityName(worldName);

            return pluginInfo;
        };

        // Check if there are sdf parsing errors
        assert(utils::sdfStringValid(root.Element()->Clone()->ToString("")));

        if (utils::verboseFromEnvironment()) {
            sDebug << "Loading the following SDF file in the gazebo server:"
                   << std::endl;
            std::cout << root.Element()->ToString("") << std::endl;
        }

        ignition::gazebo::ServerConfig config;

        config.SetSeed(0);
        config.SetUseLevels(false);
        config.SetSdfString(root.Element()->ToString(""));

        // Add the ECMProvider plugin for all worlds
        for (size_t worldIdx = 0; worldIdx < root.WorldCount(); ++worldIdx) {
            auto worldName = root.WorldByIndex(worldIdx)->Name();
            config.AddPlugin(getECMPluginInfo(worldName));
        }

        // Create the server.
        // The worlds are initialized with the physics parameters
        // (rtf and physics step) defined in the SDF. They get overridden below.
        auto server = std::make_shared<ignition::gazebo::Server>(config);
        assert(server);

        sDebug << "Starting the gazebo server" << std::endl;

        if (!server->RunOnce(/*paused=*/true)) {
            sError << "Failed to initialize the first gazebo server run"
                   << std::endl;
            return nullptr;
        }

        if (!gazebo.physics.valid()) {
            sError << "The physics parameters are not valid" << std::endl;
            return nullptr;
        }

        // Get the ECM singleton
        auto& singleton = scenario::plugins::gazebo::ECMSingleton::Instance();

        // Set the Physics parameters.
        // Note: all worlds must share the same parameters.
        for (const auto& worldName : singleton.worldNames()) {

            // Get the resources needed by the world
            auto* ecm = singleton.getECM(worldName);

            // Get the world entity
            const auto worldEntity = ecm->EntityByComponents(
                ignition::gazebo::components::World(),
                ignition::gazebo::components::Name(worldName));

            // Create a new PhysicsCmd component
            auto& physics = utils::getComponentData< //
                ignition::gazebo::components::PhysicsCmd>(ecm, worldEntity);

            // Store the physics parameters.
            // They are processed the next simulator step.
            physics.set_max_step_size(gazebo.physics.maxStepSize);
            physics.set_real_time_factor(gazebo.physics.rtf);
            physics.set_real_time_update_rate(
                gazebo.physics.realTimeUpdateRate);
        }

        // Step the server to process the physics parameters.
        // This call executes SimulationRunner::SetStepSize, updating the
        // rate at which all systems are called.
        // Note: it processes only the parameters of the first world.
        if (!server->RunOnce(/*paused=*/true)) {
            sError << "Failed to step the server to configure the physics"
                   << std::endl;
            return nullptr;
        }

        for (size_t worldIdx = 0; worldIdx < root.WorldCount(); ++worldIdx) {
            // Get the world name
            const auto& worldName = root.WorldByIndex(worldIdx)->Name();

            sDebug << "Creating and caching World '" << worldName << "'"
                   << std::endl;

            // Create the world object.
            // Note: performing this operation is important because the World
            //       objects are created and cached. During the first
            //       initialization, the World objects create important
            //       componentes like Timestamp and SimulatedTime.
            const auto& world = Impl::CreateGazeboWorld(worldName);

            if (!(world && world->valid())) {
                sError << "Failed to create world " << worldName << std::endl;
                return nullptr;
            }

            // Cache the world object
            assert(this->worlds.find(worldName) == this->worlds.end());
            this->worlds[worldName] = world;
        }

        // Store the server
        gazebo.server = server;
    }

    return gazebo.server;
}

std::shared_ptr<World>
GazeboSimulator::Impl::CreateGazeboWorld(const std::string& worldName)
{
    auto& ecmSingleton = scenario::plugins::gazebo::ECMSingleton::Instance();

    if (!ecmSingleton.hasWorld(worldName)) {
        sError << "Failed to find world in the singleton" << std::endl;
        return nullptr;
    }

    if (!ecmSingleton.valid(worldName)) {
        sError << "Resources of world " << worldName << " not valid"
               << std::endl;
        return nullptr;
    }

    // Get the resources needed by the world
    auto* ecm = ecmSingleton.getECM(worldName);
    auto* eventManager = ecmSingleton.getEventManager(worldName);

    // Get the world entity
    const auto worldEntity =
        ecm->EntityByComponents(ignition::gazebo::components::World(),
                                ignition::gazebo::components::Name(worldName));

    // Create the world object
    auto world = std::make_shared<scenario::gazebo::World>();

    if (!world->initialize(worldEntity, ecm, eventManager)) {
        sError << "Failed to initialize the world" << std::endl;
        return nullptr;
    }

    if (!world->createECMResources()) {
        sError << "Failed to initialize the ECM world resources" << std::endl;
        return nullptr;
    }

    if (world->id() == 0) {
        sError << "The id of the world is 0. Something went wrong" << std::endl;
        return nullptr;
    }

    return world;
}

bool GazeboSimulator::Impl::sceneBroadcasterActive(const std::string& worldName)
{
    ignition::transport::Node node;
    std::vector<ignition::transport::ServicePublisher> publishers;

    std::string serviceName{"/world/" + worldName + "/scene/info"};
    node.ServiceInfo(serviceName, publishers);

    return !publishers.empty();
}
