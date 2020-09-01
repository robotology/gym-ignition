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

#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/ServerConfig.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/Publisher.hh>
#include <sdf/Element.hh>
#include <sdf/Physics.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

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

    bool operator==(const PhysicsData& other)
    {
        return doubleEq(this->rtf, other.rtf)
               && doubleEq(this->maxStepSize, other.maxStepSize)
               && doubleEq(this->realTimeUpdateRate, other.realTimeUpdateRate);
    }

    static bool doubleEq(const double first, const double second)
    {
        return std::abs(first - second)
               < 10 * std::numeric_limits<double>::epsilon();
    }

    friend std::ostream& operator<<(std::ostream& out, const PhysicsData& data)
    {
        out << "max_step_size=" << data.maxStepSize << std::endl;
        out << "real_time_factor=" << data.rtf << std::endl;
        out << "real_time_update_rate=" << data.realTimeUpdateRate;
        return out;
    }
};

// ==============
// Implementation
// ==============

class GazeboSimulator::Impl
{
public:
    sdf::ElementPtr sdfElement = nullptr;

    struct
    {
        detail::PhysicsData physics;
        uint64_t numOfIterations = 0;
        std::unique_ptr<TinyProcessLib::Process> gui;
        std::shared_ptr<ignition::gazebo::Server> server;
    } gazebo;

    bool insertWorld(const sdf::World& world);
    std::shared_ptr<ignition::gazebo::Server> getServer();
    bool postProcessWorld(const std::string& worldName);

    using WorldName = std::string;
    using GazeboWorldPtr = std::shared_ptr<scenario::gazebo::World>;
    std::unordered_map<WorldName, GazeboWorldPtr> worlds;

    static detail::PhysicsData getPhysicsData(const sdf::Root& root,
                                              const size_t worldIndex);
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
}

GazeboSimulator::~GazeboSimulator()
{
    this->close();
}

double GazeboSimulator::stepSize() const
{
    return pImpl->gazebo.physics.maxStepSize;
}

double GazeboSimulator::realTimeFactor() const
{
    return pImpl->gazebo.physics.rtf;
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

    if (paused && !server->RunOnce(/*paused=*/true)) {
        sError << "The server couldn't execute the paused step" << std::endl;
        return false;
    }

    // Run the simulation
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

    std::vector<std::string> worldNames = this->worldNames();

    if (worldNames.empty()) {
        sError << "Failed to find any world in the simulator" << std::endl;
        return false;
    }

    // NOTE: we connect to the first world
    std::string worldName = worldNames[0];

    if (!pImpl->sceneBroadcasterActive(worldName)) {
        sDebug << "Starting the SceneBroadcaster plugin" << std::endl;
        auto world = this->getWorld(worldName);
        if (!std::static_pointer_cast<World>(world)->insertWorldPlugin(
                "ignition-gazebo-scene-broadcaster-system",
                "ignition::gazebo::systems::SceneBroadcaster")) {
            sError << "Failed to load SceneBroadcaster plugin" << std::endl;
        }
    }

    // Allow specifying a GUI verbosity different than the server verbosity
    int appliedVerbosity = verbosity;

    if (appliedVerbosity < 0) {
        // Get the verbosity level
        appliedVerbosity = ignition::common::Console::Verbosity();
    }

    // Spawn a new process with the GUI
    pImpl->gazebo.gui = std::make_unique<TinyProcessLib::Process>(
        "ign gazebo -g -v " + std::to_string(appliedVerbosity));

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

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
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

    size_t numOfWorlds = this->worldNames().size();

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

    return pImpl->insertWorld(world);
}

bool GazeboSimulator::insertWorldsFromSDF(
    const std::string& worldFile,
    const std::vector<std::string>& worldNames)
{
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

        if (!pImpl->insertWorld(thisWorld)) {
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

    // Get the existing world names
    const std::vector<std::string>& worldNames = this->worldNames();

    // Copy the world name since we'll try to automatically detect it
    std::string returnedWorldName = worldName;

    // In most cases there will be only one world. In this case we allow
    // omitting to specify the world name and automatically get it.
    if (worldName.empty()) {
        if (worldNames.size() == 1) {
            returnedWorldName = worldNames[0];
        }
        else {
            sError << "Found multiple worlds. "
                   << "You must specify the world name." << std::endl;
            return nullptr;
        }
    }

    // Return the cached world object if it exists
    if (pImpl->worlds.find(returnedWorldName) != pImpl->worlds.end()) {
        assert(pImpl->worlds.at(returnedWorldName));
        return pImpl->worlds.at(returnedWorldName);
    }

    if (std::find(worldNames.begin(), worldNames.end(), returnedWorldName)
        == worldNames.end()) {
        sError << "Failed to find world '" << returnedWorldName << "'"
               << std::endl;
        return nullptr;
    }

    auto& ecmSingleton = scenario::plugins::gazebo::ECMSingleton::Instance();

    if (!ecmSingleton.hasWorld(returnedWorldName)) {
        sError << "Failed to find world in the singleton" << std::endl;
        return nullptr;
    }

    if (!ecmSingleton.valid(returnedWorldName)) {
        sError << "Resources of world " << worldName << " not valid"
               << std::endl;
        return nullptr;
    }

    // Get the resources needed by the world
    auto* ecm = ecmSingleton.getECM(returnedWorldName);
    auto* eventManager = ecmSingleton.getEventManager(returnedWorldName);

    // Get the world entity
    auto worldEntity = ecm->EntityByComponents(
        ignition::gazebo::components::World(),
        ignition::gazebo::components::Name(returnedWorldName));

    // Create the world object
    pImpl->worlds[returnedWorldName] =
        std::make_shared<scenario::gazebo::World>();
    pImpl->worlds[returnedWorldName]->initialize(
        worldEntity, ecm, eventManager);
    pImpl->worlds[returnedWorldName]->createECMResources();

    assert(pImpl->worlds[returnedWorldName]->id() != 0);
    return pImpl->worlds[returnedWorldName];
}

// ==============
// Implementation
// ==============

bool GazeboSimulator::Impl::insertWorld(const sdf::World& world)
{
    if (!sdfElement) {
        sdfElement = sdf::SDF::WrapInRoot(world.Element()->Clone());
    }
    else {
        // Check that there are no worlds with the same name already stored
        auto root = utils::getSdfRootFromString(sdfElement->ToString(""));

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

        // There is no way yet to set the physics step size if not passing
        // through the physics element of the SDF. We update here the SDF
        // overriding the default profile.
        // NOTE: this could be avoided if gazebo::Server would expose the
        //       SimulationRunner::SetStepSize method.
        for (size_t worldIdx = 0; worldIdx < root.WorldCount(); ++worldIdx) {
            if (!utils::updateSDFPhysics(root,
                                         gazebo.physics.maxStepSize,
                                         gazebo.physics.rtf,
                                         /*realTimeUpdateRate=*/-1,
                                         worldIdx)) {
                sError << "Failed to set physics profile" << std::endl;
                return nullptr;
            }

            assert(Impl::getPhysicsData(root, worldIdx) == gazebo.physics);
        }

        sDebug << "Physics profile:" << std::endl
               << this->gazebo.physics << std::endl;

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

        // Create the server
        auto server = std::make_shared<ignition::gazebo::Server>(config);
        assert(server);

        sDebug << "Starting the gazebo server" << std::endl;

        if (!server->RunOnce(/*paused=*/true)) {
            sError << "Failed to initialize the first gazebo server run"
                   << std::endl;
            return nullptr;
        }

        for (size_t worldIdx = 0; worldIdx < root.WorldCount(); ++worldIdx) {
            // Get the world name
            const auto& worldName = root.WorldByIndex(worldIdx)->Name();

            // Post-process the world
            if (!this->postProcessWorld(worldName)) {
                sError << "Failed to post-process world " << worldName
                       << std::endl;
                return nullptr;
            }
        }

        // Store the server
        gazebo.server = server;
    }

    return gazebo.server;
}

bool GazeboSimulator::Impl::postProcessWorld(const std::string& worldName)
{
    auto& ecmSingeton = plugins::gazebo::ECMSingleton::Instance();

    if (!ecmSingeton.hasWorld(worldName)) {
        sError << "Failed to initialize ECMProvider" << std::endl;
        return false;
    }

    // Get the ECM
    auto* ecm = ecmSingeton.getECM(worldName);

    // Get the entity of the world
    auto worldEntity =
        ecm->EntityByComponents(ignition::gazebo::components::World(),
                                ignition::gazebo::components::Name(worldName));

    if (worldEntity == ignition::gazebo::kNullEntity) {
        sError << "Couldn't find world entity" << std::endl;
        return false;
    }

    // Insert a new component with the simulated time
    // (Physics will update it)
    ecm->CreateComponent(worldEntity,
                         ignition::gazebo::components::SimulatedTime(
                             std::chrono::steady_clock::duration::zero()));

    // Insert the time of creation of the world
    ecm->CreateComponent(worldEntity,
                         ignition::gazebo::components::Timestamp(
                             std::chrono::steady_clock::duration::zero()));

    return true;
}

detail::PhysicsData
GazeboSimulator::Impl::getPhysicsData(const sdf::Root& root,
                                      const size_t worldIndex)
{
    const sdf::World* world = root.WorldByIndex(worldIndex);
    assert(world->PhysicsCount() == 1);

    detail::PhysicsData physics;

    physics.rtf = world->PhysicsByIndex(0)->RealTimeFactor();
    physics.maxStepSize = world->PhysicsByIndex(0)->MaxStepSize();

    return physics;
}

bool GazeboSimulator::Impl::sceneBroadcasterActive(const std::string& worldName)
{
    ignition::transport::Node node;
    std::vector<ignition::transport::ServicePublisher> publishers;

    std::string serviceName{"/world/" + worldName + "/scene/info"};
    node.ServiceInfo(serviceName, publishers);

    return !publishers.empty();
}
