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
#include "scenario/gazebo/Log.h"
#include "scenario/gazebo/World.h"
#include "scenario/gazebo/components/SimulatedTime.h"
#include "scenario/gazebo/components/Timestamp.h"
#include "scenario/gazebo/helpers.h"
#include "scenario/gazebo/signals.h"
#include "scenario/gazebo/utils.h"
#include "scenario/plugins/gazebo/ECMSingleton.h"

#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/ServerConfig.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/Publisher.hh>
#include <sdf/Element.hh>
#include <sdf/Param.hh>
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

namespace scenario {
    namespace gazebo {
        namespace detail {
            struct PhysicsData;
        } //  namespace detail
    } // namespace gazebo
} // namespace scenario

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
    std::shared_ptr<sdf::Root> sdf;

    struct
    {
        detail::PhysicsData physics;
        uint64_t numOfIterations = 0;
        std::unique_ptr<TinyProcessLib::Process> gui;
        std::shared_ptr<ignition::gazebo::Server> server;
    } gazebo;

    std::shared_ptr<ignition::gazebo::Server> getServer();

    using WorldName = std::string;
    std::unordered_map<WorldName, scenario::gazebo::WorldPtr> worlds;

    bool setPhysics(const detail::PhysicsData& physicsData);
    detail::PhysicsData getPhysicsData(const size_t worldIndex) const;

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
        gymppMessage << "The simulator is already initialized" << std::endl;
        return true;
    }

    // Initialize the server
    if (!pImpl->getServer()) {
        gymppError << "Failed to get the Gazebo server" << std::endl;
        return false;
    }

    auto cb = [&](int sig) {
        this->close();
        exit(sig);
    };

    // Setup signals callbacks.
    // It must be done after the creation of the simulator since
    // we override their callbacks.
    base::SignalManager::Instance().setCallback(SIGINT, cb);
    base::SignalManager::Instance().setCallback(SIGTERM, cb);
    base::SignalManager::Instance().setCallback(SIGABRT, cb);

    return true;
}

bool GazeboSimulator::initialized() const
{
    return bool(pImpl->gazebo.server);
}

bool GazeboSimulator::run(const bool paused)
{
    if (!this->initialized()) {
        gymppError << "The simulator was not initialized" << std::endl;
        return false;
    }

    // Get the gazebo server
    auto server = pImpl->getServer();
    if (!server) {
        gymppError << "Failed to get the ignition server" << std::endl;
        return false;
    }

    // If the server was configured to run in background (iterations = 0)
    // only the first call to this run method should trigger the start of
    // the simulation in non-blocking mode.
    // NOTE: non-blocking implementation is partial and not supported
    bool deterministic = pImpl->gazebo.numOfIterations != 0 ? true : false;

    if (!deterministic && server->Running()) {
        gymppWarning << "The server is already running in background"
                     << std::endl;
        return true;
    }

    size_t iterations = pImpl->gazebo.numOfIterations;

    // Allow executing a single paused step in non-blocking mode,
    // allowing to refresh the visualized world state
    if (!deterministic && !server->Running() && paused) {
        deterministic = true;
        iterations = 1;
    }

    // Run the simulation
    if (!server->Run(/*blocking=*/deterministic,
                     /*iterations=*/iterations,
                     /*paused=*/paused)) {
        gymppError << "The server couldn't execute the step" << std::endl;
        return false;
    }

    return true;
}

bool GazeboSimulator::gui(const int verbosity)
{
    if (!this->initialized()) {
        gymppError << "The simulator was not initialized" << std::endl;
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
        gymppError << "Failed to find any world in the simulator" << std::endl;
        return false;
    }

    // NOTE: we connect to the first world
    std::string worldName = worldNames[0];

    if (!pImpl->sceneBroadcasterActive(worldName)) {
        gymppDebug << "Starting the SceneBroadcaster plugin" << std::endl;
        auto world = this->getWorld();
        if (!world->insertWorldPlugin(
                "libignition-gazebo-scene-broadcaster-system.so",
                "ignition::gazebo::systems::SceneBroadcaster")) {
            gymppError << "Failed to load SceneBroadcaster plugin" << std::endl;
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
        gymppDebug << "Waiting GUI to show up... " << std::endl;
        node.ServiceList(serviceList);

        for (const auto& serviceName : serviceList) {
            if (serviceName.find("/gui/") == 0) {
                guiServiceExists = true;
                break;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } while (!guiServiceExists);

    gymppDebug << "GUI up and running" << std::endl;
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

    // Delete the simulator
    pImpl->gazebo.server.reset();

    return true;
}

bool GazeboSimulator::pause()
{
    if (!this->initialized()) {
        gymppMessage << "Couldn't pause the simulator, it was never initialized"
                     << std::endl;
        return true;
    }

    if (!this->running()) {
        gymppMessage << "The simulation is already paused" << std::endl;
        return true;
    }

    assert(this->worldNames().size() <= 1);
    pImpl->getServer()->SetPaused(true, 0);

    return !this->running();
}

bool GazeboSimulator::running() const
{
    if (!this->initialized()) {
        gymppMessage << "The simulator was not initialized" << std::endl;
        return false;
    }

    if (!pImpl->gazebo.server->Running()) {
        return false;
    }
    else {
        assert(this->worldNames().size() <= 1);
        return !pImpl->gazebo.server->Paused(0).value();
    }
}

bool GazeboSimulator::loadSdfWorld(const std::string& worldFile,
                                   const std::string& worldName)
{
    if (pImpl->sdf) {
        gymppError << "A world file has been already loaded" << std::endl;
        return false;
    }

    pImpl->sdf = utils::getSdfRootFromFile(worldFile);

    if (!pImpl->sdf) {
        // Error printed in the function call
        return false;
    }

    if (pImpl->sdf->WorldCount() != 1) {
        gymppError << "Only world files with one world are currently supported"
                   << std::endl;
        return false;
    }

    if (!worldName.empty()) {
        if (!utils::renameSDFWorld(*pImpl->sdf, worldName, /*worldIndex=*/0)) {
            return false;
        }
    }

    return true;
}

std::vector<std::string> GazeboSimulator::worldNames() const
{
    if (!this->initialized()) {
        gymppError << "The simulator was not initialized" << std::endl;
        return {};
    }

    if (!scenario::plugins::gazebo::ECMSingleton::Instance().valid()) {
        throw std::runtime_error("The ECM singleton is not valid");
    }

    auto* ecm = scenario::plugins::gazebo::ECMSingleton::Instance().getECM();

    std::vector<std::string> worldNames;

    ecm->Each<ignition::gazebo::components::World,
              ignition::gazebo::components::Name>(
        [&](const ignition::gazebo::Entity& worldEntity,
            ignition::gazebo::components::World* /*worldComponent*/,
            ignition::gazebo::components::Name* nameComponent) {
            if (!nameComponent) {
                gymppError
                    << "Failed to get the name of the world with entity ["
                    << worldEntity << "]. Ignoring the world." << std::endl;
                return true;
            }

            worldNames.push_back(nameComponent->Data());
            return true;
        });

    return worldNames;
}

std::shared_ptr<scenario::gazebo::World>
GazeboSimulator::getWorld(const std::string& worldName) const
{
    if (!this->initialized()) {
        gymppError << "The simulator was not initialized" << std::endl;
        return {};
    }

    // Get the existing world names
    const std::vector<std::string> worldNames = this->worldNames();

    // Copy the world name since we'll try to automatically detect it
    std::string returnedWorldName = worldName;

    // In most cases there will be only one world. In this case we allow
    // omitting to specify the world name and automatically get it.
    if (worldName.empty() && worldNames.size() == 1) {
        returnedWorldName = worldNames[0];
    }

    // Returne the cached world object if it exists
    if (pImpl->worlds.find(returnedWorldName) != pImpl->worlds.end()) {
        assert(pImpl->worlds.at(returnedWorldName));
        return pImpl->worlds.at(returnedWorldName);
    }

    if (std::find(worldNames.begin(), worldNames.end(), returnedWorldName)
        == worldNames.end()) {
        gymppError << "Failed to find world '" << returnedWorldName << "'"
                   << std::endl;
        return nullptr;
    }

    // Get the resources needed by the world
    auto ecm = scenario::plugins::gazebo::ECMSingleton::Instance().getECM();
    auto eventManager =
        scenario::plugins::gazebo::ECMSingleton::Instance().getEventManager();

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

    return pImpl->worlds[returnedWorldName];
}

// ==============
// Implementation
// ==============

std::shared_ptr<ignition::gazebo::Server> GazeboSimulator::Impl::getServer()
{
    // Lazy initialization of the server
    if (!gazebo.server) {

        if (gazebo.numOfIterations == 0) {
            gymppError << "Non-deterministic mode (iterations=0) is not "
                       << "currently supported" << std::endl;
            return nullptr;
        }

        if (!sdf) {
            gymppMessage << "Using default empty world" << std::endl;
            sdf = std::make_shared<sdf::Root>();
            sdf->LoadSdfString(scenario::gazebo::utils::getEmptyWorld());
        }

        if (!sdf->Element() || sdf->Element()->ToString("").empty()) {
            gymppError << "Failed to get SDF world from the configuration"
                       << std::endl;
            return nullptr;
        }

        if (sdf->WorldCount() == 0) {
            gymppError << "Failed to find a world in the SDF" << std::endl;
            return nullptr;
        }

        // Get the first world name. This is used to extract the ECM pointer.
        // Since all worlds share the same ECM, if there are multiple worlds
        // we just get the first one.
        // NOTE: check in the future if the server provides a new method
        //       to get the ECM pointer.
        const std::string world0Name = sdf->WorldByIndex(0)->Name();

        // The following could be replaced if the Server would return the
        // pointers to the ECM and EventManager
        ignition::gazebo::ServerConfig::PluginInfo ecmProvider;
        ecmProvider.SetFilename("libECMProvider.so");
        ecmProvider.SetName("scenario::plugins::gazebo::ECMProvider");
        ecmProvider.SetEntityType("world");
        ecmProvider.SetEntityName(world0Name);

        // Check if there are sdf parsing errors
        assert(utils::sdfStringValid(sdf->Element()->Clone()->ToString("")));

        // There is no way yet to set the physics step size if not passing
        // through the physics element of the SDF. We update here the SDF
        // overriding the default profile.
        // NOTE: this could be avoided if gazebo::Server would expose the
        //       SimulationRunner::SetStepSize method.
        if (!this->setPhysics(gazebo.physics)) {
            gymppError << "Failed to set physics profile" << std::endl;
            return nullptr;
        }

        gymppDebug << "Physics profile:" << std::endl
                   << this->gazebo.physics << std::endl;

        if (utils::verboseFromEnvironment()) {
            gymppDebug << "Loading the following SDF file in the gazebo server:"
                       << std::endl;
            std::cout << sdf->Element()->ToString("") << std::endl;
        }

        ignition::gazebo::ServerConfig config;

        config.SetSeed(0);
        config.SetUseLevels(false);
        config.AddPlugin(ecmProvider);
        config.SetSdfString(sdf->Element()->ToString(""));

        // Create the server
        gazebo.server = std::make_shared<ignition::gazebo::Server>(config);

        gymppDebug << "Starting the gazebo server" << std::endl;
        if (!gazebo.server->Run(
                /*blocking=*/true, /*iterations=*/1, /*paused=*/true)) {
            gymppError << "Failed to initialize the first gazebo server run"
                       << std::endl;
            return nullptr;
        }

        if (!scenario::plugins::gazebo::ECMSingleton::Instance().valid()) {
            gymppError << "The ECM singleton is not valid" << std::endl;
            return nullptr;
        }

        // Get the ECM
        auto* ecm =
            scenario::plugins::gazebo::ECMSingleton::Instance().getECM();

        // Insert custom world components
        for (size_t idx = 0; idx < sdf->WorldCount(); ++idx) {
            // Get the world name from the SDF
            const sdf::World* worldSdf = sdf->WorldByIndex(idx);
            const std::string worldName = worldSdf->Name();

            // Get the entity of the world
            auto worldEntity = ecm->EntityByComponents(
                ignition::gazebo::components::World(),
                ignition::gazebo::components::Name(worldName));

            if (worldEntity == ignition::gazebo::kNullEntity) {
                gymppError << "Couldn't find world " << worldName << std::endl;
                return nullptr;
            }

            // Insert a new component with the simulated time
            // (Physics will update it)
            ecm->CreateComponent(
                worldEntity,
                ignition::gazebo::components::SimulatedTime(
                    std::chrono::steady_clock::duration::zero()));

            // Insert the time of creation of the world
            ecm->CreateComponent(
                worldEntity,
                ignition::gazebo::components::Timestamp(
                    std::chrono::steady_clock::duration::zero()));
        }
    }

    return gazebo.server;
}

bool GazeboSimulator::Impl::setPhysics(const detail::PhysicsData& physicsData)
{
    if (physicsData.rtf <= 0) {
        gymppError << "Invalid RTF value (" << physicsData.rtf << ")"
                   << std::endl;
        return false;
    }

    if (physicsData.maxStepSize <= 0) {
        gymppError << "Invalid physics step size (" << physicsData.maxStepSize
                   << ")" << std::endl;
        return false;
    }

    if (std::isinf(physicsData.maxStepSize)) {
        gymppError << "The physics step size cannot be infinite" << std::endl;
        return false;
    }

    assert(sdf);
    for (size_t idx = 0; idx < sdf->WorldCount(); ++idx) {

        const sdf::World* world = sdf->WorldByIndex(idx);

        if (world->PhysicsCount() != 1) {
            gymppError << "Found more than one physics profile." << std::endl;
            return false;
        }

        // Set the physics properties using the helper.
        // This sets the internal value but it does not update the DOM.
        auto* physics = const_cast<sdf::Physics*>(world->PhysicsByIndex(0));
        physics->SetMaxStepSize(physicsData.maxStepSize);
        physics->SetRealTimeFactor(physicsData.rtf);
    }

    // Update the DOM operating directly on the raw elements
    sdf::ElementPtr world = sdf->Element()->GetElement("world");

    if (!world) {
        gymppError << "Failed to find any world" << std::endl;
        return false;
    }

    while (world) {
        sdf::ElementPtr physics = world->GetElement("physics");
        assert(physics);

        sdf::ElementPtr max_step_size = physics->GetElement("max_step_size");
        max_step_size->AddValue(
            "double", std::to_string(physicsData.maxStepSize), true);

        sdf::ElementPtr real_time_update_rate =
            physics->GetElement("real_time_update_rate");
        real_time_update_rate->AddValue(
            "double", std::to_string(physicsData.realTimeUpdateRate), true);

        sdf::ElementPtr real_time_factor =
            physics->GetElement("real_time_factor");
        real_time_factor->AddValue(
            "double", std::to_string(physicsData.rtf), true);

        world = world->GetNextElement("world");
    }

    assert(this->getPhysicsData(0) == gazebo.physics);

    return true;
}

detail::PhysicsData
GazeboSimulator::Impl::getPhysicsData(const size_t worldIndex) const
{
    assert(sdf);
    const sdf::World* world = sdf->WorldByIndex(worldIndex);
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
