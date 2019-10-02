/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "gympp/gazebo/GazeboWrapper.h"
#include "gympp/Log.h"
#include "gympp/gazebo/ECMSingleton.h"
#include "gympp/gazebo/IgnitionRobot.h"
#include "gympp/gazebo/RobotSingleton.h"
#include "process.hpp"

#include <ignition/common/Console.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/common/Util.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/SdfEntityCreator.hh>
#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/ServerConfig.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/config.hh>
#include <sdf/Element.hh>
#include <sdf/Error.hh>
#include <sdf/Model.hh>
#include <sdf/Physics.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <cassert>
#include <chrono>

using namespace gympp::gazebo;
const std::string GymppVerboseEnvVar = "GYMPP_VERBOSE";

// =====
// PIMPL
// =====

struct GazeboData
{
    PhysicsData physics;
    uint64_t numOfIterations = 0;
    ignition::gazebo::ServerConfig config;
    std::unique_ptr<TinyProcessLib::Process> gui;
    std::shared_ptr<ignition::gazebo::Server> server;
};

class GazeboWrapper::Impl
{
public:
    size_t id;
    sdf::Root sdf;

    GazeboData gazebo;
    std::shared_ptr<ignition::gazebo::Server> getServer();

    std::shared_ptr<ignition::gazebo::SdfEntityCreator> sdfEntityCreator;
    std::shared_ptr<ignition::gazebo::SdfEntityCreator> getSdfEntityCreator();

    PhysicsData getPhysicsData() const;
    bool setPhysics(const PhysicsData& physicsData);

    ignition::common::SystemPaths systemPaths;
};

std::shared_ptr<ignition::gazebo::Server> GazeboWrapper::Impl::getServer()
{
    // Lazy initialization of the server
    if (!gazebo.server) {

        assert(sdf.Element());
        assert(!sdf.Element()->ToString("").empty());

        // Check if there are sdf parsing errors
        sdf::Root root;
        auto errors = root.LoadSdfString(sdf.Element()->ToString(""));
        assert(errors.empty()); // This should be already ok

        // To simplify debugging, use an environment variable to enable / disable
        // printing the SDF string
        std::string envVarContent{};
        if (ignition::common::env(GymppVerboseEnvVar, envVarContent) && envVarContent == "1") {
            gymppDebug << "Loading the following SDF file in the gazebo server:" << std::endl;
            std::cout << sdf.Element()->ToString("") << std::endl;
        }

        // Store the sdf string in the gazebo configuration
        gazebo.config.SetSdfString(sdf.Element()->ToString(""));

        // Create the server
        gazebo.config.SetUseLevels(false);
        gazebo.server = std::make_shared<ignition::gazebo::Server>(gazebo.config);
        assert(gazebo.server);

        // The GUI needs the server already up. Warming up the first iteration and pausing the
        // server. It will be unpaused at the step() call.
        gymppDebug << "Starting the server as paused" << std::endl;
        if (!gazebo.server->Run(/*blocking=*/false, gazebo.numOfIterations, /*paused=*/true)) {
            gymppError << "Failed to warm up the gazebo server in paused state" << std::endl;
            return nullptr;
        }
    }

    return gazebo.server;
}

std::shared_ptr<ignition::gazebo::SdfEntityCreator> GazeboWrapper::Impl::getSdfEntityCreator()
{
    if (sdfEntityCreator) {
        return sdfEntityCreator;
    }

    if (!ECMSingleton::get().valid()) {
        gymppError << "ECMSingleton not yet initialized. Failed to get the SdfEntityCreator"
                   << std::endl;
        return {};
    }

    // Create the SdfEntityCreator
    sdfEntityCreator = std::make_unique<ignition::gazebo::SdfEntityCreator>(
        *ECMSingleton::get().getECM(), *ECMSingleton::get().getEventManager());

    return sdfEntityCreator;
}

// TODO: there's a bug in the destructor of sdf::Physics that prevents returning std::optional
bool GazeboWrapper::findAndLoadSdf(const std::string& sdfFileName, sdf::Root& root)
{
    if (sdfFileName.empty()) {
        gymppError << "The SDF file name of the gazebo model is empty" << std::endl;
        return false;
    }

    // Find the file
    // TODO: add install directory of our world and model files
    std::string sdfFilePath = pImpl->systemPaths.FindFile(sdfFileName);

    if (sdfFilePath.empty()) {
        gymppError << "Failed to find '" << sdfFileName << "'. "
                   << "Check that it's contained in the paths defined in IGN_GAZEBO_RESOURCE_PATH."
                   << std::endl;
        return false;
    }

    // Load the sdf
    auto errors = root.Load(sdfFilePath);

    if (!errors.empty()) {
        gymppError << "Failed to load sdf file '" << sdfFilePath << "." << std::endl;
        for (const auto& error : errors) {
            gymppError << error << std::endl;
        }
        return false;
    }
    return true;
}

PhysicsData GazeboWrapper::Impl::getPhysicsData() const
{
    assert(sdf.WorldCount() == 1);
    assert(sdf.WorldByIndex(0)->PhysicsCount() == 1);

    PhysicsData physics;

    physics.rtf = sdf.WorldByIndex(0)->PhysicsByIndex(0)->RealTimeFactor();
    physics.maxStepSize = sdf.WorldByIndex(0)->PhysicsByIndex(0)->MaxStepSize();

    return physics;
}

bool GazeboWrapper::Impl::setPhysics(const PhysicsData& physicsData)
{
    assert(sdf.WorldCount() == 1);
    assert(sdf.WorldByIndex(0)->PhysicsCount() == 1);

    gymppDebug << "Setting physics profile" << std::endl;
    gymppDebug << "Desired RTF: " << physicsData.rtf << std::endl;
    gymppDebug << "Max physics step size:" << physicsData.maxStepSize << std::endl;
    gymppDebug << "Real time update rate: " << physicsData.realTimeUpdateRate << std::endl;

    if (physicsData.rtf <= 0) {
        gymppError << "The real time factor cannot be less than zero" << std::endl;
        return false;
    }

    if (physicsData.maxStepSize <= 0) {
        gymppError << "The maximum step size of the physics cannot be less than zero" << std::endl;
        return false;
    }

    // Set the physics properties using the helper.
    // This sets the internal value but it does not update the DOM.
    {
        auto* physics = const_cast<sdf::Physics*>(sdf.WorldByIndex(0)->PhysicsByIndex(0));
        physics->SetMaxStepSize(physicsData.maxStepSize);
        physics->SetRealTimeFactor(physicsData.rtf);
    }

    // Update the DOM operating directly on the raw elements
    {
        sdf::ElementPtr world = sdf.Element()->GetElement("world");
        assert(world);

        sdf::ElementPtr physics = world->GetElement("physics");
        assert(physics);

        sdf::ElementPtr max_step_size = physics->GetElement("max_step_size");
        max_step_size->AddValue("double", std::to_string(physicsData.maxStepSize), true);

        sdf::ElementPtr real_time_update_rate = physics->GetElement("real_time_update_rate");
        real_time_update_rate->AddValue(
            "double", std::to_string(physicsData.realTimeUpdateRate), true);

        sdf::ElementPtr real_time_factor = physics->GetElement("real_time_factor");
        real_time_factor->AddValue("double", std::to_string(physicsData.rtf), true);
    }

    assert(getPhysicsData() == gazebo.physics);
    return true;
}

// =============
// GAZEBOWRAPPER
// =============

GazeboWrapper::GazeboWrapper(const size_t numOfIterations,
                             const double desiredRTF,
                             const double physicsUpdateRate)
    : pImpl{new GazeboWrapper::Impl, [](Impl* impl) { delete impl; }}
{
    // Configure gazebo
    pImpl->gazebo.numOfIterations = numOfIterations;

    // Configure the physics profile
    pImpl->gazebo.physics.rtf = desiredRTF;
    pImpl->gazebo.physics.maxStepSize = 1 / physicsUpdateRate;

    if (numOfIterations == 0) {
        gymppError << "Failed to set the number of gazebo iterations" << std::endl;
        gymppError << "Check that the agent rate is not higher than the physics rate" << std::endl;
        assert(numOfIterations != 0);
    }

    // Configure search path of resources
    pImpl->systemPaths.SetFilePathEnv("IGN_GAZEBO_RESOURCE_PATH");
    pImpl->systemPaths.AddFilePaths(IGN_GAZEBO_WORLD_INSTALL_DIR);
    // TODO: add install world / models path

    // Set default verbosity which is dependent on the compilation flags
    setVerbosity();
}

GazeboWrapper::~GazeboWrapper()
{
    // Close the gui if it was opened
    close();
}

bool GazeboWrapper::initialize()
{
    // Initialize the server
    // TODO: check that all the other configures have been called
    if (!pImpl->getServer()) {
        gymppError << "Failed to get the ignition server" << std::endl;
        return false;
    }

    return true;
}

bool GazeboWrapper::run()
{
    // Get the gazebo server
    auto server = pImpl->getServer();
    if (!server) {
        gymppError << "Failed to get the ignition server" << std::endl;
        return false;
    }

    assert(server);
    assert(server->Paused());
    assert((server->Running() && server->Paused().value())
           || (!server->Running() && !server->Paused().value()));

    // Handle first iteration
    if (server->Running() && server->Paused().value()) {
        gymppDebug << "Unpausing the server. Running the first simulation run." << std::endl;
        bool ok = server->SetPaused(false);
        assert(ok);

        // Since the server was started in non-blocking mode, we have to wait that this first
        // iteration finishes
        while (server->Running()) {
            gymppDebug << "Waiting the first simulation run to finish..." << std::endl;
            assert(!server->Paused().value());
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        gymppDebug << "First run ok" << std::endl;
    }
    // Run regular iteration
    else {
        if (!server->Run(/*blocking=*/true, pImpl->gazebo.numOfIterations, /*paused=*/false)) {
            gymppError << "The server couldn't execute the step" << std::endl;
            return false;
        }
    }

    return true;
}

bool GazeboWrapper::gui()
{
    // If ign-gazebo-gui is already running, return without doing anything.
    int exit_status;
    if (pImpl->gazebo.gui && !pImpl->gazebo.gui->try_get_exit_status(exit_status)) {
        return true;
    }

    // The GUI needs the ignition server running. Initialize it.
    if (!pImpl->getServer()) {
        gymppError << "Failed to get the ignition server" << std::endl;
        return false;
    }

    // Spawn a new process with the GUI
    pImpl->gazebo.gui = std::make_unique<TinyProcessLib::Process>("ign gazebo -g");

    return true;
}

bool GazeboWrapper::close()
{
    if (pImpl->gazebo.gui) {
#if defined(WIN32) || defined(_WIN32)
        bool force = false;
#else
        bool force = true;
#endif
        pImpl->gazebo.gui->kill(force);
    }

    return true;
}

bool GazeboWrapper::initialized()
{
    if (pImpl->gazebo.server) {
        return true;
    }
    else {
        return false;
    }
}

PhysicsData GazeboWrapper::getPhysicsData() const
{
    return pImpl->gazebo.physics;
}

void GazeboWrapper::setVerbosity(int level)
{
    ignition::common::Console::SetVerbosity(level);
}

std::string GazeboWrapper::getModelNameFromSDF(const std::string& sdfString)
{
    sdf::Root root;

    // Load the sdf
    if (auto errors = root.LoadSdfString(sdfString); !errors.empty()) {
        gymppError << "Failed to load SDF string" << std::endl;
        for (const auto& error : errors) {
            gymppError << error << std::endl;
        }
        return {};
    }

    assert(root.ModelCount() == 1);
    return root.ModelByIndex(0)->Name();
}

std::string GazeboWrapper::getWorldName() const
{
    if (pImpl->sdf.WorldCount() == 0) {
        gymppError << "Failed to find any world. Has the world been configured?" << std::endl;
        return {};
    }

    assert(pImpl->sdf.WorldCount() == 1);

    // We get the world from the SDF file.
    return pImpl->sdf.WorldByIndex(0)->Name();
}

bool GazeboWrapper::insertModel(const gympp::gazebo::ModelInitData& modelData) const
{
    gymppDebug << "Inserting new model " << modelData.modelName << std::endl;

    auto sdfEntityCreator = pImpl->getSdfEntityCreator();
    assert(sdfEntityCreator);

    // Parse the SDF string
    sdf::Root root;
    auto errors = root.LoadSdfString(modelData.sdfString);

    if (!errors.empty()) {
        for (const auto& err : errors) {
            gymppError << err << std::endl;
        }
        return false;
    }

    assert(root.ModelCount() == 1);

    // Get the world entity
    auto worldEntities =
        ECMSingleton::get().getECM()->EntitiesByComponents(ignition::gazebo::components::World());
    assert(worldEntities.size() == 1);
    auto worldEntity = worldEntities[0];

    // Handle entity name
    std::string finalEntityName;

    if (modelData.modelName.empty()) {
        finalEntityName = root.ModelByIndex(0)->Name();
    }
    else {
        finalEntityName = modelData.modelName;
    }

    // Check that there is no entity name clashing
    if (ECMSingleton::get().getECM()->EntityByComponents(
            ignition::gazebo::components::Name(finalEntityName),
            ignition::gazebo::components::ParentEntity(worldEntity))
        != ignition::gazebo::kNullEntity) {
        gymppError << "Failed to insert entity for model '" << modelData.modelName
                   << "'. Another entity with the same name already exists." << std::endl;
        return false;
    }

    // Update the name in the sdf model
    const_cast<sdf::Model*>(root.ModelByIndex(0))->SetName(finalEntityName);

    // To simplify debugging, use an environment variable to enable / disable
    // printing the SDF string
    std::string envVarContent{};
    if (ignition::common::env(GymppVerboseEnvVar, envVarContent) && envVarContent == "1") {
        gymppDebug << "Inserting a model from the following SDF:" << std::endl;
        std::cout << root.Element()->ToString("") << std::endl;
    }

    // Create the model entity
    gymppDebug << "Creating new entity for the model" << std::endl;
    auto modelEntity = sdfEntityCreator->CreateEntities(root.ModelByIndex(0));
    gymppDebug << "Created entity [" << modelEntity << "] named [" << modelData.modelName << "]"
               << std::endl;

    // Attach the model to the world
    sdfEntityCreator->SetParent(modelEntity, worldEntity);

    // Create pose data
    ignition::math::Pose3d pose;
    pose.Pos() = ignition::math::Vector3<double>(
        modelData.position[0], modelData.position[1], modelData.position[2]);
    pose.Rot() = ignition::math::Quaternion(modelData.orientation[0],
                                            modelData.orientation[1],
                                            modelData.orientation[2],
                                            modelData.orientation[3]);

    // Create a pose component
    auto poseComp =
        ECMSingleton::get().getECM()->Component<ignition::gazebo::components::Pose>(modelEntity);
    *poseComp = ignition::gazebo::components::Pose(pose);

    // Update entity name
    auto nameComp =
        ECMSingleton::get().getECM()->Component<ignition::gazebo::components::Name>(modelEntity);
    *nameComp = ignition::gazebo::components::Name(finalEntityName);

    // Create an IgnitionRobot object from the ecm
    auto ignRobot = std::make_shared<gympp::gazebo::IgnitionRobot>();
    if (!ignRobot->configureECM(modelEntity, root.Element(), *ECMSingleton::get().getECM())) {
        gymppError << "Failed to configure the Robot interface" << std::endl;
        return false;
    }

    if (!ignRobot->valid()) {
        gymppError << "The Robot interface is not valid" << std::endl;
        return false;
    }

    // Store the robot in the singleton
    if (!RobotSingleton::get().storeRobot(ignRobot)) {
        gymppError << "Failed to store the robot in the singleton" << std::endl;
        return false;
    }

    return true;
}

bool GazeboWrapper::removeModel(const std::string& modelName) const
{
    gymppDebug << "Removing existing model '" << modelName << "'" << std::endl;

    auto sdfEntityCreator = pImpl->getSdfEntityCreator();
    assert(sdfEntityCreator);

    // Try to find a matching entity
    auto modelEntity = ECMSingleton::get().getECM()->EntityByComponents(
        ignition::gazebo::components::Model(), ignition::gazebo::components::Name(modelName));

    if (modelEntity == ignition::gazebo::kNullEntity) {
        gymppError << "Failed to find model '" << modelName << "' in the ECM. Model not removed."
                   << std::endl;
        return false;
    }

    // Request the removal of the model
    gymppDebug << "Requesting removal of entity [" << modelEntity << "]" << std::endl;
    sdfEntityCreator->RequestRemoveEntity(modelEntity);

    // Remove the robot from the singleton
    if (!RobotSingleton::get().deleteRobot(modelName)) {
        gymppWarning << "The Robot object associated to the model was not in the singleton"
                     << std::endl;
    }

    assert(!RobotSingleton::get().exists(modelName));

    return true;
}

bool GazeboWrapper::setupGazeboWorld(const std::string& worldFile)
{
    // Find and load the sdf file that contains the world
    if (!findAndLoadSdf(worldFile, pImpl->sdf)) {
        gymppError << "Failed to find and load sdf file '" << worldFile << "'" << std::endl;
        return false;
    }

    if (!pImpl->setPhysics(pImpl->gazebo.physics)) {
        gymppError << "Failed to set physics profile" << std::endl;
        return false;
    }

    return true;
}
