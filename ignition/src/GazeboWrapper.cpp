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
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/ContactSensor.hh>
#include <ignition/gazebo/components/ContactSensorData.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/Link.hh>
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
    std::string worldName;

    GazeboData gazebo;
    std::shared_ptr<ignition::gazebo::Server> getServer();

    std::shared_ptr<ignition::gazebo::SdfEntityCreator> sdfEntityCreator;
    std::shared_ptr<ignition::gazebo::SdfEntityCreator>
    getSdfEntityCreator(const std::string& worldName);

    PhysicsData getPhysicsData() const;
    bool setPhysics(const PhysicsData& physicsData);

    std::vector<std::string> modelPendingToRemove;

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

        // Running a dummy initial step to start the server. This is necessary because the GUI needs
        // the server already running.
        gymppDebug << "Starting the gazebo server" << std::endl;
        if (!gazebo.server->Run(/*blocking=*/false, /*iterations=*/1, /*paused=*/true)) {
            gymppError << "Failed to run the first gazebo server step" << std::endl;
            return {};
        }
    }

    return gazebo.server;
}

std::shared_ptr<ignition::gazebo::SdfEntityCreator>
GazeboWrapper::Impl::getSdfEntityCreator(const std::string& worldName)
{
    if (sdfEntityCreator) {
        return sdfEntityCreator;
    }

    if (!ECMSingleton::get().valid(worldName)) {
        gymppError << "ECMSingleton not yet initialized. Failed to get the SdfEntityCreator"
                   << std::endl;
        return {};
    }

    // Create the SdfEntityCreator
    sdfEntityCreator = std::make_unique<ignition::gazebo::SdfEntityCreator>(
        *ECMSingleton::get().getECM(worldName), *ECMSingleton::get().getEventManager(worldName));

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
    gymppDebug << "Max physics step size: " << physicsData.maxStepSize << std::endl;
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

    // Handle first dummy iteration
    if (server->Running() && server->Paused().value()) {
        gymppDebug << "Unpausing the server. Running the first dummy iteration." << std::endl;
        bool ok = server->SetPaused(false);
        assert(ok);

        while (server->Running()) {
            gymppDebug << "Waiting the first iteration run to finish..." << std::endl;
            assert(!server->Paused().value());
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        gymppDebug << "First dummy iteration executed" << std::endl;
    }

    // Run the simulation
    if (!server->Run(/*blocking=*/true, pImpl->gazebo.numOfIterations, /*paused=*/false)) {
        gymppError << "The server couldn't execute the step" << std::endl;
        return false;
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

    // Get the names of all the pending models.
    // We need to make a copy because in the following for loop the elements of the vector are
    // iterated and, in the removeModel, erased. This messes up iterators.
    std::vector<std::string> pendingModels = pImpl->modelPendingToRemove;

    // Remove all models
    for (const auto& modelName : pendingModels) {
        gymppDebug << "Removing model '" << modelName << "' added through the wrapper" << std::endl;
        if (!removeModel(modelName)) {
            gymppWarning << "Failed to remove model '" << modelName
                         << "' while closing the gazebo wrapper" << std::endl;
        }
    }

    // Remove the ECM from the singleton
    if (ECMSingleton::get().valid(getWorldName())) {
        gymppDebug << "Cleaning the ECM singleton from world '" << getWorldName() << "'"
                   << std::endl;
        ECMSingleton::get().clean(getWorldName());
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

double GazeboWrapper::getSimulatedTime() const
{
    auto dt = pImpl->gazebo.physics.maxStepSize;
    auto iterationCount = pImpl->getServer()->IterationCount();

    if (!iterationCount) {
        gymppError << "Failed to get the simulated time" << std::endl;
        return 0.0;
    }

    // TODO: this will no longer work if Gazebo changes the step dynamically
    return dt * iterationCount.value();
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
    // TODO: this does not read the updated name
    //    gymppWarning << pImpl->sdf.WorldByIndex(0)->Name() << std::endl;
    //    return pImpl->sdf.WorldByIndex(0)->Name();

    return pImpl->worldName;
}

bool GazeboWrapper::insertModel(const gympp::gazebo::ModelInitData& modelData,
                                const gympp::gazebo::PluginData& pluginData)
{
    if (!initialized()) {
        gymppError << "The simulator was not initialized. Call initialize() first." << std::endl;
        return false;
    }

    gymppDebug << "Inserting new model " << modelData.modelName << std::endl;

    // ====================
    // INITIALIZE RESOURCES
    // ====================

    // Check that the ECM has been stored by the plugin
    if (!ECMSingleton::get().valid(getWorldName())) {
        gymppError << "No ECM found for world '" << getWorldName()
                   << "'. Does your world file have the ECMProvider plugin?" << std::endl;
        return false;
    }

    // Get the ECM and the EventManager
    auto* ecm = ECMSingleton::get().getECM(getWorldName());
    auto* eventManager = ECMSingleton::get().getEventManager(getWorldName());

    // Get the SdfEntityCreator that abstracts the ECM to easily create entities
    auto sdfEntityCreator = pImpl->getSdfEntityCreator(getWorldName());
    assert(sdfEntityCreator);

    // Parse the SDF string
    sdf::Root modelSdfRoot;
    auto errors = modelSdfRoot.LoadSdfString(modelData.sdfString);

    if (!errors.empty()) {
        for (const auto& err : errors) {
            gymppError << err << std::endl;
        }
        return false;
    }

    // Get the world entity
    auto worldEntities = ecm->EntitiesByComponents(
        ignition::gazebo::components::World(), ignition::gazebo::components::Name(getWorldName()));
    assert(worldEntities.size() == 1);
    auto worldEntity = worldEntities[0];
    assert(worldEntity != ignition::gazebo::kNullEntity);
    gymppDebug << "Inserting the model in the world '" << getWorldName() << "' [" << worldEntity
               << "]" << std::endl;

    // ============================
    // RENAME THE MODEL NAME IN SDF
    // ============================

    std::string finalModelEntityName;

    // Handle model entity name
    if (modelData.modelName.empty()) {
        assert(modelSdfRoot.ModelCount() == 1);
        finalModelEntityName = modelSdfRoot.ModelByIndex(0)->Name();
    }
    else {
        finalModelEntityName = modelData.modelName;
    }

    // Check that there is no entity name clashing
    if (ecm->EntityByComponents(ignition::gazebo::components::Name(finalModelEntityName),
                                ignition::gazebo::components::ParentEntity(worldEntity))
        != ignition::gazebo::kNullEntity) {
        gymppError << "Failed to insert entity for model '" << finalModelEntityName
                   << "'. Another entity with the same name already exists." << std::endl;
        return false;
    }

    // Update the name in the sdf model. This is necessary because model plugins are loaded right
    // before the creation of the model entity and, instead of receiving the model entity name, the
    // receive the model sdf name.
    // NOTE: The following is not enough because the name is not serialized to string. We need also
    //       to operate directly on the raw element.
    const_cast<sdf::Model*>(modelSdfRoot.ModelByIndex(0))->SetName(finalModelEntityName);

    // Create a new model with the scoped name
    sdf::ElementPtr renamedModel(new sdf::Element);
    renamedModel->SetName("model");
    renamedModel->AddAttribute("name", "string", finalModelEntityName, true);

    sdf::ElementPtr child = modelSdfRoot.ModelByIndex(0)->Element()->GetFirstElement();

    // Add all the children
    while (child) {
        renamedModel->InsertElement(child);
        child = child->GetNextElement();
    }

    // Remove the old model
    auto originalModelElement = modelSdfRoot.ModelByIndex(0)->Element();
    originalModelElement->RemoveFromParent();

    // Insert the renamed model
    renamedModel->SetParent(modelSdfRoot.Element());
    modelSdfRoot.Element()->InsertElement(renamedModel);
    assert(modelSdfRoot.ModelCount() == 1);
    assert(modelSdfRoot.ModelNameExists(finalModelEntityName));

    // ==============
    // ADD THE PLUGIN
    // ==============

    // Add the model plugin, if defined. This is used to insert either the RobotController plugin
    // for python environments, or the gympp plugin for C++ environments.
    if (!pluginData.libName.empty() && !pluginData.className.empty()) {
        gymppDebug << "Inserting SDF plugin '" << pluginData.libName << "@" << pluginData.className
                   << "'" << std::endl;

        // Create the plugin SDF element
        auto pluginElement = std::make_shared<sdf::Element>();
        pluginElement->SetName("plugin");
        pluginElement->AddAttribute("name", "string", "pluginname", true, "plugin name");
        pluginElement->AddAttribute(
            "filename", "string", "pluginfilename", true, "plugin filename");

        sdf::ParamPtr pluginNameParam = pluginElement->GetAttribute("name");
        pluginNameParam->SetFromString(pluginData.className);
        assert(pluginNameParam->GetSet());

        sdf::ParamPtr pluginFileNameParam = pluginElement->GetAttribute("filename");
        pluginFileNameParam->SetFromString(pluginData.libName);
        assert(pluginNameParam->GetSet());

        // Store the plugin SDF element in the model SDF
        assert(modelSdfRoot.ModelCount() == 1);
        pluginElement->SetParent(modelSdfRoot.ModelByIndex(0)->Element());
        modelSdfRoot.ModelByIndex(0)->Element()->InsertElement(pluginElement);

        assert(pluginElement->GetParent() == modelSdfRoot.ModelByIndex(0)->Element());
        assert(modelSdfRoot.ModelByIndex(0)->Element()->HasElement("plugin"));
    }

    // =======================
    // CREATE THE MODEL ENTITY
    // =======================

    // To simplify debugging, use an environment variable to enable / disable
    // printing the final SDF model string
    std::string envVarContent{};
    if (ignition::common::env(GymppVerboseEnvVar, envVarContent) && envVarContent == "1") {
        gymppDebug << "Inserting a model from the following SDF:" << std::endl;
        std::cout << modelSdfRoot.Element()->ToString("") << std::endl;
    }

    ignition::gazebo::Entity modelEntity;

    {
        auto& ecmSingleton = ECMSingleton::get();

        // The first iteration is quite delicate for two reasons:
        //
        // 1) The GUI needs the simulator running and it needs to receive the state to draw the
        //    world. For this reason, we need to start the simulator with a dummy iteration and
        //    pause its execution. Gazebo in paused state runs anyway all the systems, without
        //    the need to step it (the simulated time does not progress).
        //
        // 2) Typically models are inserted right after the server creation, and we want to be
        //    able to visualize them without advancing the physics. Using a first dummy iteration
        //    in paused state allow inserting the model entities in the physics. The problem is
        //    that we need to be sure that this happens in the PreUpdate step.
        //
        // When the first run() command is executed, the simulation switches to syncronous mode.
        // This means that all the PreUpdate and PostUpdate calls are executed only when run() is
        // called.
        //
        // In order to handle the particular state of the initial iteration, we need a rather
        // involved synchronization logic. This allows avoiding to use the UserCommands system
        // to create entities, giving us full power.
        assert(pImpl->getServer()->Paused().has_value());
        std::unique_lock<std::mutex> lock;
        if (pImpl->getServer()->Running() && pImpl->getServer()->Paused().value()) {
            lock = ecmSingleton.waitPreUpdate(getWorldName());
        }

        // Create the model entity. It will create a model with the name specified in the SDF.
        modelEntity = sdfEntityCreator->CreateEntities(modelSdfRoot.ModelByIndex(0));

        // Rename the model entity name. We want a scoped model name so that we can add multiple
        // models from the same model SDF file.
        ecm->RemoveComponent<ignition::gazebo::components::Name>(modelEntity);
        ecm->CreateComponent(modelEntity, ignition::gazebo::components::Name(finalModelEntityName));

        auto modelNameComponent = ecm->Component<ignition::gazebo::components::Name>(modelEntity);
        gymppDebug << "Created entity [" << modelEntity << "] named [" << modelNameComponent->Data()
                   << "]" << std::endl;
        assert(modelNameComponent->Data() == finalModelEntityName);

        // Attach the model entity to the world entity
        sdfEntityCreator->SetParent(modelEntity, worldEntity);

        // -------------------
        // HANDLE INITIAL POSE
        // -------------------

        // Create pose data
        ignition::math::Pose3d pose;
        pose.Pos() = ignition::math::Vector3<double>(
            modelData.position[0], modelData.position[1], modelData.position[2]);
        pose.Rot() = ignition::math::Quaternion<double>(modelData.orientation[0],
                                                        modelData.orientation[1],
                                                        modelData.orientation[2],
                                                        modelData.orientation[3]);

        // Create a pose component
        auto poseComp = ecm->Component<ignition::gazebo::components::Pose>(modelEntity);
        *poseComp = ignition::gazebo::components::Pose(pose);

        // ----------------------
        // HANDLE CONTACT SENSORS
        // ----------------------

        // Store collision entities that need to be modified.
        // We use an external data structure because editing the ECM within the Each() method
        // can cause undefined behavior.
        using CollisionName = std::string;
        using CollisionEntity = ignition::gazebo::Entity;
        std::unordered_map<CollisionName, CollisionEntity> collisions;

        // Get the collision names of all the links that belong to the new model
        ecm->Each<ignition::gazebo::components::Name,
                  ignition::gazebo::components::Collision,
                  ignition::gazebo::components::ParentEntity>(
            [&](const ignition::gazebo::Entity& collisionEntity,
                ignition::gazebo::components::Name* collisionName,
                ignition::gazebo::components::Collision*,
                ignition::gazebo::components::ParentEntity* parentEntity) -> bool {
                // The parent entity of the collision must be a link
                assert(parentEntity->Data());
                auto parentLinkEntity = parentEntity->Data();
                auto linkLinkComp =
                    ecm->Component<ignition::gazebo::components::Link>(parentLinkEntity);

                // And must have a name
                assert(linkLinkComp);
                auto linkNameComp =
                    ecm->Component<ignition::gazebo::components::Name>(parentLinkEntity);
                assert(linkNameComp);

                // Get the model entity (parent of the link)
                auto parentModelEntity = ecm->ParentEntity(parentLinkEntity);

                // Discard all the links that do not belong to this model
                if (parentModelEntity != modelEntity) {
                    return true;
                }

                // Insert the collision entity in the map
                collisions.insert({collisionName->Data(), collisionEntity});
                return true;
            });

        // Create the ContactSensor and ContactSensorData components.
        // Note that here we are bypassing the Contact system.
        for (const auto& [collisionName, collisionEntity] : collisions) {
            // Create the SDF element of the contact sensor
            auto sensorElement = std::make_shared<sdf::Element>();
            sensorElement->SetName("sensor");
            sensorElement->AddAttribute("name", "string", "contact_sensor", true, "sensor name");
            sensorElement->AddAttribute("type", "string", "contact", true, "sensor type");

            auto alwaysOnElement = std::make_shared<sdf::Element>();
            alwaysOnElement->SetName("always_on");
            alwaysOnElement->SetParent(sensorElement);
            sensorElement->InsertElement(alwaysOnElement);
            alwaysOnElement->AddValue("bool", "1", true, "initialize the sensor status");

            auto contactElement = std::make_shared<sdf::Element>();
            contactElement->SetName("contact");
            contactElement->SetParent(sensorElement);
            sensorElement->InsertElement(contactElement);

            auto collisionElement = std::make_shared<sdf::Element>();
            collisionElement->SetName("collision");
            collisionElement->SetParent(contactElement);
            contactElement->InsertElement(collisionElement);
            collisionElement->AddValue("string", collisionName, true, "collision element name");

            // Parse the SDF contact element
            sdf::Sensor contactSensor;
            auto errors = contactSensor.Load(sensorElement);

            if (!errors.empty()) {
                gymppError << "Failed to load contact sensor sdf" << std::endl;
                for (const auto& error : errors) {
                    gymppError << error << std::endl;
                }
                return false;
            }

            // Create the ContactSensorData component inside the collision entity
            ecm->CreateComponent(collisionEntity,
                                 ignition::gazebo::components::ContactSensorData());

            // Get the parent entity of the collision, which is a link entity
            //            ignition::gazebo::Entity parentLinkEntity =
            //            ecm->ParentEntity(collisionEntity);

            //            std::cout << contactSensor.Element()->ToString("") << std::endl;

            // Create the ContactSensor component inside the sensor entity
            // Note: this is not strictly required. Let's leave it here in the case
            //       this component is used for something else in the Physics system.
            //            auto sensorEntity = sdfEntityCreator->CreateEntities(&contactSensor);
            //            assert(sensorEntity != ignition::gazebo::kNullEntity);
            //            sdfEntityCreator->SetParent(sensorEntity, parentLinkEntity);
            //            assert(ecm->EntityHasComponentType(
            //                sensorEntity,
            //                ignition::gazebo::components::ContactSensor().TypeId()));
        }

        // ========================================================
        // CREATE THE ROBOT OBJECT AND REGISTER IT IN THE SINGLETON
        // ========================================================

        // Create an IgnitionRobot object from the ecm
        auto ignRobot = std::make_shared<gympp::gazebo::IgnitionRobot>();
        if (!ignRobot->configureECM(modelEntity, ecm, eventManager)) {
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

        // Set the base link
        if (!modelData.baseLink.empty() && !ignRobot->setBaseFrame(modelData.baseLink)) {
            gymppError << "Failed to set '" << modelData.baseLink << "' as base link" << std::endl;
            return false;
        }

        // Handle the fixed base by creating a joint to fix the robot to the world.
        // In this case the pose is already set when inserting the model in the world,
        // and it is not necessary to also reset it through the robot interface.
        if (!ignRobot->setAsFloatingBase(!modelData.fixedPose)) {
            gymppError << "Failed to configure the robot as floating or fixed base" << std::endl;
            return false;
        }

        // In the first iteration, we need to notify that we finished to operate on the ECM
        if (pImpl->getServer()->Running() && pImpl->getServer()->Paused().value()) {
            ecmSingleton.notifyExecutorFinished(getWorldName());
        }
    }

    // Add the model in a list of allocated models
    pImpl->modelPendingToRemove.push_back(finalModelEntityName);

    gymppDebug << "New model successfully inserted in the world" << std::endl;
    return true;
}

bool GazeboWrapper::removeModel(const std::string& modelName)
{
    if (!initialized()) {
        gymppError << "The simulator was not initialized. Call initialize() first." << std::endl;
        return false;
    }

    gymppDebug << "Removing existing model '" << modelName << "'" << std::endl;

    // =======================
    // DELETE THE MODEL ENTITY
    // =======================

    // Check that the ECM has been stored by the plugin
    if (!ECMSingleton::get().valid(getWorldName())) {
        gymppError << "No ECM found for world '" << getWorldName()
                   << "'. Does your world file have the ECMProvider plugin?" << std::endl;
        return false;
    }

    // Get the ECM
    ignition::gazebo::EntityComponentManager* ecm = ECMSingleton::get().getECM(getWorldName());

    // Get the SdfEntityCreator that abstracts the ECM to easily create entities
    auto sdfEntityCreator = pImpl->getSdfEntityCreator(getWorldName());
    assert(sdfEntityCreator);

    // Try to find a matching entity
    auto modelEntity = ecm->EntityByComponents(ignition::gazebo::components::Model(),
                                               ignition::gazebo::components::Name(modelName));

    if (modelEntity == ignition::gazebo::kNullEntity) {
        gymppError << "Failed to find model '" << modelName << "' in the ECM. Model not removed."
                   << std::endl;
        return false;
    }

    // Request the removal of the model
    gymppDebug << "Requesting removal of entity [" << modelEntity << "]" << std::endl;
    sdfEntityCreator->RequestRemoveEntity(modelEntity);

    // ===================================
    // DELETE THE ROBOT FROM THE SINGLETON
    // ===================================

    // Remove the robot from the singleton
    if (!RobotSingleton::get().deleteRobot(modelName)) {
        gymppError << "The Robot object associated to the model was not in the singleton"
                   << std::endl;
    }

    assert(!RobotSingleton::get().exists(modelName));

    // Remove the model from the list of models pending to be removed
    auto it = std::find(
        pImpl->modelPendingToRemove.begin(), pImpl->modelPendingToRemove.end(), modelName);
    if (it != pImpl->modelPendingToRemove.end()) {
        pImpl->modelPendingToRemove.erase(it);
    }

    gymppDebug << "Model successfully removed from the world" << std::endl;
    return true;
}

bool GazeboWrapper::setupGazeboWorld(const std::string& worldFile)
{
    // Find and load the sdf file that contains the world
    if (!findAndLoadSdf(worldFile, pImpl->sdf)) {
        gymppError << "Failed to find and load sdf file '" << worldFile << "'" << std::endl;
        return false;
    }

    if (pImpl->sdf.WorldCount() != 1) {
        gymppError << "Only one world per world file is currently supported" << std::endl;
        return false;
    }

    // Assign to the world an unique name in order to avoid collisions with other instances
    // that run in the same process
    std::string worldName = pImpl->sdf.WorldByIndex(0)->Name();
    std::string suffix = std::to_string(reinterpret_cast<int64_t>(this));
    std::string newWorldName = worldName + "_" + suffix;
    const_cast<sdf::World*>(pImpl->sdf.WorldByIndex(0))->SetName(newWorldName);
    gymppDebug << "Configuring new simulation with the world '" << newWorldName << "'" << std::endl;

    // Create a new world with the scoped name
    auto renamedWorld = std::make_shared<sdf::Element>();
    renamedWorld->SetName("world");
    renamedWorld->AddAttribute("name", "string", "worldname", true, "world name");

    sdf::ParamPtr worldNameParam = renamedWorld->GetAttribute("name");
    worldNameParam->SetFromString(newWorldName);
    assert(worldNameParam->GetSet());

    sdf::ElementPtr child = pImpl->sdf.WorldByIndex(0)->Element()->GetFirstElement();

    // Add all the children
    while (child) {
        renamedWorld->InsertElement(child);
        child = child->GetNextElement();
    }

    // Remove the old world and insert the renamed one
    pImpl->sdf.WorldByIndex(0)->Element()->RemoveFromParent();
    pImpl->sdf.Element()->InsertElement(renamedWorld);
    assert(pImpl->sdf.WorldNameExists(newWorldName));

    // TODO: reading the update name from the SDFRoot does not work (similarly to the Model).
    //       Storing it in the Impl.
    pImpl->worldName = newWorldName;

    // Configure the physics profile
    if (!pImpl->setPhysics(pImpl->gazebo.physics)) {
        gymppError << "Failed to set physics profile" << std::endl;
        return false;
    }

    return true;
}
