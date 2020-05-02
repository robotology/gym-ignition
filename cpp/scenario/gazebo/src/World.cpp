/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
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

#include "scenario/gazebo/World.h"
#include "scenario/gazebo/Log.h"
#include "scenario/gazebo/Model.h"
#include "scenario/gazebo/components/SimulatedTime.h"
#include "scenario/gazebo/components/Timestamp.h"
#include "scenario/gazebo/exceptions.h"
#include "scenario/gazebo/helpers.h"

#include <ignition/common/Event.hh>
#include <ignition/gazebo/Events.hh>
#include <ignition/gazebo/SdfEntityCreator.hh>
#include <ignition/gazebo/components/Gravity.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/Element.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>

#include <algorithm>
#include <cassert>
#include <functional>
#include <unordered_map>

using namespace scenario::gazebo;

class World::Impl
{
public:
    ignition::gazebo::EventManager* eventManager = nullptr;
    ignition::gazebo::EntityComponentManager* ecm = nullptr;

    ignition::gazebo::Entity worldEntity = ignition::gazebo::kNullEntity;
    std::shared_ptr<ignition::gazebo::SdfEntityCreator> sdfEntityCreator;

    using ModelName = std::string;
    std::unordered_map<ModelName, ModelPtr> models;

    struct
    {
        std::vector<std::string> modelNames;
    } buffers;
};

World::World()
    : pImpl{std::make_unique<Impl>()}
{}

uint64_t World::id() const
{
    return std::hash<std::string>{}(this->name());
}

World::~World() = default;

bool World::initialize(const ignition::gazebo::Entity worldEntity,
                       ignition::gazebo::EntityComponentManager* ecm,
                       ignition::gazebo::EventManager* eventManager)
{
    if (worldEntity == ignition::gazebo::kNullEntity || !ecm || !eventManager) {
        return false;
    }

    // Store the ECM resources
    pImpl->ecm = ecm;
    pImpl->worldEntity = worldEntity;
    pImpl->eventManager = eventManager;

    // Create the SdfEntityCreator
    pImpl->sdfEntityCreator = std::make_unique< //
        ignition::gazebo::SdfEntityCreator>(*ecm, *eventManager);

    return true;
}

bool World::createECMResources()
{
    return true;
}

bool World::insertWorldPlugin(const std::string& libName,
                              const std::string& className,
                              const std::string& context)
{
    // Create a new <plugin name="" filename=""> element without context
    sdf::ElementPtr pluginElement =
        utils::getPluginSDFElement(libName, className);

    // Insert the context into the plugin element
    if (!context.empty()) {

        // Try to get the sdf::Root (it will alredy print an error if it fails)
        auto contextRoot = utils::getSdfRootFromString(context);

        if (!contextRoot) {
            return false;
        }

        // Get the first element of the context
        // (stripping out the <sdf> container)
        auto contextNextElement = contextRoot->Element()->GetFirstElement();

        // Insert the plugin context elements
        while (contextNextElement) {
            pluginElement->InsertElement(contextNextElement);
            contextNextElement = contextNextElement->GetNextElement();
        }
    }

    // The plugin element must be wrapped in another element, otherwise
    // who receives it does not get the additional context
    auto wrapped = sdf::SDF::WrapInRoot(pluginElement);

    // Trigger the plugin loading
    pImpl->eventManager->Emit<ignition::gazebo::events::LoadPlugins>(
        pImpl->worldEntity, wrapped);

    return true;
}

bool World::setPhysicsEngine(const PhysicsEngine engine)
{
    std::string libName;
    std::string className;

    switch (engine) {
        case PhysicsEngine::Dart:
            libName = "libPhysicsSystem.so";
            className = "scenario::plugins::gazebo::Physics";
            break;
    }

    if (!this->insertWorldPlugin(libName, className)) {
        sError << "Failed to insert the physics plugin" << std::endl;
        return false;
    }

    return true;
}

std::array<double, 3> World::gravity() const
{
    auto gravity = utils::getExistingComponentData< //
        ignition::gazebo::components::Gravity>(pImpl->ecm, pImpl->worldEntity);

    return utils::fromIgnitionVector(gravity);
}

bool World::setGravity(const std::array<double, 3>& gravity)
{
    auto& simTimeAtWorldCreation = utils::getExistingComponentData<
        ignition::gazebo::components::Timestamp>(pImpl->ecm,
                                                 pImpl->worldEntity);

    const double simTimeAtWorldCreationInSeconds =
        utils::steadyClockDurationToDouble(simTimeAtWorldCreation);

    if (this->time() > simTimeAtWorldCreationInSeconds) {
        sError << "Physics already processed the world and its"
               << "parameters cannot be modified" << std::endl;
        return false;
    }

    utils::setExistingComponentData<ignition::gazebo::components::Gravity>(
        pImpl->ecm, pImpl->worldEntity, utils::toIgnitionVector3(gravity));

    return true;
}

double World::time() const
{
    auto simTime = utils::getExistingComponentData< //
        ignition::gazebo::components::SimulatedTime>(pImpl->ecm,
                                                     pImpl->worldEntity);

    return utils::steadyClockDurationToDouble(simTime);
}

std::string World::name() const
{
    std::string worldName = utils::getExistingComponentData< //
        ignition::gazebo::components::Name>(pImpl->ecm, pImpl->worldEntity);

    return worldName;
}

std::vector<std::string> World::modelNames() const
{
    pImpl->buffers.modelNames.clear();

    pImpl->ecm->Each<ignition::gazebo::components::Name,
                     ignition::gazebo::components::Model,
                     ignition::gazebo::components::ParentEntity>(
        [&](const ignition::gazebo::Entity& /*entity*/,
            ignition::gazebo::components::Name* nameComponent,
            ignition::gazebo::components::Model* /*modelComponent*/,
            ignition::gazebo::components::ParentEntity* parentEntityComponent)
            -> bool {
            assert(nameComponent);
            assert(parentEntityComponent);

            // Discard models not belonging to this world
            if (parentEntityComponent->Data() != pImpl->worldEntity) {
                return true;
            }

            pImpl->buffers.modelNames.push_back(nameComponent->Data());
            return true;
        });

    return pImpl->buffers.modelNames;
}

scenario::gazebo::ModelPtr World::getModel(const std::string& modelName) const
{
    if (pImpl->models.find(modelName) != pImpl->models.end()) {
        assert(pImpl->models.at(modelName));
        return pImpl->models.at(modelName);
    }

    // Find the model entity
    auto modelEntity = pImpl->ecm->EntityByComponents(
        ignition::gazebo::components::Name(modelName),
        ignition::gazebo::components::Model(),
        ignition::gazebo::components::ParentEntity(pImpl->worldEntity));

    if (modelEntity == ignition::gazebo::kNullEntity) {
        throw exceptions::ModelNotFound(modelName);
    }

    // Create and initialize the model
    pImpl->models[modelName] = std::make_shared<scenario::gazebo::Model>();
    pImpl->models[modelName]->initialize(
        modelEntity, pImpl->ecm, pImpl->eventManager);

    return pImpl->models[modelName];
}

bool World::insertModel(const std::string& modelFile,
                        const base::Pose& pose,
                        const std::string& overrideModelName)
{
    std::shared_ptr<sdf::Root> modelSdfRoot =
        utils::getSdfRootFromFile(modelFile);

    if (!modelSdfRoot) {
        return false;
    }

    if (modelSdfRoot->ModelCount() != 1) {
        sError << "The SDF file contains more than one model" << std::endl;
        return false;
    }

    constexpr size_t ModelIndex = 0;

    // Every SDF model has a name. In order to insert multiple models from the
    // same SDF file, the modelData struct allows providing a scoped name.
    std::string finalModelEntityName;

    // Get the final name of the model
    if (overrideModelName.empty()) {
        assert(modelSdfRoot->ModelByIndex(ModelIndex));
        finalModelEntityName = modelSdfRoot->ModelByIndex(ModelIndex)->Name();
    }
    else {
        finalModelEntityName = overrideModelName;
    }

    // Check for model name clash
    const std::vector<std::string> modelNames = this->modelNames();
    if (std::find(modelNames.begin(), modelNames.end(), finalModelEntityName)
        != modelNames.end()) {
        sError << "Failed to insert model '" << finalModelEntityName
               << "'. Another entity with the same name already exists."
               << std::endl;
        return false;
    }

    // Rename the model.
    // NOTE: The following is not enough because the name is not serialized to
    //       string. We need also to operate directly on the raw element.
    const_cast<sdf::Model*>(modelSdfRoot->ModelByIndex(ModelIndex))
        ->SetName(finalModelEntityName);

    // Update the name in the sdf model. This is necessary because model plugins
    // are loaded right before the creation of the model entity and, instead of
    // receiving the model entity name, they receive the model sdf name.
    if (!utils::renameSDFModel(
            *modelSdfRoot, finalModelEntityName, ModelIndex)) {
        sError << "Failed to rename SDF model" << std::endl;
        return false;
    }

    if (utils::verboseFromEnvironment()) {
        sDebug << "Inserting a model from the following SDF:" << std::endl;
        std::cout << modelSdfRoot->Element()->ToString("") << std::endl;
    }

    // Create the model entity
    ignition::gazebo::Entity modelEntity;
    modelEntity = pImpl->sdfEntityCreator->CreateEntities(
        modelSdfRoot->ModelByIndex(ModelIndex));

    // Attach the model entity to the world entity
    pImpl->sdfEntityCreator->SetParent(modelEntity, pImpl->worldEntity);

    {
        // Check that the model name is correct
        assert(modelSdfRoot->ModelCount() == 1);
        std::string modelNameSDF =
            modelSdfRoot->ModelByIndex(ModelIndex)->Name();
        std::string modelNameEntity = utils::getExistingComponentData< //
            ignition::gazebo::components::Name>(pImpl->ecm, modelEntity);
        assert(modelNameSDF == modelNameEntity);
    }

    // Set the initial model pose
    if (pose != base::Pose::Identity()) {
        utils::setComponentData<ignition::gazebo::components::Pose>(
            pImpl->ecm, modelEntity, utils::toIgnitionPose(pose));
    }

    // Get the current time
    auto time = utils::getExistingComponentData<
        ignition::gazebo::components::Timestamp>(pImpl->ecm,
                                                 pImpl->worldEntity);

    // Insert the time of creation of the model
    pImpl->ecm->CreateComponent(modelEntity,
                                ignition::gazebo::components::Timestamp(time));

    // Create the model
    auto model = std::make_shared<scenario::gazebo::Model>();
    model->initialize(modelEntity, pImpl->ecm, pImpl->eventManager);

    // Create required model resources. This call prepares all the necessary
    // components in the ECM to make our bindings work.
    if (!model->createECMResources()) {
        sError << "Failed to initialize ECM model resources" << std::endl;
        return false;
    }

    return true;
}

bool World::removeModel(const std::string& modelName)
{
    auto modelEntity = pImpl->ecm->EntityByComponents(
        ignition::gazebo::components::Name(modelName),
        ignition::gazebo::components::Model(),
        ignition::gazebo::components::ParentEntity(pImpl->worldEntity));

    if (modelEntity == ignition::gazebo::kNullEntity) {
        sError << "Model '" << modelName << "' not found in the world"
               << std::endl;
        return false;
    }

    // Request the removal of the model
    sDebug << "Requesting removal of entity [" << modelEntity << "]"
           << std::endl;
    pImpl->sdfEntityCreator->RequestRemoveEntity(modelEntity);

    // Remove the cached model
    pImpl->models.erase(modelName);

    return true;
}
