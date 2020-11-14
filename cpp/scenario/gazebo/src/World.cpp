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
    std::shared_ptr<ignition::gazebo::SdfEntityCreator> sdfEntityCreator;

    using ModelName = std::string;
    std::unordered_map<ModelName, core::ModelPtr> models;

    struct
    {
        std::vector<std::string> modelNames;
    } buffers;
};

World::World()
    : pImpl{std::make_unique<Impl>()}
{}

World::~World() = default;

uint64_t World::id() const
{
    return std::hash<std::string>{}(this->name());
}

bool World::initialize(const ignition::gazebo::Entity worldEntity,
                       ignition::gazebo::EntityComponentManager* ecm,
                       ignition::gazebo::EventManager* eventManager)
{
    if (worldEntity == ignition::gazebo::kNullEntity || !ecm || !eventManager) {
        return false;
    }

    // Store the GazeboEntity resources
    m_ecm = ecm;
    m_entity = worldEntity;
    m_eventManager = eventManager;

    // Create the SdfEntityCreator
    pImpl->sdfEntityCreator = std::make_unique< //
        ignition::gazebo::SdfEntityCreator>(*ecm, *eventManager);

    return true;
}

bool World::createECMResources()
{
    // Store the time of creation (big bang)
    if (!m_ecm->EntityHasComponentType(
            m_entity, ignition::gazebo::components::Timestamp::typeId)) {
        utils::setComponentData<ignition::gazebo::components::Timestamp>(
            m_ecm, m_entity, std::chrono::steady_clock::duration::zero());
    }

    // Initialize the simulated time at 0.0 (Physics will then update it)
    if (!m_ecm->EntityHasComponentType(
            m_entity, ignition::gazebo::components::SimulatedTime::typeId)) {
        utils::setComponentData<ignition::gazebo::components::SimulatedTime>(
            m_ecm, m_entity, std::chrono::steady_clock::duration::zero());
    }

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
    const auto wrapped = sdf::SDF::WrapInRoot(pluginElement);

    // Trigger the plugin loading
    m_eventManager->Emit<ignition::gazebo::events::LoadPlugins>(m_entity,
                                                                wrapped);

    return true;
}

bool World::setPhysicsEngine(const PhysicsEngine engine)
{
    std::string libName;
    std::string className;

    switch (engine) {
        case PhysicsEngine::Dart:
            libName = "PhysicsSystem";
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
    const auto& gravity = utils::getExistingComponentData< //
        ignition::gazebo::components::Gravity>(m_ecm, m_entity);

    return utils::fromIgnitionVector(gravity);
}

bool World::setGravity(const std::array<double, 3>& gravity)
{
    const auto& simTimeAtWorldCreation = utils::getExistingComponentData<
        ignition::gazebo::components::Timestamp>(m_ecm, m_entity);

    const double simTimeAtWorldCreationInSeconds =
        utils::steadyClockDurationToDouble(simTimeAtWorldCreation);

    if (this->time() > simTimeAtWorldCreationInSeconds) {
        sError << "Physics already processed the world and its"
               << "parameters cannot be modified" << std::endl;
        return false;
    }

    utils::setExistingComponentData<ignition::gazebo::components::Gravity>(
        m_ecm, m_entity, utils::toIgnitionVector3(gravity));

    return true;
}

bool World::valid() const
{
    return this->validEntity();
}

double World::time() const
{
    const auto& simTime = utils::getExistingComponentData<
        ignition::gazebo::components::SimulatedTime>(m_ecm, m_entity);

    return utils::steadyClockDurationToDouble(simTime);
}

std::string World::name() const
{
    const std::string& worldName = utils::getExistingComponentData< //
        ignition::gazebo::components::Name>(m_ecm, m_entity);

    return worldName;
}

std::vector<std::string> World::modelNames() const
{
    pImpl->buffers.modelNames.clear();

    m_ecm->Each<ignition::gazebo::components::Name,
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
            if (parentEntityComponent->Data() != m_entity) {
                return true;
            }

            pImpl->buffers.modelNames.push_back(nameComponent->Data());
            return true;
        });

    return pImpl->buffers.modelNames;
}

scenario::core::ModelPtr World::getModel(const std::string& modelName) const
{
    if (pImpl->models.find(modelName) != pImpl->models.end()) {
        assert(pImpl->models.at(modelName));
        return pImpl->models.at(modelName);
    }

    // Find the model entity
    const auto modelEntity = m_ecm->EntityByComponents(
        ignition::gazebo::components::Name(modelName),
        ignition::gazebo::components::Model(),
        ignition::gazebo::components::ParentEntity(m_entity));

    if (modelEntity == ignition::gazebo::kNullEntity) {
        throw exceptions::ModelNotFound(modelName);
    }

    // Create and initialize the model
    auto model = std::make_shared<scenario::gazebo::Model>();
    model->initialize(modelEntity, m_ecm, m_eventManager);

    pImpl->models[modelName] = model;
    return pImpl->models[modelName];
}

bool World::insertModel(const std::string& modelFile,
                        const core::Pose& pose,
                        const std::string& overrideModelName)
{
    const std::shared_ptr<sdf::Root> modelSdfRoot =
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
    const std::vector<std::string>& modelNames = this->modelNames();
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
    pImpl->sdfEntityCreator->SetParent(modelEntity, m_entity);

    {
        // Check that the model name is correct
        assert(modelSdfRoot->ModelCount() == 1);
        std::string modelNameSDF =
            modelSdfRoot->ModelByIndex(ModelIndex)->Name();
        std::string modelNameEntity = utils::getExistingComponentData< //
            ignition::gazebo::components::Name>(m_ecm, modelEntity);
        assert(modelNameSDF == modelNameEntity);
    }

    // Create the model
    auto model = std::make_shared<scenario::gazebo::Model>();

    // Initialize the model
    if (!model->initialize(modelEntity, m_ecm, m_eventManager)) {
        sError << "Failed to initialize the model" << std::endl;
        if (!this->removeModel(finalModelEntityName)) {
            sError << "Failed to remove temporary model after failure"
                   << std::endl;
        }
        return false;
    }

    // Create required model resources. This call prepares all the necessary
    // components in the ECM to make our bindings work.
    if (!model->createECMResources()) {
        sError << "Failed to initialize ECM model resources" << std::endl;
        return false;
    }

    // Set the initial model pose.
    // We directly override the Pose component instead of using
    // Model::resetBasePose because it would just store a pose command that
    // needs to be processed by the Physics system. Overriding the component,
    // instead, has direct effect.
    if (pose != core::Pose::Identity()) {
        utils::setComponentData<ignition::gazebo::components::Pose>(
            m_ecm, modelEntity, utils::toIgnitionPose(pose));
    }

    return true;
}

bool World::removeModel(const std::string& modelName)
{
    const auto modelEntity = m_ecm->EntityByComponents(
        ignition::gazebo::components::Name(modelName),
        ignition::gazebo::components::Model(),
        ignition::gazebo::components::ParentEntity(m_entity));

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
