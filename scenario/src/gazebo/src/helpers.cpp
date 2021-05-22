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

#include "scenario/gazebo/helpers.h"
#include "ignition/common/Util.hh"
#include "scenario/gazebo/Log.h"
#include "scenario/gazebo/components/Timestamp.h"

#include <Eigen/Dense>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/msgs/contact.pb.h>
#include <sdf/Error.hh>
#include <sdf/Model.hh>
#include <sdf/Physics.hh>
#include <sdf/World.hh>

#include <cassert>
#include <sstream>

using namespace scenario::gazebo;

std::shared_ptr<sdf::Root>
utils::getSdfRootFromFile(const std::string& sdfFileName)
{
    // NOTE: there's a double free error if we use std::optional
    // auto root = std::make_optional<sdf::Root>();

    auto root = std::make_shared<sdf::Root>();
    auto errors = root->Load(sdfFileName);

    if (!errors.empty()) {
        sError << "Failed to load sdf file " << sdfFileName << std::endl;

        for (const auto& error : errors) {
            sError << error << std::endl;
        }
        return {};
    }

    return root;
}

std::shared_ptr<sdf::Root>
utils::getSdfRootFromString(const std::string& sdfString)
{
    // NOTE: there's a double free error if we use std::optional
    // auto root = std::make_optional<sdf::Root>();

    auto root = std::make_shared<sdf::Root>();
    auto errors = root->LoadSdfString(sdfString);

    if (!errors.empty()) {
        sError << "Failed to load sdf string" << std::endl;

        for (const auto& error : errors) {
            sError << error << std::endl;
        }
        return {};
    }

    return root;
}

bool utils::verboseFromEnvironment()
{
    std::string envVarContent;
    ignition::common::env(ScenarioVerboseEnvVar, envVarContent);

    return envVarContent == "1";
}

std::chrono::steady_clock::duration
utils::doubleToSteadyClockDuration(const double durationInSeconds)
{
    using namespace std::chrono;

    // Create a floating-point duration
    const auto durationInSecondsChrono = duration<double>(durationInSeconds);

    // Cast to integral duration
    return round<steady_clock::duration>(durationInSecondsChrono);
}

double utils::steadyClockDurationToDouble(
    const std::chrono::steady_clock::duration duration)
{
    // This is the value in seconds
    return std::chrono::duration<double>(duration).count();
}

void utils::rowMajorToColumnMajor(std::vector<double>& input,
                                  const long rows,
                                  const long cols)
{
    using namespace Eigen;
    using RowMajorMat = Matrix<double, Dynamic, Dynamic, RowMajor>;
    using ColMajorMat = Matrix<double, Dynamic, Dynamic, ColMajor>;

    Map<RowMajorMat> rowMajorView(input.data(), rows, cols);
    Map<ColMajorMat> colMajorView(input.data(), rows, cols);

    colMajorView = rowMajorView.eval();
}

bool utils::parentModelJustCreated(const GazeboEntity& gazeboEntity)
{
    // Get the parent world
    const auto& world = utils::getParentWorld(gazeboEntity);

    // Get the entity of the parent model
    const auto parentModelEntity = [&]() -> ignition::gazebo::Entity {
        return gazeboEntity.ecm()->EntityHasComponentType(
                   gazeboEntity.entity(),
                   ignition::gazebo::components::Model::typeId)
                   ? gazeboEntity.entity()
                   : utils::getParentModel(gazeboEntity)->entity();
    }();

    assert(world);
    assert(parentModelEntity != ignition::gazebo::kNullEntity);

    // Get the time the model was inserted
    const auto& simTimeAtModelCreation = utils::getExistingComponentData<
        ignition::gazebo::components::Timestamp>(gazeboEntity.ecm(),
                                                 parentModelEntity);

    const double simTimeAtModelCreationInSeconds =
        utils::steadyClockDurationToDouble(simTimeAtModelCreation);

    return world->time() == simTimeAtModelCreationInSeconds;
}

scenario::core::Pose
utils::fromIgnitionPose(const ignition::math::Pose3d& ignitionPose)
{
    core::Pose pose;

    pose.position[0] = ignitionPose.Pos().X();
    pose.position[1] = ignitionPose.Pos().Y();
    pose.position[2] = ignitionPose.Pos().Z();

    pose.orientation[0] = ignitionPose.Rot().W();
    pose.orientation[1] = ignitionPose.Rot().X();
    pose.orientation[2] = ignitionPose.Rot().Y();
    pose.orientation[3] = ignitionPose.Rot().Z();

    return pose;
}

ignition::math::Pose3d utils::toIgnitionPose(const scenario::core::Pose& pose)
{
    ignition::math::Pose3d ignitionPose;

    ignitionPose.Pos() = ignition::math::Vector3d(
        pose.position[0], pose.position[1], pose.position[2]);

    ignitionPose.Rot() = ignition::math::Quaterniond(pose.orientation[0],
                                                     pose.orientation[1],
                                                     pose.orientation[2],
                                                     pose.orientation[3]);

    return ignitionPose;
}

scenario::core::Contact
utils::fromIgnitionContactMsgs(ignition::gazebo::EntityComponentManager* ecm,
                               const ignition::msgs::Contact& contactMsg)
{
    auto getEntityName =
        [&](const ignition::gazebo::Entity entity) -> std::string {
        return utils::getExistingComponentData<
            ignition::gazebo::components::Name>(ecm, entity);
    };

    // Get the names of the links in contact following:
    // collision entity -> collision link -> link name
    const auto collisionEntityA = contactMsg.collision1().id();
    const auto collisionEntityB = contactMsg.collision2().id();

    const auto linkEntityA = ecm->ParentEntity(collisionEntityA);
    const auto linkEntityB = ecm->ParentEntity(collisionEntityB);
    const std::string linkNameA = getEntityName(linkEntityA);
    const std::string linkNameB = getEntityName(linkEntityB);

    // Return the link names scoped with the model name
    const auto modelEntityA = ecm->ParentEntity(linkEntityA);
    const auto modelEntityB = ecm->ParentEntity(linkEntityB);
    const std::string modelNameA = getEntityName(modelEntityA);
    const std::string modelNameB = getEntityName(modelEntityB);

    const std::string scopedBodyA = modelNameA + "::" + linkNameA;
    const std::string scopedBodyB = modelNameB + "::" + linkNameB;

    // Returned data structure
    scenario::core::Contact contact;
    contact.bodyA = scopedBodyA;
    contact.bodyB = scopedBodyB;

    // Dimensions of contact points data must match
    const auto numOfDepths = contactMsg.depth_size();
    const auto numOfNormals = contactMsg.normal_size();
    const auto numOfWrenches = contactMsg.wrench_size();
    const auto numOfPositions = contactMsg.position_size();

    int numOfPoints = numOfDepths;
    assert(numOfPoints == numOfNormals);
    assert(numOfPoints == numOfWrenches);
    assert(numOfPoints == numOfPositions);

    auto fromIgnMsg =
        [](const ignition::msgs::Vector3d& vec) -> std::array<double, 3> {
        return {vec.x(), vec.y(), vec.z()};
    };

    // Get all the contact points data
    for (int pointIdx = 0; pointIdx < numOfPoints; ++pointIdx) {
        // Create a contact point
        scenario::core::ContactPoint contactPoint;
        contactPoint.depth = contactMsg.depth(pointIdx);
        contactPoint.normal = fromIgnMsg(contactMsg.normal(pointIdx));
        contactPoint.position = fromIgnMsg(contactMsg.position(pointIdx));

        // Get the wrench acting on bodyA
        const ignition::msgs::JointWrench wrench = contactMsg.wrench(pointIdx);
        const auto& wrench1 = wrench.body_1_wrench();
        contactPoint.force = fromIgnMsg(wrench1.force());
        contactPoint.torque = fromIgnMsg(wrench1.torque());

        // Store the contact point
        contact.points.push_back(contactPoint);
    }

    return contact;
}

std::vector<scenario::core::Contact>
utils::fromIgnitionContactsMsgs(ignition::gazebo::EntityComponentManager* ecm,
                                const ignition::msgs::Contacts& contactsMsg)
{
    std::vector<core::Contact> contacts;

    for (int contactIdx = 0; contactIdx < contactsMsg.contact_size();
         ++contactIdx) {
        contacts.push_back(
            fromIgnitionContactMsgs(ecm, contactsMsg.contact(contactIdx)));
    }

    return contacts;
}

sdf::World utils::renameSDFWorld(const sdf::World& world,
                                 const std::string& newWorldName)
{
    const size_t initialNrOfModels = world.ModelCount();

    // Create a new world with the scoped name
    auto renamedWorldElement = std::make_shared<sdf::Element>();
    renamedWorldElement->SetName("world");
    renamedWorldElement->AddAttribute("name", "string", newWorldName, true);

    // Get the element of the world
    sdf::ElementPtr child = world.Element()->GetFirstElement();

    // Add all the children
    while (child) {
        renamedWorldElement->InsertElement(child);
        child = child->GetNextElement();
    }

    // Create a new world
    sdf::World renamedWorld;

    auto errors = renamedWorld.Load(renamedWorldElement);

    if (!errors.empty()) {
        sError << "Failed to create the World from the element" << std::endl;

        for (const auto& error : errors) {
            sError << error << std::endl;
        }
        return {};
    }

    if (renamedWorld.Name() != newWorldName) {
        sError << "Failed to rename the world" << std::endl;
        return {};
    }

    if (renamedWorld.ModelCount() != initialNrOfModels) {
        sError << "Failed to copy all models to the new world" << std::endl;
        return {};
    }

    return renamedWorld;
}

bool utils::renameSDFModel(sdf::Root& sdfRoot, const std::string& newModelName)
{
    // Create a new model with the scoped name
    auto renamedModel = std::make_shared<sdf::Element>();
    renamedModel->SetName("model");
    renamedModel->AddAttribute("name", "string", newModelName, true);

    if (!sdfRoot.Model()) {
        sError << "The sdf Root does not contain any model" << std::endl;
        return false;
    }

    // Get the first child of the original model element
    sdf::ElementPtr child = sdfRoot.Model()->Element()->GetFirstElement();

    // Add all the children to the renamed model element
    while (child) {
        renamedModel->InsertElement(child);
        child->SetParent(renamedModel);
        child = child->GetNextElement();
    }

    // Remove the old model
    auto originalModelElement = sdfRoot.Model()->Element();
    originalModelElement->RemoveFromParent();

    // Insert the renamed model
    renamedModel->SetParent(sdfRoot.Element());
    sdfRoot.Element()->InsertElement(renamedModel);

    if (sdfRoot.Model()->Name() != newModelName) {
        sError << "Failed to insert renamed model in SDF root" << std::endl;
        return false;
    }

    return true;
}

namespace scenario::gazebo::utils {
    /**
     * Convert a double to a string ignoring the current locale.
     *
     * Furthermore, set it with precision 25 to ensure that the
     * string can be converted back exactly to the double that
     * generated it.
     */
    std::string toExactStringNoLocale(const double in);
} // namespace scenario::gazebo::utils

std::string utils::toExactStringNoLocale(const double in)
{
    std::ostringstream ss;
    ss.imbue(std::locale::classic());
    ss << std::setprecision(25) << in;
    return ss.str();
}

bool utils::updateSDFPhysics(sdf::Root& sdfRoot,
                             const double maxStepSize,
                             const double rtf,
                             const double realTimeUpdateRate,
                             const size_t worldIndex)
{
    if (rtf <= 0) {
        sError << "Invalid RTF value (" << rtf << ")" << std::endl;
        return false;
    }

    if (maxStepSize <= 0) {
        sError << "Invalid physics max step size (" << maxStepSize << ")"
               << std::endl;
        return false;
    }

    const sdf::World* world = sdfRoot.WorldByIndex(worldIndex);

    if (world->PhysicsCount() != 1) {
        sError << "Found more than one physics profile" << std::endl;
        return false;
    }

    // Set the physics properties using the helper.
    // It sets the internal value but it does not update the DOM.
    auto* physics = const_cast<sdf::Physics*>(world->PhysicsByIndex(0));
    physics->SetMaxStepSize(maxStepSize);
    physics->SetRealTimeFactor(rtf);

    // Update the DOM operating directly on the raw elements
    sdf::ElementPtr worldElement = world->Element();

    sdf::ElementPtr physicsElement = worldElement->GetElement("physics");
    assert(physicsElement);

    sdf::ElementPtr max_step_size = physicsElement->GetElement("max_step_size");
    max_step_size->AddValue("double", toExactStringNoLocale(maxStepSize), true);

    sdf::ElementPtr real_time_update_rate =
        physicsElement->GetElement("real_time_update_rate");
    real_time_update_rate->AddValue(
        "double", toExactStringNoLocale(realTimeUpdateRate), true);

    sdf::ElementPtr real_time_factor =
        physicsElement->GetElement("real_time_factor");
    real_time_factor->AddValue("double", toExactStringNoLocale(rtf), true);

    return true;
}

sdf::ElementPtr utils::getPluginSDFElement(const std::string& libName,
                                           const std::string& className)
{
    // Create the plugin SDF element
    auto pluginElement = std::make_shared<sdf::Element>();

    // Initialize the attributes
    pluginElement->SetName("plugin");
    pluginElement->AddAttribute(
        "name", "string", className, true, "plugin name");
    pluginElement->AddAttribute(
        "filename", "string", libName, true, "pluginfilename");

    // Create the plugin description
    pluginElement->AddElementDescription(pluginElement->Clone());

    return pluginElement;
}

sdf::JointType utils::toSdf(const scenario::core::JointType type)
{
    sdf::JointType sdfType;

    switch (type) {
        case core::JointType::Fixed:
            sdfType = sdf::JointType::FIXED;
            break;
        case core::JointType::Revolute:
            sdfType = sdf::JointType::REVOLUTE;
            break;
        case core::JointType::Prismatic:
            sdfType = sdf::JointType::PRISMATIC;
            break;
        case core::JointType::Ball:
            sdfType = sdf::JointType::BALL;
            break;
        default:
            sError << "Joint type not recognized" << std::endl;
            sdfType = sdf::JointType::INVALID;
            break;
    }

    return sdfType;
}

scenario::core::JointType utils::fromSdf(const sdf::JointType sdfType)
{
    core::JointType type;

    switch (sdfType) {
        case sdf::JointType::FIXED:
            type = core::JointType::Fixed;
            break;
        case sdf::JointType::REVOLUTE:
            type = core::JointType::Revolute;
            break;
        case sdf::JointType::PRISMATIC:
            type = core::JointType::Prismatic;
            break;
        case sdf::JointType::BALL:
            type = core::JointType::Ball;
            break;
        default:
            sError << "Joint type not recognized" << std::endl;
            type = core::JointType::Invalid;
            break;
    }

    return type;
}

ignition::math::Vector3d utils::fromModelToBaseLinearVelocity(
    const ignition::math::Vector3d& linModelVelocity,
    const ignition::math::Vector3d& angModelVelocity,
    const ignition::math::Pose3d& M_H_B,
    const ignition::math::Quaterniond& W_R_B)
{
    // Extract the rotation and the position of the model wrt to the base
    auto B_R_M = M_H_B.Rot().Inverse();
    auto M_o_B = M_H_B.Pos();
    auto B_o_M = -B_R_M * M_o_B;

    // Compute the base linear velocity
    const ignition::math::Vector3d linBaseVelocity =
        linModelVelocity - angModelVelocity.Cross(W_R_B * B_o_M);

    // Return the linear part of the mixed velocity of the base
    return linBaseVelocity;
}

ignition::math::Vector3d utils::fromBaseToModelLinearVelocity(
    const ignition::math::Vector3d& linBaseVelocity,
    const ignition::math::Vector3d& angBaseVelocity,
    const ignition::math::Pose3d& M_H_B,
    const ignition::math::Quaterniond& W_R_B)
{
    // Extract the rotation and the position of the model wrt to the base
    auto B_R_M = M_H_B.Rot().Inverse();
    auto M_o_B = M_H_B.Pos();

    // Compute the model linear velocity
    const ignition::math::Vector3d linModelVelocity =
        linBaseVelocity - angBaseVelocity.Cross(W_R_B * B_R_M * M_o_B);

    // Return the linear part of the mixed velocity of the model
    return linModelVelocity;
}

std::shared_ptr<World> utils::getParentWorld(const GazeboEntity& gazeboEntity)
{
    if (!gazeboEntity.validEntity()) {
        sError << "The GazeboEntity is not valid" << std::endl;
        return nullptr;
    }

    auto worldEntity = getFirstParentEntityWithComponent< //
        ignition::gazebo::components::World>(gazeboEntity.ecm(),
                                             gazeboEntity.entity());

    if (worldEntity == ignition::gazebo::kNullEntity) {
        sError << "Failed to find parent world entity" << std::endl;
        return nullptr;
    }

    auto world = std::make_shared<World>();

    if (!world->initialize(
            worldEntity, gazeboEntity.ecm(), gazeboEntity.eventManager())) {
        sError << "Failed to initialize world" << std::endl;
        return nullptr;
    }

    return world;
}

std::shared_ptr<Model> utils::getParentModel(const GazeboEntity& gazeboEntity)
{
    if (!gazeboEntity.validEntity()) {
        sError << "The GazeboEntity is not valid" << std::endl;
        return nullptr;
    }

    auto modelEntity = getFirstParentEntityWithComponent< //
        ignition::gazebo::components::Model>(gazeboEntity.ecm(),
                                             gazeboEntity.entity());

    if (modelEntity == ignition::gazebo::kNullEntity) {
        sError << "Failed to find parent model entity" << std::endl;
        return nullptr;
    }

    auto model = std::make_shared<Model>();

    if (!model->initialize(
            modelEntity, gazeboEntity.ecm(), gazeboEntity.eventManager())) {
        sError << "Failed to initialize model" << std::endl;
        return nullptr;
    }

    return model;
}
