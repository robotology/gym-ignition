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

#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/msgs/contact.pb.h>
#include <sdf/Error.hh>
#include <sdf/Model.hh>
#include <sdf/Physics.hh>
#include <sdf/World.hh>

#include <cassert>

using namespace scenario::gazebo;

std::shared_ptr<sdf::Root>
utils::getSdfRootFromFile(const std::string& sdfFileName)
{
    // NOTE: there's a double free error if we use std::optional
    // auto root = std::make_optional<sdf::Root>();

    auto root = std::make_shared<sdf::Root>();
    auto errors = root->Load(sdfFileName);

    if (!errors.empty()) {
        gymppError << "Failed to load sdf file " << sdfFileName << std::endl;

        for (const auto& error : errors) {
            gymppError << error << std::endl;
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
        gymppError << "Failed to load sdf string" << std::endl;

        for (const auto& error : errors) {
            gymppError << error << std::endl;
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

scenario::base::Pose
utils::fromIgnitionPose(const ignition::math::Pose3d& ignitionPose)
{
    base::Pose pose;

    pose.position[0] = ignitionPose.Pos().X();
    pose.position[1] = ignitionPose.Pos().Y();
    pose.position[2] = ignitionPose.Pos().Z();

    pose.orientation[0] = ignitionPose.Rot().W();
    pose.orientation[1] = ignitionPose.Rot().X();
    pose.orientation[2] = ignitionPose.Rot().Y();
    pose.orientation[3] = ignitionPose.Rot().Z();

    return pose;
}

ignition::math::Pose3d utils::toIgnitionPose(const scenario::base::Pose& pose)
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

std::vector<scenario::base::ContactData>
utils::fromIgnitionContactsMsgs(ignition::gazebo::EntityComponentManager* ecm,
                                const ignition::msgs::Contacts& contactsMsg)
{
    auto getEntityName =
        [&](const ignition::gazebo::Entity entity) -> std::string {
        auto nameComponent =
            ecm->Component<ignition::gazebo::components::Name>(entity);
        assert(nameComponent);
        return nameComponent->Data();
    };

    std::vector<base::ContactData> contacts;

    for (int i = 0; i < contactsMsg.contact_size(); ++i) {
        // Extract the contact object
        const ignition::msgs::Contact& contactData = contactsMsg.contact(i);

        auto collisionEntityA = contactData.collision1().id();
        auto collisionEntityB = contactData.collision2().id();

        auto linkEntityA = ecm->ParentEntity(collisionEntityA);
        auto linkEntityB = ecm->ParentEntity(collisionEntityB);
        std::string linkNameA = getEntityName(linkEntityA);
        std::string linkNameB = getEntityName(linkEntityB);

        auto modelEntityA = ecm->ParentEntity(linkEntityA);
        auto modelEntityB = ecm->ParentEntity(linkEntityB);
        std::string modelNameA = getEntityName(modelEntityA);
        std::string modelNameB = getEntityName(modelEntityB);

        std::string scopedBodyA = modelNameA + "::" + linkNameA;
        std::string scopedBodyB = modelNameB + "::" + linkNameB;

        // Fill the contact data
        base::ContactData contact;
        contact.bodyA = scopedBodyA;
        contact.bodyB = scopedBodyB;
        contact.position[0] = contactData.position(i).x();
        contact.position[1] = contactData.position(i).y();
        contact.position[2] = contactData.position(i).z();

        // TODO: fill the normal, contacty depth, wrench magnitude
        assert(contactData.depth_size() == 0);
        assert(contactData.normal_size() == 0);
        assert(contactData.wrench_size() == 0);

        // Add the new contact data to the contacts vector
        contacts.push_back(contact);
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
        gymppError << "Failed to create the World from the element"
                   << std::endl;

        for (const auto& error : errors) {
            gymppError << error << std::endl;
        }
        return {};
    }

    if (renamedWorld.Name() != newWorldName) {
        gymppError << "Failed to rename the world" << std::endl;
        return {};
    }

    if (renamedWorld.ModelCount() != initialNrOfModels) {
        gymppError << "Failed to copy all models to the new world" << std::endl;
        return {};
    }

    return renamedWorld;
}

bool utils::renameSDFModel(sdf::Root& sdfRoot,
                           const std::string& newModelName,
                           size_t modelIndex)
{
    const size_t initialNrOfModels = sdfRoot.ModelCount();

    // Create a new model with the scoped name
    sdf::ElementPtr renamedModel(new sdf::Element);
    renamedModel->SetName("model");
    renamedModel->AddAttribute("name", "string", newModelName, true);

    // Get the first child of the original model element
    sdf::ElementPtr child =
        sdfRoot.ModelByIndex(modelIndex)->Element()->GetFirstElement();

    // Add all the children to the renamed model element
    while (child) {
        renamedModel->InsertElement(child);
        child->SetParent(renamedModel);
        child = child->GetNextElement();
    }

    // Remove the old model
    auto originalModelElement = sdfRoot.ModelByIndex(modelIndex)->Element();
    originalModelElement->RemoveFromParent();

    // Insert the renamed model
    renamedModel->SetParent(sdfRoot.Element());
    sdfRoot.Element()->InsertElement(renamedModel);

    if (sdfRoot.ModelCount() != initialNrOfModels) {
        gymppError << "Failed to rename SDF model" << std::endl;
        return false;
    }

    if (!sdfRoot.ModelNameExists(newModelName)) {
        gymppError << "Failed to insert rename model in SDF root" << std::endl;
        return false;
    }

    return true;
}

bool utils::updateSDFPhysics(sdf::Root& sdfRoot,
                             const double maxStepSize,
                             const double rtf,
                             const double realTimeUpdateRate,
                             const size_t worldIndex)
{
    if (rtf <= 0) {
        gymppError << "Invalid RTF value (" << rtf << ")" << std::endl;
        return false;
    }

    if (maxStepSize <= 0) {
        gymppError << "Invalid physics max step size (" << maxStepSize << ")"
                   << std::endl;
        return false;
    }

    const sdf::World* world = sdfRoot.WorldByIndex(worldIndex);

    if (world->PhysicsCount() != 1) {
        gymppError << "Found more than one physics profile" << std::endl;
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
    max_step_size->AddValue("double", std::to_string(maxStepSize), true);

    sdf::ElementPtr real_time_update_rate =
        physicsElement->GetElement("real_time_update_rate");
    real_time_update_rate->AddValue(
        "double", std::to_string(realTimeUpdateRate), true);

    sdf::ElementPtr real_time_factor =
        physicsElement->GetElement("real_time_factor");
    real_time_factor->AddValue("double", std::to_string(rtf), true);

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

sdf::JointType utils::toSdf(const scenario::base::JointType type)
{
    sdf::JointType sdfType;

    switch (type) {
        case base::JointType::Fixed:
            sdfType = sdf::JointType::FIXED;
            break;
        case base::JointType::Revolute:
            sdfType = sdf::JointType::REVOLUTE;
            break;
        case base::JointType::Prismatic:
            sdfType = sdf::JointType::PRISMATIC;
            break;
        case base::JointType::Ball:
            sdfType = sdf::JointType::BALL;
            break;
        default:
            gymppError << "Joint type not recognized" << std::endl;
            sdfType = sdf::JointType::INVALID;
            break;
    }

    return sdfType;
}

scenario::base::JointType utils::fromSdf(const sdf::JointType sdfType)
{
    base::JointType type;

    switch (sdfType) {
        case sdf::JointType::FIXED:
            type = base::JointType::Fixed;
            break;
        case sdf::JointType::REVOLUTE:
            type = base::JointType::Revolute;
            break;
        case sdf::JointType::PRISMATIC:
            type = base::JointType::Prismatic;
            break;
        case sdf::JointType::BALL:
            type = base::JointType::Ball;
            break;
        default:
            gymppError << "Joint type not recognized" << std::endl;
            type = base::JointType::Invalid;
            break;
    }

    return type;
}

std::pair<ignition::math::Vector3d, ignition::math::Vector3d>
utils::fromModelToBaseVelocity(const ignition::math::Vector3d& linModelVelocity,
                               const ignition::math::Vector3d& angModelVelocity,
                               const ignition::math::Pose3d& M_H_B,
                               const ignition::math::Quaterniond& W_R_B)
{
    ignition::math::Vector3d linBaseVelocity;
    const ignition::math::Vector3d& angBaseVelocity = angModelVelocity;

    // Extract the rotation and the position of the model wrt to the base
    auto B_R_M = M_H_B.Rot().Inverse();
    auto M_o_B = M_H_B.Pos();
    auto B_o_M = -B_R_M * M_o_B;

    // Compute the base linear velocity
    linBaseVelocity = linModelVelocity - angModelVelocity.Cross(W_R_B * B_o_M);

    // Return the mixed velocity of the base
    return {linBaseVelocity, angBaseVelocity};
}

std::pair<ignition::math::Vector3d, ignition::math::Vector3d>
utils::fromBaseToModelVelocity(const ignition::math::Vector3d& linBaseVelocity,
                               const ignition::math::Vector3d& angBaseVelocity,
                               const ignition::math::Pose3d& M_H_B,
                               const ignition::math::Quaterniond& W_R_B)
{
    ignition::math::Vector3d linModelVelocity;
    const ignition::math::Vector3d& angModelVelocity = angBaseVelocity;

    // Extract the rotation and the position of the model wrt to the base
    auto B_R_M = M_H_B.Rot().Inverse();
    auto M_o_B = M_H_B.Pos();

    // Compute the model linear velocity
    linModelVelocity =
        linBaseVelocity - angBaseVelocity.Cross(W_R_B * B_R_M * M_o_B);

    // Return the mixed velocity of the model
    return {linModelVelocity, angModelVelocity};
}
