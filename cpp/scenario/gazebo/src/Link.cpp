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

#include "scenario/gazebo/Link.h"

#include "scenario/gazebo/Log.h"
#include "scenario/gazebo/Model.h"
#include "scenario/gazebo/World.h"
#include "scenario/gazebo/components/ExternalWorldWrenchCmdWithDuration.h"
#include "scenario/gazebo/components/SimulatedTime.h"
#include "scenario/gazebo/exceptions.h"
#include "scenario/gazebo/helpers.h"

#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/components/AngularAcceleration.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/CanonicalLink.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/ContactSensorData.hh>
#include <ignition/gazebo/components/Inertial.hh>
#include <ignition/gazebo/components/LinearAcceleration.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/math/Inertial.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs/contacts.pb.h>

#include <cassert>
#include <chrono>
#include <optional>

using namespace scenario::gazebo;

class Link::Impl
{
public:
    ignition::gazebo::EventManager* eventManager = nullptr;
    ignition::gazebo::EntityComponentManager* ecm = nullptr;

    ignition::gazebo::Link link;
    ignition::gazebo::Entity linkEntity = ignition::gazebo::kNullEntity;

    inline bool isCanonical() const
    {
        return ecm->EntityHasComponentType(
            linkEntity, ignition::gazebo::components::CanonicalLink().TypeId());
    }
};

Link::Link()
    : pImpl{std::make_unique<Impl>()}
{}

uint64_t Link::id() const
{
    // Get the parent world
    WorldPtr parentWorld = utils::getParentWorld(
        pImpl->ecm, pImpl->eventManager, pImpl->linkEntity);
    assert(parentWorld);

    // Get the parent model
    ModelPtr parentModel = utils::getParentModel(
        pImpl->ecm, pImpl->eventManager, pImpl->linkEntity);
    assert(parentModel);

    // Build a unique string identifier of this joint
    std::string scopedLinkName =
        parentWorld->name() + "::" + parentModel->name() + "::" + this->name();

    // Return the hashed string
    return std::hash<std::string>{}(scopedLinkName);
}

Link::~Link() = default;

bool Link::initialize(const ignition::gazebo::Entity linkEntity,
                      ignition::gazebo::EntityComponentManager* ecm,
                      ignition::gazebo::EventManager* eventManager)
{
    if (linkEntity == ignition::gazebo::kNullEntity || !ecm || !eventManager) {
        sError << "Failed to initialize Link" << std::endl;
        return false;
    }

    pImpl->ecm = ecm;
    pImpl->linkEntity = linkEntity;
    pImpl->eventManager = eventManager;

    pImpl->link = ignition::gazebo::Link(linkEntity);

    // Check that the link is valid
    if (!pImpl->link.Valid(*ecm)) {
        sError << "The link entity is not valid" << std::endl;
        return false;
    }

    return true;
}

bool Link::createECMResources()
{
    sMessage << "  [" << pImpl->linkEntity << "] " << this->name() << std::endl;

    using namespace ignition::gazebo;

    // Create link components
    pImpl->ecm->CreateComponent(pImpl->linkEntity, //
                                components::WorldPose());
    pImpl->ecm->CreateComponent(pImpl->linkEntity,
                                components::WorldLinearVelocity());
    pImpl->ecm->CreateComponent(pImpl->linkEntity,
                                components::WorldAngularVelocity());
    pImpl->ecm->CreateComponent(pImpl->linkEntity,
                                components::WorldLinearAcceleration());
    pImpl->ecm->CreateComponent(pImpl->linkEntity,
                                components::WorldAngularAcceleration());
    pImpl->ecm->CreateComponent(pImpl->linkEntity,
                                components::LinearVelocity());
    pImpl->ecm->CreateComponent(pImpl->linkEntity,
                                components::AngularVelocity());
    pImpl->ecm->CreateComponent(pImpl->linkEntity,
                                components::LinearAcceleration());
    pImpl->ecm->CreateComponent(pImpl->linkEntity,
                                components::AngularAcceleration());

    if (!this->enableContactDetection(true)) {
        sError << "Failed to enable contact detection" << std::endl;
        return false;
    }

    return true;
}

std::string Link::name(const bool scoped) const
{
    auto linkNameOptional = pImpl->link.Name(*pImpl->ecm);

    if (!linkNameOptional) {
        throw exceptions::LinkError("Failed to get link name");
    }

    std::string linkName = linkNameOptional.value();

    if (scoped) {
        auto parentModel = utils::getParentModel(
            pImpl->ecm, pImpl->eventManager, pImpl->linkEntity);
        linkName = parentModel->name() + "::" + linkName;
    }

    return linkName;
}

double Link::mass() const
{
    auto inertial = utils::getExistingComponentData< //
        ignition::gazebo::components::Inertial>(pImpl->ecm, pImpl->linkEntity);

    return inertial.MassMatrix().Mass();
}

std::array<double, 3> Link::position() const
{
    ignition::math::Pose3d linkPose;

    if (!pImpl->isCanonical()) {
        auto linkPoseOptional = pImpl->link.WorldPose(*pImpl->ecm);

        if (!linkPoseOptional.has_value()) {
            throw exceptions::LinkError("Failed to get world position", name());
        }

        linkPose = linkPoseOptional.value();
    }
    else {
        auto parentModelOptional = pImpl->link.ParentModel(*pImpl->ecm);
        assert(parentModelOptional.has_value());

        ignition::gazebo::Model parentModel = parentModelOptional.value();
        ignition::gazebo::Entity parentModelEntity = parentModel.Entity();

        auto W_H_M = utils::getExistingComponentData< //
            ignition::gazebo::components::Pose>(pImpl->ecm, parentModelEntity);

        auto M_H_B = utils::getExistingComponentData< //
            ignition::gazebo::components::Pose>(pImpl->ecm, pImpl->linkEntity);

        linkPose = W_H_M * M_H_B;
    }

    return utils::fromIgnitionPose(linkPose).position;
}

std::array<double, 4> Link::orientation() const
{
    ignition::math::Pose3d linkPose;

    if (!pImpl->isCanonical()) {
        auto linkPoseOptional = pImpl->link.WorldPose(*pImpl->ecm);

        if (!linkPoseOptional.has_value()) {
            throw exceptions::LinkError("Failed to get world position", name());
        }

        linkPose = linkPoseOptional.value();
    }
    else {
        auto parentModelOptional = pImpl->link.ParentModel(*pImpl->ecm);
        assert(parentModelOptional.has_value());

        ignition::gazebo::Model parentModel = parentModelOptional.value();
        ignition::gazebo::Entity parentModelEntity = parentModel.Entity();

        auto W_H_M = utils::getExistingComponentData< //
            ignition::gazebo::components::Pose>(pImpl->ecm, parentModelEntity);

        auto M_H_B = utils::getExistingComponentData< //
            ignition::gazebo::components::Pose>(pImpl->ecm, pImpl->linkEntity);

        linkPose = W_H_M * M_H_B;
    }

    return utils::fromIgnitionPose(linkPose).orientation;
}

std::array<double, 3> Link::worldLinearVelocity() const
{
    auto linkLinearVelocity = pImpl->link.WorldLinearVelocity(*pImpl->ecm);

    if (!linkLinearVelocity) {
        throw exceptions::LinkError("Failed to get linear velocity",
                                    this->name());
    }

    return utils::fromIgnitionVector(linkLinearVelocity.value());
}

std::array<double, 3> Link::worldAngularVelocity() const
{
    auto linkAngularVelocity = pImpl->link.WorldAngularVelocity(*pImpl->ecm);

    if (!linkAngularVelocity) {
        throw exceptions::LinkError("Failed to get angular velocity",
                                    this->name());
    }

    return utils::fromIgnitionVector(linkAngularVelocity.value());
}

std::array<double, 3> Link::bodyLinearVelocity() const
{
    auto linkBodyLinVel = utils::getComponentData< //
        ignition::gazebo::components::LinearVelocity>(pImpl->ecm,
                                                      pImpl->linkEntity);

    return utils::fromIgnitionVector(linkBodyLinVel);
}

std::array<double, 3> Link::bodyAngularVelocity() const
{
    auto linkBodyAngVel = utils::getComponentData< //
        ignition::gazebo::components::AngularVelocity>(pImpl->ecm,
                                                       pImpl->linkEntity);

    return utils::fromIgnitionVector(linkBodyAngVel);
}

std::array<double, 3> Link::worldLinearAcceleration() const
{
    auto linkLinearAcceleration =
        pImpl->link.WorldLinearAcceleration(*pImpl->ecm);

    if (!linkLinearAcceleration) {
        throw exceptions::LinkError("Failed to get linear acceleration",
                                    this->name());
    }

    return utils::fromIgnitionVector(linkLinearAcceleration.value());
}

std::array<double, 3> Link::worldAngularAcceleration() const
{
    auto linkWorldAngAcc = utils::getComponentData< //
        ignition::gazebo::components::WorldAngularAcceleration>(
        pImpl->ecm, pImpl->linkEntity);

    return utils::fromIgnitionVector(linkWorldAngAcc);
}

std::array<double, 3> Link::bodyLinearAcceleration() const
{
    auto linkBodyLinAcc = utils::getComponentData< //
        ignition::gazebo::components::LinearAcceleration>(pImpl->ecm,
                                                          pImpl->linkEntity);

    return utils::fromIgnitionVector(linkBodyLinAcc);
}

std::array<double, 3> Link::bodyAngularAcceleration() const
{
    auto linkBodyAngAcc = utils::getComponentData< //
        ignition::gazebo::components::AngularAcceleration>(pImpl->ecm,
                                                           pImpl->linkEntity);

    return utils::fromIgnitionVector(linkBodyAngAcc);
}

bool Link::contactsEnabled() const
{
    // Here we return true only if contacts are enables on all
    // link's collision elements;
    bool enabled = true;

    auto collisionEntities = pImpl->ecm->ChildrenByComponents(
        pImpl->linkEntity,
        ignition::gazebo::components::Collision(),
        ignition::gazebo::components::ParentEntity(pImpl->linkEntity));

    // Create the contact sensor data component that enables the Physics
    // system to extract contact information from the physics engine
    for (const auto collisionEntity : collisionEntities) {
        bool hasContactSensorData = pImpl->ecm->EntityHasComponentType(
            collisionEntity,
            ignition::gazebo::components::ContactSensorData().TypeId());
        enabled = enabled && hasContactSensorData;
    }

    return enabled;
}

bool Link::enableContactDetection(const bool enable)
{
    if (enable && !this->contactsEnabled()) {
        // Get all the collision entities of this link
        auto collisionEntities = pImpl->ecm->ChildrenByComponents(
            pImpl->linkEntity,
            ignition::gazebo::components::Collision(),
            ignition::gazebo::components::ParentEntity(pImpl->linkEntity));

        // Create the contact sensor data component that enables the Physics
        // system to extract contact information from the physics engine
        for (const auto collisionEntity : collisionEntities) {
            pImpl->ecm->CreateComponent(
                collisionEntity,
                ignition::gazebo::components::ContactSensorData());
        }

        return true;
    }

    if (!enable && this->contactsEnabled()) {
        // Get all the collision entities of this link
        auto collisionEntities = pImpl->ecm->ChildrenByComponents(
            pImpl->linkEntity,
            ignition::gazebo::components::Collision(),
            ignition::gazebo::components::ParentEntity(pImpl->linkEntity));

        // Delete the contact sensor data component
        for (const auto collisionEntity : collisionEntities) {
            pImpl->ecm->RemoveComponent<
                ignition::gazebo::components::ContactSensorData>(
                collisionEntity);
        }

        return true;
    }

    return true;
}

bool Link::inContact() const
{
    return this->contactData().empty() ? false : true;
}

std::vector<scenario::base::ContactData> Link::contactData() const
{
    std::vector<ignition::gazebo::Entity> collisionEntities;

    // Get all the collision entities associated with this link
    pImpl->ecm->Each<ignition::gazebo::components::Collision,
                     ignition::gazebo::components::ContactSensorData,
                     ignition::gazebo::components::ParentEntity>(
        [&](const ignition::gazebo::Entity& collisionEntity,
            ignition::gazebo::components::Collision*,
            ignition::gazebo::components::ContactSensorData*,
            ignition::gazebo::components::ParentEntity* parentEntityComponent)
            -> bool {
            // Keep only the collision of this link
            if (parentEntityComponent->Data() != pImpl->linkEntity) {
                return true;
            }

            collisionEntities.push_back(collisionEntity);
            return true;
        });

    if (collisionEntities.empty()) {
        return {};
    }

    std::vector<base::ContactData> linkAllContacts;

    for (const auto collisionEntity : collisionEntities) {

        // Get the contact data for the selected collision entity
        ignition::msgs::Contacts contactSensorData =
            utils::getExistingComponentData< //
                ignition::gazebo::components::ContactSensorData>(
                pImpl->ecm, collisionEntity);

        // Convert the ignition msg
        std::vector<base::ContactData> collisionContacts =
            utils::fromIgnitionContactsMsgs(pImpl->ecm, contactSensorData);

        // Insert the collision contact data to the link's buffer
        linkAllContacts.insert(linkAllContacts.end(),
                               collisionContacts.begin(),
                               collisionContacts.end());
    }

    return linkAllContacts;
}

bool Link::applyWorldForce(const std::array<double, 3>& force,
                           const double duration)
{
    return this->applyWorldWrench(force, {0, 0, 0}, duration);
}

bool Link::applyWorldTorque(const std::array<double, 3>& torque,
                            const double duration)
{
    return this->applyWorldWrench({0, 0, 0}, torque, duration);
}

bool Link::applyWorldWrench(const std::array<double, 3>& force,
                            const std::array<double, 3>& torque,
                            const double duration)
{
    // Adapted from ignition::gazebo::Link::AddWorld{Force,Wrench}
    using namespace std::chrono;
    using namespace ignition;
    using namespace ignition::gazebo;

    auto inertial = utils::getExistingComponentData<components::Inertial>(
        pImpl->ecm, pImpl->linkEntity);

    auto worldPose = utils::getExistingComponentData<components::WorldPose>(
        pImpl->ecm, pImpl->linkEntity);

    auto forceIgnitionMath = utils::toIgnitionVector3(force);

    // We want the force to be applied at the center of mass, but
    // ExternalWorldWrenchCmd applies the force at the link origin so we need to
    // compute the resulting force and torque on the link origin.

    // Compute W_o_I = W_R_L * L_o_I
    auto linkCOMInWorldCoordinates =
        worldPose.Rot().RotateVector(inertial.Pose().Pos());

    // Initialize the torque with the argument
    auto torqueIgnitionMath = utils::toIgnitionVector3(torque);

    // Sum the component given by the projection of the force to the link origin
    torqueIgnitionMath += linkCOMInWorldCoordinates.Cross(forceIgnitionMath);

    // Get the current simulated time
    auto& now = utils::getExistingComponentData<components::SimulatedTime>(
        pImpl->ecm,
        utils::getFirstParentEntityWithComponent<components::SimulatedTime>(
            pImpl->ecm, pImpl->linkEntity));

    // Create a new wrench with duration
    utils::WrenchWithDuration wrench(
        forceIgnitionMath,
        torqueIgnitionMath,
        utils::doubleToSteadyClockDuration(duration),
        now);

    utils::LinkWrenchCmd& linkWrenchCmd =
        utils::getComponentData<components::ExternalWorldWrenchCmdWithDuration>(
            pImpl->ecm, pImpl->linkEntity);

    linkWrenchCmd.addWorldWrench(wrench);
    return true;
}
