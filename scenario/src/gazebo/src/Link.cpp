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
#include "scenario/gazebo/utils.h"

#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/components/AngularAcceleration.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/CanonicalLink.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/ContactSensorData.hh>
#include <ignition/gazebo/components/Inertial.hh>
#include <ignition/gazebo/components/LinearAcceleration.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Model.hh>
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
    ignition::gazebo::Link link;

    static ignition::math::Pose3d GetWorldPose(const Link& link,
                                               const Link::Impl& impl)
    {
        if (!impl.link.IsCanonical(*link.ecm())) {
            const auto& linkPoseOptional = impl.link.WorldPose(*link.ecm());

            if (!linkPoseOptional.has_value()) {
                throw exceptions::LinkError("Failed to get world position",
                                            link.name());
            }

            return linkPoseOptional.value();
        }
        else {
            const auto& parentModelOptional =
                impl.link.ParentModel(*link.ecm());
            assert(parentModelOptional.has_value());

            const ignition::gazebo::Model& parentModel =
                parentModelOptional.value();
            const ignition::gazebo::Entity parentModelEntity =
                parentModel.Entity();

            auto W_H_M = utils::getExistingComponentData<
                ignition::gazebo::components::Pose>(link.ecm(),
                                                    parentModelEntity);

            auto M_H_B = utils::getExistingComponentData<
                ignition::gazebo::components::Pose>(link.ecm(), link.entity());

            return W_H_M * M_H_B;
        }
    }
};

Link::Link()
    : pImpl{std::make_unique<Impl>()}
{}

Link::~Link() = default;

uint64_t Link::id() const
{
    // Get the parent world
    const core::WorldPtr parentWorld = utils::getParentWorld(*this);
    assert(parentWorld);

    // Build a unique string identifier of this joint
    const std::string scopedLinkName =
        parentWorld->name() + "::" + this->name(/*scoped=*/true);

    // Return the hashed string
    return std::hash<std::string>{}(scopedLinkName);
}

bool Link::initialize(const ignition::gazebo::Entity linkEntity,
                      ignition::gazebo::EntityComponentManager* ecm,
                      ignition::gazebo::EventManager* eventManager)
{
    if (linkEntity == ignition::gazebo::kNullEntity || !ecm || !eventManager) {
        sError << "Failed to initialize Link" << std::endl;
        return false;
    }

    m_ecm = ecm;
    m_entity = linkEntity;
    m_eventManager = eventManager;

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
    sMessage << "  [" << m_entity << "] " << this->name() << std::endl;

    using namespace ignition::gazebo;

    // Create link components
    m_ecm->CreateComponent(m_entity, //
                           components::WorldPose());
    m_ecm->CreateComponent(m_entity, components::WorldLinearVelocity());
    m_ecm->CreateComponent(m_entity, components::WorldAngularVelocity());
    m_ecm->CreateComponent(m_entity, components::WorldLinearAcceleration());
    m_ecm->CreateComponent(m_entity, components::WorldAngularAcceleration());
    m_ecm->CreateComponent(m_entity, components::LinearVelocity());
    m_ecm->CreateComponent(m_entity, components::AngularVelocity());
    m_ecm->CreateComponent(m_entity, components::LinearAcceleration());
    m_ecm->CreateComponent(m_entity, components::AngularAcceleration());

    if (!this->enableContactDetection(false)) {
        sError << "Failed to initialize contact detection" << std::endl;
        return false;
    }

    return true;
}

bool Link::insertLinkPlugin(const std::string& libName,
                            const std::string& className,
                            const std::string& context)
{
    return utils::insertPluginToGazeboEntity(
        *this, libName, className, context);
}

bool Link::valid() const
{
    return this->validEntity() && pImpl->link.Valid(*m_ecm);
}

std::string Link::name(const bool scoped) const
{
    const auto& linkNameOptional = pImpl->link.Name(*m_ecm);

    if (!linkNameOptional) {
        throw exceptions::LinkError("Failed to get link name");
    }

    std::string linkName = linkNameOptional.value();

    if (scoped) {
        linkName = utils::getParentModel(*this)->name() + "::" + linkName;
    }

    return linkName;
}

double Link::mass() const
{
    const auto& inertial = utils::getExistingComponentData< //
        ignition::gazebo::components::Inertial>(m_ecm, m_entity);

    return inertial.MassMatrix().Mass();
}

std::array<double, 3> Link::position() const
{
    const ignition::math::Pose3d& linkPose = Impl::GetWorldPose(*this, *pImpl);
    return utils::fromIgnitionPose(linkPose).position;
}

std::array<double, 4> Link::orientation() const
{
    const ignition::math::Pose3d& linkPose = Impl::GetWorldPose(*this, *pImpl);
    return utils::fromIgnitionPose(linkPose).orientation;
}

std::array<double, 3> Link::worldLinearVelocity() const
{
    const auto& linkLinearVelocity = pImpl->link.WorldLinearVelocity(*m_ecm);

    if (!linkLinearVelocity) {
        throw exceptions::LinkError("Failed to get linear velocity",
                                    this->name());
    }

    return utils::fromIgnitionVector(linkLinearVelocity.value());
}

std::array<double, 3> Link::worldAngularVelocity() const
{
    const auto& linkAngularVelocity = pImpl->link.WorldAngularVelocity(*m_ecm);

    if (!linkAngularVelocity) {
        throw exceptions::LinkError("Failed to get angular velocity",
                                    this->name());
    }

    return utils::fromIgnitionVector(linkAngularVelocity.value());
}

std::array<double, 3> Link::bodyLinearVelocity() const
{
    const auto& linkBodyLinVel = utils::getComponentData< //
        ignition::gazebo::components::LinearVelocity>(m_ecm, m_entity);

    return utils::fromIgnitionVector(linkBodyLinVel);
}

std::array<double, 3> Link::bodyAngularVelocity() const
{
    const auto& linkBodyAngVel = utils::getComponentData< //
        ignition::gazebo::components::AngularVelocity>(m_ecm, m_entity);

    return utils::fromIgnitionVector(linkBodyAngVel);
}

std::array<double, 3> Link::worldLinearAcceleration() const
{
    const auto& linkLinearAcceleration =
        pImpl->link.WorldLinearAcceleration(*m_ecm);

    if (!linkLinearAcceleration) {
        throw exceptions::LinkError("Failed to get linear acceleration",
                                    this->name());
    }

    return utils::fromIgnitionVector(linkLinearAcceleration.value());
}

std::array<double, 3> Link::worldAngularAcceleration() const
{
    const auto& linkWorldAngAcc = utils::getComponentData<
        ignition::gazebo::components::WorldAngularAcceleration>(m_ecm,
                                                                m_entity);

    return utils::fromIgnitionVector(linkWorldAngAcc);
}

std::array<double, 3> Link::bodyLinearAcceleration() const
{
    const auto& linkBodyLinAcc = utils::getComponentData<
        ignition::gazebo::components::LinearAcceleration>(m_ecm, m_entity);

    return utils::fromIgnitionVector(linkBodyLinAcc);
}

std::array<double, 3> Link::bodyAngularAcceleration() const
{
    const auto& linkBodyAngAcc = utils::getComponentData<
        ignition::gazebo::components::AngularAcceleration>(m_ecm, m_entity);

    return utils::fromIgnitionVector(linkBodyAngAcc);
}

bool Link::contactsEnabled() const
{
    const auto& collisionEntities = m_ecm->ChildrenByComponents(
        m_entity,
        ignition::gazebo::components::Collision(),
        ignition::gazebo::components::ParentEntity(m_entity));

    // If the link has no collision elements, we return true regardless.
    // To prevent surprises, e.g. users expecting that calling Link::inContact
    // for such links would return true, we print a debug message.
    if (collisionEntities.empty()) {
        sDebug << "The link '" << this->name() << "' has no collision elements "
               << "and contacts cannot be detected" << std::endl;
        return true;
    }

    // Iterate through all link's collisions
    for (const auto collisionEntity : collisionEntities) {
        const bool hasContactSensorData = m_ecm->EntityHasComponentType(
            collisionEntity,
            ignition::gazebo::components::ContactSensorData::typeId);

        // Return false if a collision does not have the contact data component
        if (!hasContactSensorData) {
            return false;
        }
    }

    // We return true only if contacts are enables on all collision entities
    return true;
}

bool Link::enableContactDetection(const bool enable)
{
    if (enable && !this->contactsEnabled()) {
        // Get all the collision entities of this link
        const auto& collisionEntities = m_ecm->ChildrenByComponents(
            m_entity,
            ignition::gazebo::components::Collision(),
            ignition::gazebo::components::ParentEntity(m_entity));

        // Create the contact sensor data component that enables the Physics
        // system to extract contact information from the physics engine
        for (const auto collisionEntity : collisionEntities) {
            m_ecm->CreateComponent(
                collisionEntity,
                ignition::gazebo::components::ContactSensorData());
        }

        return true;
    }

    if (!enable && this->contactsEnabled()) {
        // Get all the collision entities of this link
        const auto& collisionEntities = m_ecm->ChildrenByComponents(
            m_entity,
            ignition::gazebo::components::Collision(),
            ignition::gazebo::components::ParentEntity(m_entity));

        // Links with no collision elements already print a sDebug in the
        // contactsEnabled method, and not further action is needed
        if (collisionEntities.empty()) {
            return true;
        }

        // Delete the contact sensor data component
        for (const auto collisionEntity : collisionEntities) {
            m_ecm->RemoveComponent<
                ignition::gazebo::components::ContactSensorData>(
                collisionEntity);
        }

        if (this->contactsEnabled()) {
            sError << "Failed to disable contact detection" << std::endl;
            return false;
        }

        return true;
    }

    return true;
}

bool Link::inContact() const
{
    return this->contacts().empty() ? false : true;
}

std::vector<scenario::core::Contact> Link::contacts() const
{
    // Get the collisions of this link
    const auto& collisionEntities = m_ecm->EntitiesByComponents(
        ignition::gazebo::components::ParentEntity(m_entity),
        ignition::gazebo::components::Collision());

    // Return early if the link has no collision elements
    if (collisionEntities.empty()) {
        return {};
    }

    using BodyNameA = std::string;
    using BodyNameB = std::string;
    using CollisionsInContact = std::pair<BodyNameA, BodyNameB>;
    auto contactsMap = std::map<CollisionsInContact, core::Contact>();

    for (const auto collisionEntity : collisionEntities) {

        // Skip collisions entities without contact sensor
        if (!m_ecm->EntityHasComponentType(
                collisionEntity,
                ignition::gazebo::components::ContactSensorData::typeId)) {
            continue;
        }

        // Get the contact data for the selected collision entity
        const ignition::msgs::Contacts& contactSensorData =
            utils::getExistingComponentData<
                ignition::gazebo::components::ContactSensorData>(
                m_ecm, collisionEntity);

        // Convert the ignition msg
        const std::vector<core::Contact>& collisionContacts =
            utils::fromIgnitionContactsMsgs(m_ecm, contactSensorData);

        for (const auto& contact : collisionContacts) {
            assert(!contact.bodyA.empty());
            assert(!contact.bodyB.empty());

            // Create the key that collects the entry containing the
            // Contact object of the pair of bodies
            const auto key = std::make_pair(contact.bodyA, contact.bodyB);

            if (contactsMap.find(key) != contactsMap.end()) {
                // Get the existing contact object
                auto& thisContact = contactsMap.at(key);

                // Insert the new points
                thisContact.points.insert(thisContact.points.end(),
                                          contact.points.begin(),
                                          contact.points.end());
            }
            else {
                // Create a new Contact
                contactsMap[key] = contact;
            }
        }
    }

    // Copy data from the map to the output vector
    // TODO: any trick to move values from the map to the vector?
    std::vector<core::Contact> allContacts;
    allContacts.reserve(contactsMap.size());

    for (const auto& [_, contact] : contactsMap) {
        allContacts.push_back(contact);
    }

    return allContacts;
}

std::array<double, 6> Link::contactWrench() const
{
    auto totalForce = ignition::math::Vector3d::Zero;
    auto totalTorque = ignition::math::Vector3d::Zero;

    const auto& contacts = this->contacts();

    for (const auto& contact : contacts) {
        // Each contact wrench is expressed with respect to the contact point
        // and with the orientation of the world frame. We need to translate it
        // to the link frame.

        for (const auto& contactPoint : contact.points) {
            // The contact points extracted from the physics do not have torque
            constexpr std::array<double, 3> zero = {0, 0, 0};
            assert(contactPoint.torque == zero);

            // Link position
            const auto& o_L = utils::toIgnitionVector3(this->position());

            // Contact position
            const auto& o_P = utils::toIgnitionVector3(contactPoint.position);

            // Relative position
            const auto L_o_P = o_P - o_L;

            // The contact force and the total link force are both expressed
            // with the orientation of the world frame. This simplifies the
            // conversion since we have to take into account only the
            // displacement.
            const auto& force = utils::toIgnitionVector3(contactPoint.force);

            // The force does not have to be changed
            totalForce += force;

            // There is however a torque that balances out the resulting moment
            totalTorque += L_o_P.Cross(force);
        }
    }

    return {totalForce[0],
            totalForce[1],
            totalForce[2],
            totalTorque[0],
            totalTorque[1],
            totalTorque[2]};
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

    // Initialize the force and the torque with the input data
    const auto& forceIgnitionMath = utils::toIgnitionVector3(force);
    const auto& torqueIgnitionMath = utils::toIgnitionVector3(torque);

    const auto entityWithSimTime = utils::getFirstParentEntityWithComponent<
        ignition::gazebo::components::SimulatedTime>(m_ecm, m_entity);
    assert(entityWithSimTime != ignition::gazebo::kNullEntity);

    // Get the current simulated time
    const auto& now = utils::getExistingComponentData<
        ignition::gazebo::components::SimulatedTime>(m_ecm, entityWithSimTime);

    // Create a new wrench with duration
    const utils::WrenchWithDuration wrench(
        forceIgnitionMath,
        torqueIgnitionMath,
        utils::doubleToSteadyClockDuration(duration),
        now);

    utils::LinkWrenchCmd& linkWrenchCmd = utils::getComponentData<
        ignition::gazebo::components::ExternalWorldWrenchCmdWithDuration>(
        m_ecm, m_entity);

    linkWrenchCmd.addWorldWrench(wrench);
    return true;
}

bool Link::applyWorldWrenchToCoM(const std::array<double, 3>& force,
                                 const std::array<double, 3>& torque,
                                 const double duration)
{
    const ignition::math::Pose3d& worldPose = Impl::GetWorldPose(*this, *pImpl);

    // Get the data of the inertial frame
    auto inertial = utils::getExistingComponentData< //
        ignition::gazebo::components::Inertial>(m_ecm, m_entity);

    // We want the force to be applied at the center of mass, but
    // ExternalWorldWrenchCmd applies the force at the link origin so we need to
    // compute the resulting force and torque on the link origin.

    // Compute W_o_I = W_R_L * L_o_I
    auto linkCOMInWorldCoordinates =
        worldPose.Rot().RotateVector(inertial.Pose().Pos());

    // Initialize the force and the torque with the input data
    const auto forceIgnitionMath = utils::toIgnitionVector3(force);
    auto torqueIgnitionMath = utils::toIgnitionVector3(torque);

    // Sum the component given by the projection of the force to the link origin
    torqueIgnitionMath += linkCOMInWorldCoordinates.Cross(forceIgnitionMath);

    return this->applyWorldWrench(utils::fromIgnitionVector(forceIgnitionMath),
                                  utils::fromIgnitionVector(torqueIgnitionMath),
                                  duration);
}
