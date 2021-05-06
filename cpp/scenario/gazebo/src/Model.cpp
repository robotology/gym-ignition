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

#include "scenario/gazebo/Model.h"
#include "scenario/gazebo/Joint.h"
#include "scenario/gazebo/Link.h"
#include "scenario/gazebo/Log.h"
#include "scenario/gazebo/World.h"
#include "scenario/gazebo/components/BasePoseTarget.h"
#include "scenario/gazebo/components/BaseWorldAccelerationTarget.h"
#include "scenario/gazebo/components/BaseWorldVelocityTarget.h"
#include "scenario/gazebo/components/JointControllerPeriod.h"
#include "scenario/gazebo/components/Timestamp.h"
#include "scenario/gazebo/exceptions.h"
#include "scenario/gazebo/helpers.h"
#include "scenario/gazebo/utils.h"

#include <ignition/common/Event.hh>
#include <ignition/gazebo/Events.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/AngularVelocityCmd.hh>
#include <ignition/gazebo/components/CanonicalLink.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
#include <ignition/gazebo/components/SelfCollide.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/Element.hh>
#include <sdf/Root.hh>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <functional>
#include <tuple>
#include <unordered_map>

using namespace scenario::gazebo;

class Model::Impl
{
public:
    ignition::gazebo::Model model;

    using LinkName = std::string;
    using JointName = std::string;

    std::unordered_map<LinkName, core::LinkPtr> links;
    std::unordered_map<JointName, core::JointPtr> joints;

    struct
    {
        std::vector<std::string> linksInContact;
        std::optional<std::vector<std::string>> linkNames;
        std::optional<std::vector<std::string>> scopedLinkNames;
        std::optional<std::vector<std::string>> jointNames;
        std::optional<std::vector<std::string>> scopedJointNames;
    } buffers;

    static std::vector<double> getJointDataSerialized(
        const Model* model,
        const std::vector<std::string>& jointNames,
        std::function<double(core::JointPtr, const size_t)> getJointData);

    static bool setJointDataSerialized(
        Model* model,
        const std::vector<double>& data,
        const std::vector<std::string>& jointNames,
        std::function<bool(core::JointPtr, const double, const size_t)>
            setDataToDOF);
};

Model::Model()
    : pImpl{std::make_unique<Impl>()}
{}

Model::~Model() = default;

uint64_t Model::id() const
{
    // Get the parent world
    const core::WorldPtr parentWorld = utils::getParentWorld(*this);
    assert(parentWorld);

    // Build a unique string identifier of this model
    const std::string scopedModelName =
        parentWorld->name() + "::" + this->name();

    // Return the hashed string
    return std::hash<std::string>{}(scopedModelName);
}

bool Model::initialize(const ignition::gazebo::Entity modelEntity,
                       ignition::gazebo::EntityComponentManager* ecm,
                       ignition::gazebo::EventManager* eventManager)
{
    if (modelEntity == ignition::gazebo::kNullEntity || !ecm || !eventManager) {
        return false;
    }

    m_ecm = ecm;
    m_entity = modelEntity;
    m_eventManager = eventManager;

    // Create the model
    pImpl->model = ignition::gazebo::Model(modelEntity);

    // Check that the model is valid
    if (!pImpl->model.Valid(*ecm)) {
        sError << "The model entity is not valid" << std::endl;
        return false;
    }

    return true;
}

bool Model::createECMResources()
{
    sMessage << "Model: [" << m_entity << "] " << this->name() << std::endl;

    // When the model is inserted, store the time of creation
    if (!m_ecm->EntityHasComponentType(
            m_entity, ignition::gazebo::components::Timestamp::typeId)) {
        const auto& parentWorld = utils::getParentWorld(*this);
        utils::setComponentData<ignition::gazebo::components::Timestamp>(
            m_ecm,
            m_entity,
            utils::doubleToSteadyClockDuration(parentWorld->time()));
    }

    // Create required link resources
    sMessage << "Links:" << std::endl;
    for (const auto& link : this->links()) {
        if (!std::static_pointer_cast<Link>(link)->createECMResources()) {
            sError << "Failed to initialize ECM link resources" << std::endl;
            return false;
        }
    }

    // Create required model resources
    sMessage << "Joints:" << std::endl;
    for (const auto& joint : this->joints()) {
        if (!std::static_pointer_cast<Joint>(joint)->createECMResources()) {
            sError << "Failed to initialize ECM joint resources" << std::endl;
            return false;
        }
    }

    if (!this->enableSelfCollisions(false)) {
        sError << "Failed to initialize disabled self collisions" << std::endl;
        return false;
    }

    // Initialize the Joint Controller period as maximum duration.
    // In this way controllers are never updated unless a new period is
    // configured.
    m_ecm->CreateComponent(m_entity,
                           ignition::gazebo::components::JointControllerPeriod(
                               std::chrono::steady_clock::duration().max()));

    return true;
}

bool Model::insertModelPlugin(const std::string& libName,
                              const std::string& className,
                              const std::string& context)
{
    return utils::insertPluginToGazeboEntity(
        *this, libName, className, context);
}

bool Model::resetJointPositions(const std::vector<double>& positions,
                                const std::vector<std::string>& jointNames)
{
    auto lambda = [](core::JointPtr joint,
                     const double position,
                     const size_t dof) -> bool {
        return std::static_pointer_cast<Joint>(joint)->resetPosition(position,
                                                                     dof);
    };

    return Impl::setJointDataSerialized(this, positions, jointNames, lambda);
}

bool Model::resetJointVelocities(const std::vector<double>& velocities,
                                 const std::vector<std::string>& jointNames)
{
    auto lambda = [](core::JointPtr joint,
                     const double velocity,
                     const size_t dof) -> bool {
        return std::static_pointer_cast<Joint>(joint)->resetVelocity(velocity,
                                                                     dof);
    };

    return Impl::setJointDataSerialized(this, velocities, jointNames, lambda);
}

bool Model::resetBasePose(const std::array<double, 3>& position,
                          const std::array<double, 4>& orientation)
{
    // Construct the desired transform between world and base
    const core::Pose pose(position, orientation);
    ignition::math::Pose3d world_H_base = utils::toIgnitionPose(pose);

    // Get the entity of the canonical link
    const auto canonicalLinkEntity = m_ecm->EntityByComponents(
        ignition::gazebo::components::Link(),
        ignition::gazebo::components::CanonicalLink(),
        ignition::gazebo::components::Name(this->baseFrame()),
        ignition::gazebo::components::ParentEntity(m_entity));

    if (canonicalLinkEntity == ignition::gazebo::kNullEntity) {
        sError << "Failed to get entity of canonical link" << std::endl;
        return false;
    }

    // Get the Pose component of the canonical link.
    // This is the fixed transformation between the model and the base.
    auto& model_H_base = utils::getExistingComponentData< //
        ignition::gazebo::components::Pose>(m_ecm, canonicalLinkEntity);

    // Compute the robot pose that corresponds to the desired base pose
    const ignition::math::Pose3d& world_H_model =
        world_H_base * model_H_base.Inverse();

    // Store the new pose
    utils::setComponentData<ignition::gazebo::components::WorldPoseCmd>(
        m_ecm, m_entity, world_H_model);

    return true;
}

bool Model::resetBasePosition(const std::array<double, 3>& position)
{
    return this->resetBasePose(position, this->baseOrientation());
}

bool Model::resetBaseOrientation(const std::array<double, 4>& orientation)
{
    return this->resetBasePose(this->basePosition(), orientation);
}

bool Model::resetBaseWorldLinearVelocity(const std::array<double, 3>& linear)
{
    // Note: there could be a rigid transformation between the base frame and
    //       the canonical frame. The Physics system processes velocity commands
    //       in the canonical frame, but this method receives velocity commands
    //       of in the base frame (all expressed in world coordinates).
    //       Therefore, we need to compute the base linear velocity from the
    //       base frame to the canonical frame.

    // Get the entity of the canonical (base) link
    const auto canonicalLinkEntity = m_ecm->EntityByComponents(
        ignition::gazebo::components::Link(),
        ignition::gazebo::components::CanonicalLink(),
        ignition::gazebo::components::Name(this->baseFrame()),
        ignition::gazebo::components::ParentEntity(m_entity));

    // Get the Pose component of the canonical link.
    // This is the fixed transformation between the model and the base.
    const auto& M_H_B = utils::getExistingComponentData< //
        ignition::gazebo::components::Pose>(m_ecm, canonicalLinkEntity);

    // Get the rotation between base link and world
    const auto& W_R_B = utils::toIgnitionQuaternion(
        this->getLink(this->baseFrame())->orientation());

    // Compute the linear part of the base link mixed velocity
    const ignition::math::Vector3d baseLinearWorldVelocity =
        utils::fromModelToBaseLinearVelocity(
            utils::toIgnitionVector3(linear),
            utils::toIgnitionVector3(this->baseWorldAngularVelocity()),
            M_H_B,
            W_R_B);

    // Store the new velocity
    utils::setComponentData<ignition::gazebo::components::LinearVelocityCmd>(
        m_ecm, m_entity, baseLinearWorldVelocity);

    return true;
}

bool Model::resetBaseWorldAngularVelocity(const std::array<double, 3>& angular)
{
    // Note: the angular part of the velocity does not change between the base
    //       link and the canonical link (as the linear part).
    //       In fact, the angular velocity is invariant if there's a rigid
    //       transformation between the two frames, like in this case.
    utils::setComponentData<ignition::gazebo::components::AngularVelocityCmd>(
        m_ecm, m_entity, utils::toIgnitionVector3(angular));

    return true;
}

bool Model::resetBaseWorldVelocity(const std::array<double, 3>& linear,
                                   const std::array<double, 3>& angular)
{
    return this->resetBaseWorldLinearVelocity(linear)
           && this->resetBaseWorldAngularVelocity(angular);
}

bool Model::valid() const
{
    return this->validEntity() && pImpl->model.Valid(*m_ecm);
}

size_t Model::dofs(const std::vector<std::string>& jointNames) const
{
    const std::vector<std::string>& jointSerialization =
        jointNames.empty() ? this->jointNames() : jointNames;

    size_t dofs = 0;

    for (const auto& jointName : jointSerialization) {
        dofs += this->getJoint(jointName)->dofs();
    }

    return dofs;
}

std::string Model::name() const
{
    return pImpl->model.Name(*m_ecm);
}

size_t Model::nrOfLinks() const
{
    return this->linkNames().size();
}

size_t Model::nrOfJoints() const
{
    return this->jointNames().size();
}

double Model::totalMass(const std::vector<std::string>& linkNames) const
{
    const std::vector<std::string>& linkSerialization =
        linkNames.empty() ? this->linkNames() : linkNames;

    double mass = 0.0;

    for (const auto& link : this->links(linkSerialization)) {
        mass += link->mass();
    }

    return mass;
}

scenario::core::LinkPtr Model::getLink(const std::string& linkName) const
{
    if (pImpl->links.find(linkName) != pImpl->links.end()) {
        assert(pImpl->links.at(linkName));
        return pImpl->links.at(linkName);
    }

    const auto linkEntity = pImpl->model.LinkByName(*m_ecm, linkName);

    if (linkEntity == ignition::gazebo::kNullEntity) {
        throw exceptions::LinkNotFound(linkName);
    }

    // Create the link
    auto link = std::make_shared<scenario::gazebo::Link>();

    if (!link->initialize(linkEntity, m_ecm, m_eventManager)) {
        throw exceptions::LinkError("Failed to initialize link", linkName);
    }

    // Cache the link instance
    pImpl->links[linkName] = link;

    return link;
}

scenario::core::JointPtr Model::getJoint(const std::string& jointName) const
{
    if (pImpl->joints.find(jointName) != pImpl->joints.end()) {
        assert(pImpl->joints.at(jointName));
        return pImpl->joints.at(jointName);
    }

    const auto jointEntity = pImpl->model.JointByName(*m_ecm, jointName);

    if (jointEntity == ignition::gazebo::kNullEntity) {
        throw exceptions::JointNotFound(jointName);
    }

    // Create the joint
    auto joint = std::make_shared<scenario::gazebo::Joint>();

    if (!joint->initialize(jointEntity, m_ecm, m_eventManager)) {
        throw exceptions::JointError("Failed to initialize joint", jointName);
    }

    // Cache the joint instance
    pImpl->joints[jointName] = joint;

    return joint;
}

std::vector<std::string> Model::linkNames(const bool scoped) const
{
    if (!scoped && pImpl->buffers.linkNames.has_value()) {
        return pImpl->buffers.linkNames.value();
    }

    if (scoped && pImpl->buffers.scopedLinkNames.has_value()) {
        return pImpl->buffers.scopedLinkNames.value();
    }

    std::vector<std::string> linkNames;

    m_ecm->Each<ignition::gazebo::components::Name,
                ignition::gazebo::components::Link,
                ignition::gazebo::components::ParentEntity>(
        [&](const ignition::gazebo::Entity& /*entity*/,
            ignition::gazebo::components::Name* nameComponent,
            ignition::gazebo::components::Link* /*linkComponent*/,
            ignition::gazebo::components::ParentEntity* parentEntityComponent)
            -> bool {
            assert(nameComponent);
            assert(parentEntityComponent);

            // Discard links not belonging to this model
            if (parentEntityComponent->Data() != m_entity) {
                return true;
            }

            std::string prefix = "";
            if (scoped) {
                prefix = this->name() + "::";
            }

            // Append the link name
            linkNames.push_back(prefix + nameComponent->Data());
            return true;
        });

    if (!scoped) {
        pImpl->buffers.linkNames = std::move(linkNames);
        return pImpl->buffers.linkNames.value();
    }
    else {
        pImpl->buffers.scopedLinkNames = std::move(linkNames);
        return pImpl->buffers.scopedLinkNames.value();
    }
}

std::vector<std::string> Model::jointNames(const bool scoped) const
{
    if (!scoped && pImpl->buffers.jointNames.has_value()) {
        return pImpl->buffers.jointNames.value();
    }

    if (scoped && pImpl->buffers.scopedLinkNames.has_value()) {
        return pImpl->buffers.scopedLinkNames.value();
    }

    std::vector<std::string> jointNames;

    m_ecm->Each<ignition::gazebo::components::Name,
                ignition::gazebo::components::Joint,
                ignition::gazebo::components::ParentEntity>(
        [&](const ignition::gazebo::Entity& /*jointEntity*/,
            ignition::gazebo::components::Name* nameComponent,
            ignition::gazebo::components::Joint* /*jointComponent*/,
            ignition::gazebo::components::ParentEntity* parentEntityComponent)
            -> bool {
            assert(nameComponent);
            assert(parentEntityComponent);

            // Discard joints not belonging to this model
            if (parentEntityComponent->Data() != m_entity) {
                return true;
            }

            // Discard joints with no DoFs
            auto joint = this->getJoint(nameComponent->Data());
            if (joint->dofs() == 0) {
                return true;
            }

            std::string prefix = "";
            if (scoped) {
                prefix = this->name() + "::";
            }

            // Append the joint name
            jointNames.push_back(prefix + nameComponent->Data());
            return true;
        });

    if (!scoped) {
        pImpl->buffers.jointNames = std::move(jointNames);
        return pImpl->buffers.jointNames.value();
    }
    else {
        pImpl->buffers.scopedJointNames = std::move(jointNames);
        return pImpl->buffers.scopedJointNames.value();
    }
}

double Model::controllerPeriod() const
{
    const auto& duration = utils::getExistingComponentData< //
        ignition::gazebo::components::JointControllerPeriod>(m_ecm, m_entity);

    return utils::steadyClockDurationToDouble(duration);
}

bool Model::setControllerPeriod(const double period)
{
    if (period <= 0) {
        sError << "The controller period must be greater than zero"
               << std::endl;
        return false;
    }

    // Store the new period in the ECM
    utils::setExistingComponentData<
        ignition::gazebo::components::JointControllerPeriod>(
        m_ecm, m_entity, utils::doubleToSteadyClockDuration(period));
    return true;
}

bool Model::enableHistoryOfAppliedJointForces(
    const bool enable,
    const size_t maxHistorySizePerJoint,
    const std::vector<std::string>& jointNames)
{
    const std::vector<std::string>& jointSerialization =
        jointNames.empty() ? this->jointNames() : jointNames;

    bool ok = true;

    for (const auto& joint : this->joints(jointSerialization)) {
        ok = ok
             && joint->enableHistoryOfAppliedJointForces(
                 enable, maxHistorySizePerJoint);
    }

    return ok;
}

bool Model::historyOfAppliedJointForcesEnabled(
    const std::vector<std::string>& jointNames) const
{
    const std::vector<std::string>& jointSerialization =
        jointNames.empty() ? this->jointNames() : jointNames;

    bool enabled = true;

    for (const auto& joint : this->joints(jointSerialization)) {
        enabled = enabled && joint->historyOfAppliedJointForcesEnabled();
    }

    return enabled;
}

std::vector<double> Model::historyOfAppliedJointForces(
    const std::vector<std::string>& jointNames) const
{
    const std::vector<std::string>& jointSerialization =
        jointNames.empty() ? this->jointNames() : jointNames;

    std::vector<double> history;
    std::vector<double> allAppliedJointForces;

    for (const auto& joint : this->joints(jointSerialization)) {
        history = joint->historyOfAppliedJointForces();
        std::move(history.begin(),
                  history.end(),
                  std::back_inserter(allAppliedJointForces));

        if (joint->name() == jointSerialization.front()) {
            history.reserve(jointSerialization.size() * history.size());
        }
    }

    // Given:
    // * <j_1>: vector 1xH of torques of joint 1
    // * <tau_t>: vector 1xn of torques of the considered joints at a given t
    //
    // We want to convert the allAppliedJointForces:
    // * From: <j_1><j_2>...<j_n>
    // * To:   <tau_t-H>...<tau_t-2><tau_t-1><tau_t>
    //
    // In other words, we want that the torques applied at the last step are
    // piled up in the end of the returned vector.
    utils::rowMajorToColumnMajor(
        allAppliedJointForces, jointSerialization.size(), history.size());

    return allAppliedJointForces;
}

bool Model::contactsEnabled() const
{
    for (auto& link : this->links()) {
        // Note: links with no collision elements return true even though no
        //       contacts can be detected.
        if (!link->contactsEnabled()) {
            return false;
        }
    }

    // Return true only if all links have enabled contact detection
    return true;
}

bool Model::enableContacts(const bool enable)
{
    bool ok = true;

    for (auto& link : this->links()) {
        // Note: links with no collision elements return true even though no
        //       contacts can be detected.
        ok = ok && link->enableContactDetection(enable);
    }

    return ok;
}

bool Model::selfCollisionsEnabled() const
{
    const bool selfCollisionsEnabled = utils::getExistingComponentData<
        ignition::gazebo::components::SelfCollide>(m_ecm, m_entity);

    return selfCollisionsEnabled;
}

bool Model::enableSelfCollisions(const bool enable)
{
    if (!utils::parentModelJustCreated(*this)) {
        sError << "The model has been already processed and its "
               << "parameters cannot be modified" << std::endl;
        return false;
    }

    // Enable contact detection first
    if (enable && !this->enableContacts(true)) {
        sError << "Failed to enable contact detection" << std::endl;
        return false;
    }

    utils::setExistingComponentData<ignition::gazebo::components::SelfCollide>(
        m_ecm, m_entity, enable);

    return true;
}

std::vector<std::string> Model::linksInContact() const
{
    pImpl->buffers.linksInContact.clear();

    for (const auto& link : this->links()) {
        if (link->inContact()) {
            pImpl->buffers.linksInContact.push_back(link->name());
        }
    }

    return pImpl->buffers.linksInContact;
}

std::vector<scenario::core::Contact>
Model::contacts(const std::vector<std::string>& linkNames) const
{
    const std::vector<std::string>& linkSerialization =
        linkNames.empty() ? this->linkNames() : linkNames;

    std::vector<scenario::core::Contact> allContacts;

    for (const auto& linkName : linkSerialization) {
        const auto& contacts = this->getLink(linkName)->contacts();
        std::move(contacts.begin(), //
                  contacts.end(),
                  std::back_inserter(allContacts));
    }

    return allContacts;
}

std::vector<double>
Model::jointPositions(const std::vector<std::string>& jointNames) const
{
    auto lambda = [](core::JointPtr joint, const size_t dof) -> double {
        return joint->position(dof);
    };

    return Impl::getJointDataSerialized(this, jointNames, lambda);
}

std::vector<double>
Model::jointVelocities(const std::vector<std::string>& jointNames) const
{
    auto lambda = [](core::JointPtr joint, const size_t dof) -> double {
        return joint->velocity(dof);
    };

    return Impl::getJointDataSerialized(this, jointNames, lambda);
}

std::vector<double>
Model::jointAccelerations(const std::vector<std::string>& jointNames) const
{
    auto lambda = [](core::JointPtr joint, const size_t dof) -> double {
        return joint->acceleration(dof);
    };

    return Impl::getJointDataSerialized(this, jointNames, lambda);
}

std::vector<double>
Model::jointGeneralizedForces(const std::vector<std::string>& jointNames) const
{
    auto lambda = [](core::JointPtr joint, const size_t dof) -> double {
        return joint->generalizedForce(dof);
    };

    return Impl::getJointDataSerialized(this, jointNames, lambda);
}

scenario::core::JointLimit
Model::jointLimits(const std::vector<std::string>& jointNames) const
{
    const std::vector<std::string>& jointSerialization =
        jointNames.empty() ? this->jointNames() : jointNames;

    std::vector<double> low;
    std::vector<double> high;

    low.reserve(jointSerialization.size());
    high.reserve(jointSerialization.size());

    for (const auto& joint : this->joints(jointSerialization)) {
        auto limit = joint->jointPositionLimit();
        std::move(limit.min.begin(), limit.min.end(), std::back_inserter(low));
        std::move(limit.max.begin(), limit.max.end(), std::back_inserter(high));
    }

    return core::JointLimit(low, high);
}

bool Model::setJointControlMode(const scenario::core::JointControlMode mode,
                                const std::vector<std::string>& jointNames)
{
    const std::vector<std::string>& jointSerialization =
        jointNames.empty() ? this->jointNames() : jointNames;

    bool ok = true;

    for (auto& joint : this->joints(jointSerialization)) {
        ok = ok && joint->setControlMode(mode);
    }

    return ok;
}

std::vector<scenario::core::LinkPtr>
Model::links(const std::vector<std::string>& linkNames) const
{
    const std::vector<std::string>& linkSerialization =
        linkNames.empty() ? this->linkNames() : linkNames;

    std::vector<core::LinkPtr> links;

    for (const auto& linkName : linkSerialization) {
        links.push_back(this->getLink(linkName));
    }

    return links;
}

std::vector<scenario::core::JointPtr>
Model::joints(const std::vector<std::string>& jointNames) const
{
    const std::vector<std::string>& jointSerialization =
        jointNames.empty() ? this->jointNames() : jointNames;

    std::vector<core::JointPtr> joints;

    for (const auto& jointName : jointSerialization) {
        joints.push_back(this->getJoint(jointName));
    }

    return joints;
}

bool Model::setJointPositionTargets(const std::vector<double>& positions,
                                    const std::vector<std::string>& jointNames)
{
    auto lambda = [](core::JointPtr joint,
                     const double position,
                     const size_t dof) -> bool {
        return joint->setPositionTarget(position, dof);
    };

    return Impl::setJointDataSerialized(this, positions, jointNames, lambda);
}

bool Model::setJointVelocityTargets(const std::vector<double>& velocities,
                                    const std::vector<std::string>& jointNames)
{
    auto lambda = [](core::JointPtr joint,
                     const double velocity,
                     const size_t dof) -> bool {
        return joint->setVelocityTarget(velocity, dof);
    };

    return Impl::setJointDataSerialized(this, velocities, jointNames, lambda);
}

bool Model::setJointAccelerationTargets(
    const std::vector<double>& accelerations,
    const std::vector<std::string>& jointNames)
{
    auto lambda = [](core::JointPtr joint,
                     const double acceleration,
                     const size_t dof) -> bool {
        return joint->setAccelerationTarget(acceleration, dof);
    };

    return Impl::setJointDataSerialized(
        this, accelerations, jointNames, lambda);
}

bool Model::setJointGeneralizedForceTargets(
    const std::vector<double>& forces,
    const std::vector<std::string>& jointNames)
{
    auto lambda =
        [](core::JointPtr joint, const double force, const size_t dof) -> bool {
        return joint->setGeneralizedForceTarget(force, dof);
    };

    return Impl::setJointDataSerialized(this, forces, jointNames, lambda);
}

std::vector<double>
Model::jointPositionTargets(const std::vector<std::string>& jointNames) const
{
    auto lambda = [](core::JointPtr joint, const size_t dof) -> double {
        return joint->positionTarget(dof);
    };

    return Impl::getJointDataSerialized(this, jointNames, lambda);
}

std::vector<double>
Model::jointVelocityTargets(const std::vector<std::string>& jointNames) const
{
    auto lambda = [](core::JointPtr joint, const size_t dof) -> double {
        return joint->velocityTarget(dof);
    };

    return Impl::getJointDataSerialized(this, jointNames, lambda);
}

std::vector<double> Model::jointAccelerationTargets(
    const std::vector<std::string>& jointNames) const
{
    auto lambda = [](core::JointPtr joint, const size_t dof) -> double {
        return joint->accelerationTarget(dof);
    };

    return Impl::getJointDataSerialized(this, jointNames, lambda);
}

std::vector<double> Model::jointGeneralizedForceTargets(
    const std::vector<std::string>& jointNames) const
{
    auto lambda = [](core::JointPtr joint, const size_t dof) -> double {
        return joint->generalizedForceTarget(dof);
    };

    return Impl::getJointDataSerialized(this, jointNames, lambda);
}

std::string Model::baseFrame() const
{
    // Get all the canonical links of the model
    auto candidateBaseLinks = m_ecm->EntitiesByComponents(
        ignition::gazebo::components::CanonicalLink(),
        ignition::gazebo::components::ParentEntity(pImpl->model.Entity()));

    if (candidateBaseLinks.size() == 0) {
        throw exceptions::ModelError("Failed to find the canonical link",
                                     this->name());
    }

    if (candidateBaseLinks.size() > 1) {
        throw exceptions::ModelError("Found multiple canonical link",
                                     this->name());
    }

    // Get the name of the base link
    std::string baseLinkName = utils::getExistingComponentData< //
        ignition::gazebo::components::Name>(m_ecm, candidateBaseLinks.front());

    return baseLinkName;
}

std::array<double, 3> Model::basePosition() const
{
    // Get the model pose
    const ignition::math::Pose3d& world_H_model =
        utils::getExistingComponentData<ignition::gazebo::components::Pose>(
            m_ecm, m_entity);

    return utils::fromIgnitionPose(world_H_model).position;
}

std::array<double, 4> Model::baseOrientation() const
{
    // Get the model pose
    const ignition::math::Pose3d& world_H_model =
        utils::getExistingComponentData<ignition::gazebo::components::Pose>(
            m_ecm, m_entity);

    return utils::fromIgnitionPose(world_H_model).orientation;
}

std::array<double, 3> Model::baseBodyLinearVelocity() const
{
    const auto& baseWorldLinearVelocity =
        utils::toIgnitionVector3(this->baseWorldLinearVelocity());

    // Get the model pose
    const ignition::math::Pose3d& world_H_model =
        utils::getExistingComponentData<ignition::gazebo::components::Pose>(
            m_ecm, m_entity);

    return utils::fromIgnitionVector( //
        world_H_model.Inverse().Rot() * baseWorldLinearVelocity);
}

std::array<double, 3> Model::baseBodyAngularVelocity() const
{
    const auto& baseWorldAngularVelocity =
        utils::toIgnitionVector3(this->baseWorldAngularVelocity());

    // Get the model pose
    const ignition::math::Pose3d& world_H_model =
        utils::getExistingComponentData<ignition::gazebo::components::Pose>(
            m_ecm, m_entity);

    return utils::fromIgnitionVector( //
        world_H_model.Inverse().Rot() * baseWorldAngularVelocity);
}

std::array<double, 3> Model::baseWorldLinearVelocity() const
{
    // Get the entity of the canonical link
    const auto canonicalLinkEntity = m_ecm->EntityByComponents(
        ignition::gazebo::components::Link(),
        ignition::gazebo::components::CanonicalLink(),
        ignition::gazebo::components::Name(this->baseFrame()),
        ignition::gazebo::components::ParentEntity(m_entity));

    // Get the Pose component of the canonical link.
    // This is the fixed transformation between the model and the base.
    const auto& M_H_B = utils::getExistingComponentData< //
        ignition::gazebo::components::Pose>(m_ecm, canonicalLinkEntity);

    // Get the rotation between base link and world
    const auto& W_R_B = utils::toIgnitionQuaternion(
        this->getLink(this->baseFrame())->orientation());

    // Get the linear velocity of the canonical link
    const ignition::math::Vector3d& canonicalLinkLinearVelocity =
        utils::toIgnitionVector3(
            this->getLink(this->baseFrame())->worldLinearVelocity());

    // Get the angular velocity of the canonical link
    const ignition::math::Vector3d& canonicalLinkAngularVelocity =
        utils::toIgnitionVector3(
            this->getLink(this->baseFrame())->worldAngularVelocity());

    // Convert the base velocity to the model mixed velocity
    const auto& modelLinearVelocity = utils::fromBaseToModelLinearVelocity( //
        canonicalLinkLinearVelocity,
        canonicalLinkAngularVelocity,
        M_H_B,
        W_R_B);

    // Return the linear part
    return utils::fromIgnitionVector(modelLinearVelocity);
}

std::array<double, 3> Model::baseWorldAngularVelocity() const
{
    // We could use the helper to convert the base link velocity to the model
    // mixed velocity. However, since there's only a rigid transformation
    // between base and model frame, and the velocity is computed in the world
    // frame, we do not need to perform any conversion.

    // Get the name of the base link
    const std::string& baseLink = this->baseFrame();

    // Return the angular velocity of the base link
    return this->getLink(baseLink)->worldAngularVelocity();
}

bool Model::setBasePoseTarget(const std::array<double, 3>& position,
                              const std::array<double, 4>& orientation)
{
    const ignition::math::Pose3d& basePoseTarget =
        utils::toIgnitionPose(core::Pose{position, orientation});

    utils::setComponentData<ignition::gazebo::components::BasePoseTarget>(
        m_ecm, m_entity, basePoseTarget);

    return true;
}

bool Model::setBasePositionTarget(const std::array<double, 3>& position)
{
    const auto& basePoseTargetComponent = utils::getComponent< //
        ignition::gazebo::components::BasePoseTarget>(
        m_ecm, m_entity, ignition::math::Pose3d::Zero);

    const auto basePoseTarget =
        ignition::math::Pose3d(utils::toIgnitionVector3(position),
                               basePoseTargetComponent->Data().Rot());

    utils::setExistingComponentData<
        ignition::gazebo::components::BasePoseTarget>(
        m_ecm, m_entity, basePoseTarget);

    return true;
}

bool Model::setBaseOrientationTarget(const std::array<double, 4>& orientation)
{
    const auto& basePoseTargetComponent = utils::getComponent< //
        ignition::gazebo::components::BasePoseTarget,
        ignition::math::Pose3d>(m_ecm, m_entity);

    const auto basePoseTarget =
        ignition::math::Pose3d(basePoseTargetComponent->Data().Pos(),
                               utils::toIgnitionQuaternion(orientation));

    utils::setComponentData<ignition::gazebo::components::BasePoseTarget>(
        m_ecm, m_entity, basePoseTarget);

    return true;
}

bool Model::setBaseWorldVelocityTarget(const std::array<double, 3>& linear,
                                       const std::array<double, 3>& angular)
{
    // TODO: the velocity target is not used by Gazebo.
    //       Should we compute the base velocity from the inputs?
    return this->setBaseWorldLinearVelocityTarget(linear)
           && this->setBaseWorldAngularVelocityTarget(angular);
}

bool Model::setBaseWorldLinearVelocityTarget(
    const std::array<double, 3>& linear)
{
    const ignition::math::Vector3d& baseWorldLinearVelocity =
        utils::toIgnitionVector3(linear);

    utils::setComponentData<
        ignition::gazebo::components::BaseWorldLinearVelocityTarget>(
        m_ecm, m_entity, baseWorldLinearVelocity);

    return true;
}

bool Model::setBaseWorldAngularVelocityTarget(
    const std::array<double, 3>& angular)
{
    const ignition::math::Vector3d& baseWorldAngularVelocity =
        utils::toIgnitionVector3(angular);

    utils::setComponentData<
        ignition::gazebo::components::BaseWorldAngularVelocityTarget>(
        m_ecm, m_entity, baseWorldAngularVelocity);

    return true;
}

bool Model::setBaseWorldLinearAccelerationTarget(
    const std::array<double, 3>& linear)
{
    const ignition::math::Vector3d& baseWorldLinearAcceleration =
        utils::toIgnitionVector3(linear);

    // TODO: do we need to convert the model acceleration to base link
    //       acceleration? Contrarily to the velocity, this component
    //       is not used in gazebo but from custom controllers.

    utils::setComponentData<
        ignition::gazebo::components::BaseWorldLinearAccelerationTarget>(
        m_ecm, m_entity, baseWorldLinearAcceleration);

    return true;
}

bool Model::setBaseWorldAngularAccelerationTarget(
    const std::array<double, 3>& angular)
{
    const ignition::math::Vector3d& baseWorldAngularAcceleration =
        utils::toIgnitionVector3(angular);

    utils::setComponentData<
        ignition::gazebo::components::BaseWorldAngularAccelerationTarget>(
        m_ecm, m_entity, baseWorldAngularAcceleration);

    return true;
}

std::array<double, 3> Model::basePositionTarget() const
{
    const ignition::math::Pose3d& basePoseTarget =
        utils::getExistingComponentData<
            ignition::gazebo::components::BasePoseTarget>(m_ecm, m_entity);

    return utils::fromIgnitionPose(basePoseTarget).position;
}

std::array<double, 4> Model::baseOrientationTarget() const
{
    const ignition::math::Pose3d& basePoseTarget =
        utils::getExistingComponentData<
            ignition::gazebo::components::BasePoseTarget>(m_ecm, m_entity);

    return utils::fromIgnitionPose(basePoseTarget).orientation;
}

std::array<double, 3> Model::baseWorldLinearVelocityTarget() const
{
    const ignition::math::Vector3d& baseLinTarget =
        utils::getExistingComponentData<
            ignition::gazebo::components::BaseWorldLinearVelocityTarget>(
            m_ecm, m_entity);

    return utils::fromIgnitionVector(baseLinTarget);
}

std::array<double, 3> Model::baseWorldAngularVelocityTarget() const
{
    const ignition::math::Vector3d& baseAngTarget =
        utils::getExistingComponentData<
            ignition::gazebo::components::BaseWorldAngularVelocityTarget>(
            m_ecm, m_entity);

    return utils::fromIgnitionVector(baseAngTarget);
}

std::array<double, 3> Model::baseWorldLinearAccelerationTarget() const
{
    const ignition::math::Vector3d& baseLinTarget =
        utils::getExistingComponentData<
            ignition::gazebo::components::BaseWorldLinearAccelerationTarget>(
            m_ecm, m_entity);

    return utils::fromIgnitionVector(baseLinTarget);
}

std::array<double, 3> Model::baseWorldAngularAccelerationTarget() const
{
    const ignition::math::Vector3d& baseAngTarget =
        utils::getExistingComponentData<
            ignition::gazebo::components::BaseWorldAngularAccelerationTarget>(
            m_ecm, m_entity);

    return utils::fromIgnitionVector(baseAngTarget);
}

// ======================
// Implementation Methods
// ======================

std::vector<double> Model::Impl::getJointDataSerialized(
    const Model* model,
    const std::vector<std::string>& jointNames,
    std::function<double(core::JointPtr, const size_t)> getJointData)
{
    const std::vector<std::string>& jointSerialization =
        jointNames.empty() ? model->jointNames() : jointNames;

    std::vector<double> data;
    data.reserve(model->dofs());

    for (auto& joint : model->joints(jointSerialization)) {
        for (size_t dof = 0; dof < joint->dofs(); ++dof) {
            data.push_back(getJointData(joint, dof));
        }
    }

    return data;
}

bool Model::Impl::setJointDataSerialized(
    Model* model,
    const std::vector<double>& data,
    const std::vector<std::string>& jointNames,
    std::function<bool(core::JointPtr, const double, const size_t)>
        setJointData)
{
    std::vector<std::string> jointSerialization;

    size_t expectedDOFs = 0;

    if (!jointNames.empty()) {
        jointSerialization = jointNames;

        for (auto& joint : model->joints(jointSerialization)) {
            expectedDOFs += joint->dofs();
        }
    }
    else {
        expectedDOFs = model->dofs();
        jointSerialization = model->jointNames();
    }

    if (data.size() != expectedDOFs) {
        sError << "The size of the forces does not match the considered "
                  "joint's DOFs"
               << std::endl;
        return false;
    }

    auto it = data.begin();

    for (auto& joint : model->joints(jointNames)) {
        for (size_t dof = 0; dof < joint->dofs(); ++dof) {
            if (!setJointData(joint, *it++, dof)) {
                sError << "Failed to set force of joint '" << joint->name()
                       << "'" << std::endl;
                return false;
            }
        }
    }
    assert(it == data.end());
    return true;
}
