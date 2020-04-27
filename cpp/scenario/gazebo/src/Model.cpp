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
#include "scenario/gazebo/components/WorldVelocityCmd.h"
#include "scenario/gazebo/exceptions.h"
#include "scenario/gazebo/helpers.h"

#include <ignition/common/Event.hh>
#include <ignition/gazebo/Events.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/CanonicalLink.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
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
    ignition::gazebo::EventManager* eventManager = nullptr;
    ignition::gazebo::EntityComponentManager* ecm = nullptr;

    ignition::gazebo::Model model;
    ignition::gazebo::Entity modelEntity = ignition::gazebo::kNullEntity;

    using LinkName = std::string;
    using JointName = std::string;

    std::unordered_map<LinkName, scenario::gazebo::LinkPtr> links;
    std::unordered_map<JointName, scenario::gazebo::JointPtr> joints;

    struct
    {
        std::vector<std::string> linksInContact;
    } buffers;

    static std::vector<double> getJointDataSerialized(
        const Model* model,
        const std::vector<std::string>& jointNames,
        std::function<double(JointPtr, const size_t)> getJointData);

    static bool setJointDataSerialized(
        Model* model,
        const std::vector<double>& data,
        const std::vector<std::string>& jointNames,
        std::function<bool(JointPtr, const double, const size_t)> setDataToDOF);
};

Model::Model()
    : pImpl{std::make_unique<Impl>()}
{}

uint64_t Model::id() const
{
    // Get the parent world
    WorldPtr parentWorld = utils::getParentWorld(
        pImpl->ecm, pImpl->eventManager, pImpl->modelEntity);
    assert(parentWorld);

    // Build a unique string identifier of this model
    std::string scopedModelName = parentWorld->name() + "::" + this->name();

    // Return the hashed string
    return std::hash<std::string>{}(scopedModelName);
}

Model::~Model() = default;

bool Model::initialize(const ignition::gazebo::Entity modelEntity,
                       ignition::gazebo::EntityComponentManager* ecm,
                       ignition::gazebo::EventManager* eventManager)
{
    if (modelEntity == ignition::gazebo::kNullEntity || !ecm || !eventManager) {
        return false;
    }

    pImpl->ecm = ecm;
    pImpl->modelEntity = modelEntity;
    pImpl->eventManager = eventManager;

    // Create the model
    pImpl->model = ignition::gazebo::Model(modelEntity);

    // Check that the model is valid
    if (!pImpl->model.Valid(*ecm)) {
        gymppError << "The model entity is not valid" << std::endl;
        return false;
    }

    return true;
}

bool Model::createECMResources()
{
    gymppMessage << "Model: [" << pImpl->modelEntity << "] " << this->name()
                 << std::endl;

    // Create required link resources
    gymppMessage << "Links:" << std::endl;
    for (const auto& linkName : this->linkNames()) {
        if (!this->getLink(linkName)->createECMResources()) {
            gymppError << "Failed to initialize ECM link resources"
                       << std::endl;
            return false;
        }
    }

    // Create required model resources
    gymppMessage << "Joints:" << std::endl;
    for (const auto& jointName : this->jointNames()) {
        if (!this->getJoint(jointName)->createECMResources()) {
            gymppError << "Failed to initialize ECM joint resources"
                       << std::endl;
            return false;
        }
    }

    // Initialize the Joint Controller period as maximum duration.
    // In this way controllers are never updated unless a new period is
    // configured.
    pImpl->ecm->CreateComponent(
        pImpl->modelEntity,
        ignition::gazebo::components::JointControllerPeriod(
            std::chrono::steady_clock::duration().max()));

    return true;
}

bool Model::insertModelPlugin(const std::string& libName,
                              const std::string& className,
                              const std::string& context)
{
    // Create a new <plugin name="" filename=""> element without context
    sdf::ElementPtr pluginElement =
        utils::getPluginSDFElement(libName, className);

    // Insert the context into the plugin element
    if (!context.empty()) {

        std::shared_ptr<sdf::Root> contextRoot =
            utils::getSdfRootFromString(context);

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
        pImpl->modelEntity, wrapped);

    return true;
}

bool Model::historyOfAppliedJointForcesEnabled(
    const std::vector<std::string>& jointNames) const
{
    std::vector<std::string> jointSerialization =
        jointNames.empty() ? this->jointNames() : jointNames;

    bool enabled = true;

    for (const auto& joint : this->joints(jointSerialization)) {
        enabled = enabled && joint->historyOfAppliedJointForcesEnabled();
    }

    return enabled;
}

bool Model::enableHistoryOfAppliedJointForces(
    const bool enable,
    const size_t maxHistorySizePerJoint,
    const std::vector<std::string>& jointNames)
{
    const std::vector<std::string> jointSerialization =
        jointNames.empty() ? this->jointNames() : jointNames;

    bool ok = true;

    for (const auto& joint : this->joints(jointSerialization)) {
        ok = ok
             && joint->enableHistoryOfAppliedJointForces(
                 enable, maxHistorySizePerJoint);
    }

    return ok;
}

std::vector<double> Model::historyOfAppliedJointForces(
    const std::vector<std::string>& jointNames) const
{
    const std::vector<std::string> jointSerialization =
        jointNames.empty() ? this->jointNames() : jointNames;

    std::vector<double> allAppliedJointForces;
    allAppliedJointForces.reserve(this->nrOfJoints() * 3);

    for (const auto& joint : this->joints(jointSerialization)) {
        std::vector<double> history = joint->historyOfAppliedJointForces();
        std::move(history.begin(),
                  history.end(),
                  std::back_inserter(allAppliedJointForces));
    }

    return allAppliedJointForces;
}

bool Model::valid() const
{
    // TODO: extend the checks
    return pImpl->model.Valid(*pImpl->ecm);
}

size_t Model::dofs() const
{
    size_t dofs = 0;

    for (const auto& jointName : jointNames()) {
        dofs += this->getJoint(jointName)->dofs();
    }

    return dofs;
}

std::string Model::name() const
{
    return pImpl->model.Name(*pImpl->ecm);
}

size_t Model::nrOfLinks() const
{
    return this->linkNames().size();
}

size_t Model::nrOfJoints() const
{
    return this->jointNames().size();
}

scenario::gazebo::LinkPtr Model::getLink(const std::string& linkName) const
{
    if (pImpl->links.find(linkName) != pImpl->links.end()) {
        assert(pImpl->links.at(linkName));
        return pImpl->links.at(linkName);
    }

    auto linkEntity = pImpl->model.LinkByName(*pImpl->ecm, linkName);

    if (linkEntity == ignition::gazebo::kNullEntity) {
        throw exceptions::LinkNotFound(linkName);
    }

    // Create the link
    auto link = std::make_shared<scenario::gazebo::Link>();

    if (!link->initialize(linkEntity, pImpl->ecm, pImpl->eventManager)) {
        throw exceptions::LinkError("Failed to initialize link", linkName);
    }

    // Cache the link instance
    pImpl->links[linkName] = link;

    return link;
}

scenario::gazebo::JointPtr Model::getJoint(const std::string& jointName) const
{
    if (pImpl->joints.find(jointName) != pImpl->joints.end()) {
        assert(pImpl->joints.at(jointName));
        return pImpl->joints.at(jointName);
    }

    auto jointEntity = pImpl->model.JointByName(*pImpl->ecm, jointName);

    if (jointEntity == ignition::gazebo::kNullEntity) {
        throw exceptions::JointNotFound(jointName);
    }

    // Create the joint
    auto joint = std::make_shared<scenario::gazebo::Joint>();

    if (!joint->initialize(jointEntity, pImpl->ecm, pImpl->eventManager)) {
        throw exceptions::JointError("Failed to initialize joint", jointName);
    }

    // Cache the joint instance
    pImpl->joints[jointName] = joint;

    return joint;
}

std::vector<std::string> Model::linkNames(const bool scoped) const
{
    std::vector<std::string> linkNames;

    pImpl->ecm->Each<ignition::gazebo::components::Name,
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
            if (parentEntityComponent->Data() != pImpl->modelEntity) {
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

    return linkNames;
}

std::vector<std::string> Model::jointNames(const bool scoped) const
{
    std::vector<std::string> jointNames;

    pImpl->ecm->Each<ignition::gazebo::components::Name,
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
            if (parentEntityComponent->Data() != pImpl->modelEntity) {
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

    return jointNames;
}

double Model::controllerPeriod() const
{
    auto duration = utils::getExistingComponentData< //
        ignition::gazebo::components::JointControllerPeriod>(
        pImpl->ecm, pImpl->modelEntity);

    return utils::steadyClockDurationToDouble(duration);
}

bool Model::setControllerPeriod(const double period)
{
    if (period <= 0) {
        gymppError << "The controller period must be greater than zero"
                   << std::endl;
        return false;
    }

    // Store the new period in the ECM
    utils::setExistingComponentData<
        ignition::gazebo::components::JointControllerPeriod>(
        pImpl->ecm,
        pImpl->modelEntity,
        utils::doubleToSteadyClockDuration(period));
    return true;
}

bool Model::contactsEnabled() const
{
    bool enabled = true;

    for (auto& link : this->links()) {
        enabled = enabled && link->contactsEnabled();
    }

    return enabled;
}

bool Model::enableContacts(const bool enable)
{
    bool ok = true;

    for (auto& link : this->links()) {
        ok = ok && link->enableContactDetection(enable);
    }

    return ok;
}

bool Model::selfCollisions() const
{
    throw exceptions::NotImplementedError(__FUNCTION__);
}

bool Model::enableSelfCollisions(const bool /*enable*/)
{
    throw exceptions::NotImplementedError(__FUNCTION__);
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

std::vector<scenario::base::ContactData>
Model::contacts(const std::vector<std::string>& linkNames) const
{
    std::vector<std::string> linkSerialization =
        linkNames.empty() ? this->linkNames() : linkNames;

    std::vector<scenario::base::ContactData> allContacts;

    for (const auto& linkName : linkSerialization) {
        auto contacts = this->getLink(linkName)->contactData();
        std::move(contacts.begin(), //
                  contacts.end(),
                  std::back_inserter(allContacts));
    }

    return allContacts;
}

std::vector<double>
Model::jointPositions(const std::vector<std::string>& jointNames) const
{
    auto lambda = [](JointPtr joint, const size_t dof) -> double {
        return joint->position(dof);
    };

    return Impl::getJointDataSerialized(this, jointNames, lambda);
}

std::vector<double>
Model::jointVelocities(const std::vector<std::string>& jointNames) const
{
    auto lambda = [](JointPtr joint, const size_t dof) -> double {
        return joint->velocity(dof);
    };

    return Impl::getJointDataSerialized(this, jointNames, lambda);
}

scenario::base::JointLimit
Model::jointLimits(const std::vector<std::string>& jointNames) const
{
    const std::vector<std::string> jointSerialization =
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

    return base::JointLimit(low, high);
}

bool Model::setJointControlMode(const scenario::base::JointControlMode mode,
                                const std::vector<std::string>& jointNames)
{
    std::vector<std::string> jointSerialization =
        jointNames.empty() ? this->jointNames() : jointNames;

    bool ok = true;

    for (auto& joint : this->joints(jointSerialization)) {
        ok = ok && joint->setControlMode(mode);
    }

    return ok;
}

std::vector<LinkPtr>
Model::links(const std::vector<std::string>& linkNames) const
{
    std::vector<std::string> linkSerialization =
        linkNames.empty() ? this->linkNames() : linkNames;

    std::vector<LinkPtr> links;

    for (const auto& linkName : linkSerialization) {
        links.push_back(this->getLink(linkName));
    }

    return links;
}

std::vector<JointPtr>
Model::joints(const std::vector<std::string>& jointNames) const
{
    const std::vector<std::string> jointSerialization =
        jointNames.empty() ? this->jointNames() : jointNames;

    std::vector<JointPtr> joints;

    for (const auto& jointName : jointSerialization) {
        joints.push_back(this->getJoint(jointName));
    }

    return joints;
}

bool Model::setJointPositionTargets(const std::vector<double>& positions,
                                    const std::vector<std::string>& jointNames)
{
    auto lambda =
        [](JointPtr joint, const double position, const size_t dof) -> bool {
        return joint->setPositionTarget(position, dof);
    };

    return Impl::setJointDataSerialized(this, positions, jointNames, lambda);
}

bool Model::setJointVelocityTargets(const std::vector<double>& velocities,
                                    const std::vector<std::string>& jointNames)
{
    auto lambda =
        [](JointPtr joint, const double velocity, const size_t dof) -> bool {
        return joint->setVelocityTarget(velocity, dof);
    };

    return Impl::setJointDataSerialized(this, velocities, jointNames, lambda);
}

bool Model::setJointAccelerationTargets(
    const std::vector<double>& velocities,
    const std::vector<std::string>& jointNames)
{
    auto lambda =
        [](JointPtr joint, const double velocity, const size_t dof) -> bool {
        return joint->setAccelerationTarget(velocity, dof);
    };

    return Impl::setJointDataSerialized(this, velocities, jointNames, lambda);
}

bool Model::setJointGeneralizedForceTargets(
    const std::vector<double>& forces,
    const std::vector<std::string>& jointNames)
{
    auto lambda =
        [](JointPtr joint, const double force, const size_t dof) -> bool {
        return joint->setGeneralizedForceTarget(force, dof);
    };

    return Impl::setJointDataSerialized(this, forces, jointNames, lambda);
}

bool Model::resetJointPositions(const std::vector<double>& positions,
                                const std::vector<std::string>& jointNames)
{
    auto lambda =
        [](JointPtr joint, const double position, const size_t dof) -> bool {
        return joint->resetPosition(position, dof);
    };

    return Impl::setJointDataSerialized(this, positions, jointNames, lambda);
}

bool Model::resetJointVelocities(const std::vector<double>& velocities,
                                 const std::vector<std::string>& jointNames)
{
    auto lambda =
        [](JointPtr joint, const double velocity, const size_t dof) -> bool {
        return joint->resetVelocity(velocity, dof);
    };

    return Impl::setJointDataSerialized(this, velocities, jointNames, lambda);
}

std::vector<double>
Model::jointPositionTargets(const std::vector<std::string>& jointNames) const
{
    auto lambda = [](JointPtr joint, const size_t dof) -> double {
        return joint->positionTarget(dof);
    };

    return Impl::getJointDataSerialized(this, jointNames, lambda);
}

std::vector<double>
Model::jointVelocityTargets(const std::vector<std::string>& jointNames) const
{
    auto lambda = [](JointPtr joint, const size_t dof) -> double {
        return joint->velocityTarget(dof);
    };

    return Impl::getJointDataSerialized(this, jointNames, lambda);
}

std::vector<double> Model::jointAccelerationTargets(
    const std::vector<std::string>& jointNames) const
{
    auto lambda = [](JointPtr joint, const size_t dof) -> double {
        return joint->accelerationTarget(dof);
    };

    return Impl::getJointDataSerialized(this, jointNames, lambda);
}

std::vector<double> Model::jointGeneralizedForceTargets(
    const std::vector<std::string>& jointNames) const
{
    auto lambda = [](JointPtr joint, const size_t dof) -> double {
        return joint->generalizedForceTarget(dof);
    };

    return Impl::getJointDataSerialized(this, jointNames, lambda);
}

std::string Model::baseFrame() const
{
    // Get all the canonical links of the model
    auto candidateBaseLinks = pImpl->ecm->EntitiesByComponents(
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
        ignition::gazebo::components::Name>(pImpl->ecm,
                                            candidateBaseLinks.front());

    return baseLinkName;
}

bool Model::setBaseFrame(const std::string& frameName)
{
    const auto linkNames = this->linkNames();

    if (std::find(linkNames.begin(), linkNames.end(), frameName)
        == linkNames.end()) {
        gymppError
            << "Failed to set the model base on the frame of nonexistent link '"
            << frameName << "'" << std::endl;
        return false;
    }

    if (frameName == this->baseFrame()) {
        gymppDebug << "Frame '" << baseFrame()
                   << "' is already the current model base" << std::endl;
        return true;
    }

    gymppError << "Changing the base link is not yet supported" << std::endl;
    throw exceptions::NotImplementedError(__FUNCTION__);
}

bool Model::fixedBase() const
{
    throw exceptions::NotImplementedError(__FUNCTION__);
}

bool Model::setAsFixedBase(const bool fixedBase)
{
    throw exceptions::NotImplementedError(__FUNCTION__);
}

std::array<double, 3> Model::basePosition() const
{
    // Get the model pose
    ignition::math::Pose3d world_H_model = utils::getExistingComponentData< //
        ignition::gazebo::components::Pose>(pImpl->ecm, pImpl->modelEntity);

    return utils::fromIgnitionPose(world_H_model).position;
}

std::array<double, 4> Model::baseOrientation() const
{
    // Get the model pose
    ignition::math::Pose3d world_H_model = utils::getExistingComponentData< //
        ignition::gazebo::components::Pose>(pImpl->ecm, pImpl->modelEntity);

    return utils::fromIgnitionPose(world_H_model).orientation;
}

std::array<double, 3> Model::baseBodyLinearVelocity() const
{
    auto baseWorldLinearVelocity =
        utils::toIgnitionVector3(this->baseWorldLinearVelocity());

    // Get the model pose
    ignition::math::Pose3d world_H_model = utils::getExistingComponentData< //
        ignition::gazebo::components::Pose>(pImpl->ecm, pImpl->modelEntity);

    return utils::fromIgnitionVector( //
        world_H_model.Inverse().Rot() * baseWorldLinearVelocity);
}

std::array<double, 3> Model::baseBodyAngularVelocity() const
{
    auto baseWorldAngularVelocity =
        utils::toIgnitionVector3(this->baseWorldAngularVelocity());

    // Get the model pose
    ignition::math::Pose3d world_H_model = utils::getExistingComponentData< //
        ignition::gazebo::components::Pose>(pImpl->ecm, pImpl->modelEntity);

    return utils::fromIgnitionVector( //
        world_H_model.Inverse().Rot() * baseWorldAngularVelocity);
}

std::array<double, 3> Model::baseWorldLinearVelocity() const
{
    // Get the entity of the canonical link
    auto canonicalLinkEntity = pImpl->ecm->EntityByComponents(
        ignition::gazebo::components::Link(),
        ignition::gazebo::components::CanonicalLink(),
        ignition::gazebo::components::Name(this->baseFrame()),
        ignition::gazebo::components::ParentEntity(pImpl->modelEntity));

    // Get the Pose component of the canonical link.
    // This is the fixed transformation between the model and the base.
    auto& M_H_B = utils::getExistingComponentData< //
        ignition::gazebo::components::Pose>(pImpl->ecm, canonicalLinkEntity);

    // Get the rotation between base link and world
    auto W_R_B = utils::toIgnitionQuaternion(
        this->getLink(this->baseFrame())->orientation());

    // Get the linear velocity of the canonical link
    ignition::math::Vector3d canonicalLinkLinearVelocity =
        utils::toIgnitionVector3(
            this->getLink(this->baseFrame())->worldLinearVelocity());

    // Get the angular velocity of the canonical link
    ignition::math::Vector3d canonicalLinkAngularVelocity =
        utils::toIgnitionVector3(
            this->getLink(this->baseFrame())->worldAngularVelocity());

    // Convert the base velocity to the model mixed velocity
    auto [modelLinearVelocity, _] = utils::fromBaseToModelVelocity( //
        canonicalLinkLinearVelocity,
        canonicalLinkAngularVelocity,
        M_H_B,
        W_R_B);

    // Return the linear partmodel6DVelocity.first
    return utils::fromIgnitionVector(modelLinearVelocity);
}

std::array<double, 3> Model::baseWorldAngularVelocity() const
{
    // We could use the helper to convert the base link velocity to the model
    // mixed velocity. However, since there's only a rigid transformation
    // between base and model frame, and the velocity is computed in the world
    // frame, we do not need to perform any conversion.

    // Get the name of the base link
    const std::string baseLink = this->baseFrame();

    // Return the angular velocity of the base link
    return this->getLink(baseLink)->worldAngularVelocity();
}

bool Model::resetBasePose(const std::array<double, 3>& position,
                          const std::array<double, 4>& orientation)
{
    // Construct the desired transform between world and base
    base::Pose pose;
    pose.position = position;
    pose.orientation = orientation;
    ignition::math::Pose3d world_H_base = utils::toIgnitionPose(pose);

    // Get the entity of the canonical link
    auto canonicalLinkEntity = pImpl->ecm->EntityByComponents(
        ignition::gazebo::components::Link(),
        ignition::gazebo::components::CanonicalLink(),
        ignition::gazebo::components::Name(this->baseFrame()),
        ignition::gazebo::components::ParentEntity(pImpl->modelEntity));

    if (canonicalLinkEntity == ignition::gazebo::kNullEntity) {
        gymppError << "Failed to get entity of canonical link" << std::endl;
        return false;
    }

    // Get the Pose component of the canonical link.
    // This is the fixed transformation between the model and the base.
    auto& model_H_base = utils::getExistingComponentData< //
        ignition::gazebo::components::Pose>(pImpl->ecm, canonicalLinkEntity);

    // Compute the robot pose that corresponds to the desired base pose
    const ignition::math::Pose3d& world_H_model =
        world_H_base * model_H_base.Inverse();

    // Store the new pose
    utils::setComponentData<ignition::gazebo::components::WorldPoseCmd>(
        pImpl->ecm, pImpl->modelEntity, world_H_model);

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
    return this->resetBaseWorldVelocity(linear,
                                        this->baseWorldAngularVelocity());
}

bool Model::resetBaseWorldAngularVelocity(const std::array<double, 3>& angular)
{
    return this->resetBaseWorldVelocity(this->baseWorldLinearVelocity(),
                                        angular);
}

bool Model::resetBaseWorldVelocity(const std::array<double, 3>& linear,
                                   const std::array<double, 3>& angular)
{
    // Get the entity of the canonical (base) link
    auto canonicalLinkEntity = pImpl->ecm->EntityByComponents(
        ignition::gazebo::components::Link(),
        ignition::gazebo::components::CanonicalLink(),
        ignition::gazebo::components::Name(this->baseFrame()),
        ignition::gazebo::components::ParentEntity(pImpl->modelEntity));

    // Get the Pose component of the canonical link.
    // This is the fixed transformation between the model and the base.
    auto& M_H_B = utils::getExistingComponentData< //
        ignition::gazebo::components::Pose>(pImpl->ecm, canonicalLinkEntity);

    // Get the rotation between base link and world
    auto W_R_B = utils::toIgnitionQuaternion(
        this->getLink(this->baseFrame())->orientation());

    // Create the new model velocity
    ignition::gazebo::WorldVelocity baseWorldVelocity;

    // Compute the mixed velocity of the base link
    std::tie(baseWorldVelocity.linear, baseWorldVelocity.angular) =
        utils::fromModelToBaseVelocity(utils::toIgnitionVector3(linear),
                                       utils::toIgnitionVector3(angular),
                                       M_H_B,
                                       W_R_B);

    // Store the new velocity
    utils::setComponentData<ignition::gazebo::components::WorldVelocityCmd>(
        pImpl->ecm, pImpl->modelEntity, baseWorldVelocity);

    return true;
}

bool Model::setBasePoseTarget(const std::array<double, 3>& position,
                              const std::array<double, 4>& orientation)
{
    ignition::math::Pose3d basePoseTarget =
        utils::toIgnitionPose(base::Pose{position, orientation});

    utils::setComponentData<ignition::gazebo::components::BasePoseTarget>(
        pImpl->ecm, pImpl->modelEntity, basePoseTarget);

    return true;
}

bool Model::setBasePositionTarget(const std::array<double, 3>& position)
{
    auto basePoseTargetComponent = utils::getComponent< //
        ignition::gazebo::components::BasePoseTarget>(
        pImpl->ecm, pImpl->modelEntity, ignition::math::Pose3d::Zero);

    auto basePoseTarget =
        ignition::math::Pose3d(utils::toIgnitionVector3(position),
                               basePoseTargetComponent->Data().Rot());

    utils::setExistingComponentData<
        ignition::gazebo::components::BasePoseTarget>(
        pImpl->ecm, pImpl->modelEntity, basePoseTarget);

    return true;
}

bool Model::setBaseOrientationTarget(const std::array<double, 4>& orientation)
{
    auto basePoseTargetComponent = utils::getComponent< //
        ignition::gazebo::components::BasePoseTarget,
        ignition::math::Pose3d>(pImpl->ecm, pImpl->modelEntity);

    auto basePoseTarget =
        ignition::math::Pose3d(basePoseTargetComponent->Data().Pos(),
                               utils::toIgnitionQuaternion(orientation));

    utils::setComponentData<ignition::gazebo::components::BasePoseTarget>(
        pImpl->ecm, pImpl->modelEntity, basePoseTarget);

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
    ignition::math::Vector3d baseWorldLinearVelocity =
        utils::toIgnitionVector3(linear);

    utils::setComponentData<
        ignition::gazebo::components::BaseWorldLinearVelocityTarget>(
        pImpl->ecm, pImpl->modelEntity, baseWorldLinearVelocity);

    return true;
}

bool Model::setBaseWorldAngularVelocityTarget(
    const std::array<double, 3>& angular)
{
    ignition::math::Vector3d baseWorldAngularVelocity =
        utils::toIgnitionVector3(angular);

    utils::setComponentData<
        ignition::gazebo::components::BaseWorldAngularVelocityTarget>(
        pImpl->ecm, pImpl->modelEntity, baseWorldAngularVelocity);

    return true;
}

bool Model::setBaseWorldLinearAccelerationTarget(
    const std::array<double, 3>& linear)
{
    ignition::math::Vector3d baseWorldLinearAcceleration =
        utils::toIgnitionVector3(linear);

    // TODO: do we need to convert the model acceleration to base link
    //       acceleration? Contrarily to the velocity, this component
    //       is not used in gazebo but from custom controllers.

    utils::setComponentData<
        ignition::gazebo::components::BaseWorldLinearAccelerationTarget>(
        pImpl->ecm, pImpl->modelEntity, baseWorldLinearAcceleration);

    return true;
}

bool Model::setBaseWorldAngularAccelerationTarget(
    const std::array<double, 3>& angular)
{
    ignition::math::Vector3d baseWorldAngularAcceleration =
        utils::toIgnitionVector3(angular);

    utils::setComponentData<
        ignition::gazebo::components::BaseWorldAngularAccelerationTarget>(
        pImpl->ecm, pImpl->modelEntity, baseWorldAngularAcceleration);

    return true;
}

std::array<double, 3> Model::basePositionTarget() const
{
    ignition::math::Pose3d& basePoseTarget = utils::getExistingComponentData<
        ignition::gazebo::components::BasePoseTarget>(pImpl->ecm,
                                                      pImpl->modelEntity);

    return utils::fromIgnitionPose(basePoseTarget).position;
}

std::array<double, 4> Model::baseOrientationTarget() const
{
    ignition::math::Pose3d& basePoseTarget = utils::getExistingComponentData<
        ignition::gazebo::components::BasePoseTarget>(pImpl->ecm,
                                                      pImpl->modelEntity);

    return utils::fromIgnitionPose(basePoseTarget).orientation;
}

std::array<double, 3> Model::baseWorldLinearVelocityTarget() const
{
    ignition::math::Vector3d& baseLinTarget = utils::getExistingComponentData<
        ignition::gazebo::components::BaseWorldLinearVelocityTarget>(
        pImpl->ecm, pImpl->modelEntity);

    return utils::fromIgnitionVector(baseLinTarget);
}

std::array<double, 3> Model::baseWorldAngularVelocityTarget() const
{
    ignition::math::Vector3d& baseAngTarget = utils::getExistingComponentData<
        ignition::gazebo::components::BaseWorldAngularVelocityTarget>(
        pImpl->ecm, pImpl->modelEntity);

    return utils::fromIgnitionVector(baseAngTarget);
}

std::array<double, 3> Model::baseWorldLinearAccelerationTarget() const
{
    ignition::math::Vector3d& baseLinTarget = utils::getExistingComponentData<
        ignition::gazebo::components::BaseWorldLinearAccelerationTarget>(
        pImpl->ecm, pImpl->modelEntity);

    return utils::fromIgnitionVector(baseLinTarget);
}

std::array<double, 3> Model::baseWorldAngularAccelerationTarget() const
{
    ignition::math::Vector3d& baseAngTarget = utils::getExistingComponentData<
        ignition::gazebo::components::BaseWorldAngularAccelerationTarget>(
        pImpl->ecm, pImpl->modelEntity);

    return utils::fromIgnitionVector(baseAngTarget);
}

// ======================
// Implementation Methods
// ======================

std::vector<double> Model::Impl::getJointDataSerialized(
    const Model* model,
    const std::vector<std::string>& jointNames,
    std::function<double(JointPtr, const size_t)> getJointData)
{
    const std::vector<std::string> jointSerialization =
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
    std::function<bool(JointPtr, const double, const size_t)> setJointData)
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
        gymppError << "The size of the forces does not match the considered "
                      "joint's DOFs"
                   << std::endl;
        return false;
    }

    auto it = data.begin();

    for (auto& joint : model->joints(jointNames)) {
        for (size_t dof = 0; dof < joint->dofs(); ++dof) {
            if (!setJointData(joint, *it++, dof)) {
                gymppError << "Failed to set force of joint '" << joint->name()
                           << "'" << std::endl;
                return false;
            }
        }
    }
    assert(it == data.end());
    return true;
}
