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

#include "scenario/gazebo/Joint.h"
#include "scenario/gazebo/Log.h"
#include "scenario/gazebo/Model.h"
#include "scenario/gazebo/World.h"
#include "scenario/gazebo/components/HistoryOfAppliedJointForces.h"
#include "scenario/gazebo/components/JointAcceleration.h"
#include "scenario/gazebo/components/JointAccelerationTarget.h"
#include "scenario/gazebo/components/JointControlMode.h"
#include "scenario/gazebo/components/JointController.h"
#include "scenario/gazebo/components/JointControllerPeriod.h"
#include "scenario/gazebo/components/JointPID.h"
#include "scenario/gazebo/components/JointPositionTarget.h"
#include "scenario/gazebo/components/JointVelocityTarget.h"
#include "scenario/gazebo/components/Timestamp.h"
#include "scenario/gazebo/exceptions.h"
#include "scenario/gazebo/helpers.h"
#include "scenario/gazebo/utils.h"

#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/components/JointForce.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/gazebo/components/JointType.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/JointVelocityReset.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/math/PID.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>

#include <cassert>
#include <utility>

using namespace scenario::gazebo;
const ignition::math::PID DefaultPID(1, 0.1, 0.01, -1, 0, -1, 0, 0);

class Joint::Impl
{
public:
};

Joint::Joint()
    : pImpl{std::make_unique<Impl>()}
{}

Joint::~Joint() = default;

uint64_t Joint::id() const
{
    // Get the parent world
    const core::WorldPtr parentWorld = utils::getParentWorld(*this);
    assert(parentWorld);

    // Build a unique string identifier of this joint
    const std::string scopedJointName =
        parentWorld->name() + "::" + this->name(/*scoped=*/true);

    // Return the hashed string
    return std::hash<std::string>{}(scopedJointName);
}

bool Joint::initialize(const ignition::gazebo::Entity jointEntity,
                       ignition::gazebo::EntityComponentManager* ecm,
                       ignition::gazebo::EventManager* eventManager)
{
    if (jointEntity == ignition::gazebo::kNullEntity || !ecm || !eventManager) {
        sError << "Failed to initialize Joint" << std::endl;
        return false;
    }

    m_ecm = ecm;
    m_entity = jointEntity;
    m_eventManager = eventManager;

    if (this->dofs() > 1) {
        sError << "Joints with DoFs > 1 are not currently supported"
               << std::endl;
        return false;
    }

    return true;
}

bool Joint::createECMResources()
{
    sMessage << "  [" << m_entity << "] " << this->name() << std::endl;

    using namespace ignition::gazebo;

    const std::vector<double> zero(this->dofs(), 0.0);

    // Create required components
    m_ecm->CreateComponent(m_entity, components::JointForce(zero));
    m_ecm->CreateComponent(m_entity, components::JointPosition(zero));
    m_ecm->CreateComponent(m_entity, components::JointVelocity(zero));
    m_ecm->CreateComponent(m_entity, components::JointAcceleration(zero));
    m_ecm->CreateComponent(m_entity, components::JointPID(DefaultPID));
    m_ecm->CreateComponent(
        m_entity, components::JointControlMode(core::JointControlMode::Idle));

    return true;
}

bool Joint::insertJointPlugin(const std::string& libName,
                              const std::string& className,
                              const std::string& context)
{
    return utils::insertPluginToGazeboEntity(
        *this, libName, className, context);
}

bool Joint::resetPosition(const double position, size_t dof)
{
    if (dof >= this->dofs()) {
        sError << "Joint '" << this->name() << "' does not have DoF#" << dof
               << std::endl;
        return false;
    }

    auto& jointPositionReset = utils::getComponentData< //
        ignition::gazebo::components::JointPositionReset>(m_ecm, m_entity);

    if (jointPositionReset.size() != this->dofs()) {
        assert(jointPositionReset.size() == 0);
        jointPositionReset = std::vector<double>(this->dofs(), 0.0);
    }

    // Reset the PID
    auto& pid = utils::getExistingComponentData< //
        ignition::gazebo::components::JointPID>(m_ecm, m_entity);
    pid.Reset();

    jointPositionReset[dof] = position;
    return true;
}

bool Joint::resetVelocity(const double velocity, const size_t dof)
{
    if (dof >= this->dofs()) {
        sError << "Joint '" << this->name() << "' does not have DoF#" << dof
               << std::endl;
        return false;
    }

    auto& jointVelocityReset = utils::getComponentData< //
        ignition::gazebo::components::JointVelocityReset>(m_ecm, m_entity);

    if (jointVelocityReset.size() != this->dofs()) {
        assert(jointVelocityReset.size() == 0);
        jointVelocityReset = std::vector<double>(this->dofs(), 0.0);
    }

    // Reset the PID
    auto& pid = utils::getExistingComponentData< //
        ignition::gazebo::components::JointPID>(m_ecm, m_entity);
    pid.Reset();

    jointVelocityReset[dof] = velocity;
    return true;
}

bool Joint::reset(const double position, const double velocity, size_t dof)
{
    bool ok = true;

    // These methods also reset the PID
    ok = ok && this->resetPosition(position, dof);
    ok = ok && this->resetVelocity(velocity, dof);

    if (!ok) {
        sError << "Failed to reset state of joint '" << this->name() << "'"
               << std::endl;
        return false;
    }

    return true;
}

bool Joint::resetJointPosition(const std::vector<double>& position)
{
    if (position.size() != this->dofs()) {
        sError << "Wrong number of elements (joint_dofs=" << this->dofs() << ")"
               << std::endl;
        return false;
    }

    auto& jointPositionReset = utils::getComponentData< //
        ignition::gazebo::components::JointPositionReset>(m_ecm, m_entity);

    // Update the position
    jointPositionReset = position;

    // Reset the PID
    auto& pid = utils::getExistingComponentData< //
        ignition::gazebo::components::JointPID>(m_ecm, m_entity);
    pid.Reset();

    return true;
}

bool Joint::resetJointVelocity(const std::vector<double>& velocity)
{
    if (velocity.size() != this->dofs()) {
        sError << "Wrong number of elements (joint_dofs=" << this->dofs() << ")"
               << std::endl;
        return false;
    }

    auto& jointVelocityReset = utils::getComponentData< //
        ignition::gazebo::components::JointVelocityReset>(m_ecm, m_entity);

    // Update the velocity
    jointVelocityReset = velocity;

    // Reset the PID
    auto& pid = utils::getExistingComponentData< //
        ignition::gazebo::components::JointPID>(m_ecm, m_entity);
    pid.Reset();

    return true;
}

bool Joint::resetJoint(const std::vector<double>& position,
                       const std::vector<double>& velocity)
{
    bool ok = true;

    ok = ok && this->resetJointPosition(position);
    ok = ok && this->resetJointVelocity(velocity);

    if (!ok) {
        sError << "Failed to reset joint '" << this->name() << "'" << std::endl;
        return false;
    }

    return true;
}

bool Joint::setCoulombFriction(const double value)
{
    if (!utils::parentModelJustCreated(*this)) {
        sError << "The model has been already processed and its "
               << "parameters cannot be modified" << std::endl;
        return false;
    }

    switch (this->type()) {
        case core::JointType::Revolute:
        case core::JointType::Prismatic:
        case core::JointType::Ball: {
            sdf::JointAxis& axis = utils::getExistingComponentData< //
                ignition::gazebo::components::JointAxis>(m_ecm, m_entity);
            axis.SetFriction(value);
            return true;
        }
        case core::JointType::Fixed:
        case core::JointType::Invalid:
            sWarning << "Fixed and Invalid joints have no friction defined."
                     << std::endl;
            return false;
    }

    return false;
}

bool Joint::setViscousFriction(const double value)
{
    if (!utils::parentModelJustCreated(*this)) {
        sError << "The model has been already processed and its "
               << "parameters cannot be modified" << std::endl;
        return false;
    }

    switch (this->type()) {
        case core::JointType::Revolute:
        case core::JointType::Prismatic:
        case core::JointType::Ball: {
            sdf::JointAxis& axis = utils::getExistingComponentData< //
                ignition::gazebo::components::JointAxis>(m_ecm, m_entity);
            axis.SetDamping(value);
            return true;
        }
        case core::JointType::Fixed:
        case core::JointType::Invalid:
            sWarning << "Fixed and Invalid joints have no friction defined."
                     << std::endl;
            return false;
    }

    return false;
}

bool Joint::valid() const
{
    return this->validEntity();
}

size_t Joint::dofs() const
{
    switch (this->type()) {
        case core::JointType::Invalid:
            return 0;
        case core::JointType::Fixed:
            return 0;
        case core::JointType::Revolute:
            return 1;
        case core::JointType::Prismatic:
            return 1;
        case core::JointType::Ball:
            return 3;
    }

    assert(false);
    return 0;
}

std::string Joint::name(const bool scoped) const
{
    std::string jointName = utils::getExistingComponentData< //
        ignition::gazebo::components::Name>(m_ecm, m_entity);

    if (scoped) {
        jointName = utils::getParentModel(*this)->name() + "::" + jointName;
    }

    return jointName;
}

scenario::core::JointType Joint::type() const
{
    // Get the joint type
    const sdf::JointType sdfType = utils::getExistingComponentData< //
        ignition::gazebo::components::JointType>(m_ecm, m_entity);

    // Convert the joint type
    const core::JointType type = utils::fromSdf(sdfType);

    return type;
}

scenario::core::JointControlMode Joint::controlMode() const
{
    const core::JointControlMode mode = utils::getExistingComponentData<
        ignition::gazebo::components::JointControlMode>(m_ecm, m_entity);

    return mode;
}

bool Joint::setControlMode(const scenario::core::JointControlMode mode)
{
    if (mode == core::JointControlMode::PositionInterpolated) {
        sError << "PositionInterpolated not yet supported" << std::endl;
        return false;
    }

    // Insert the JointController plugin to the model if the control
    // mode is either Position or Velocity
    if (mode == core::JointControlMode::Position
        || mode == core::JointControlMode::Velocity
        || mode == core::JointControlMode::VelocityFollowerDart) {

        // Get the parent model
        const auto parentModel = utils::getParentModel(*this);

        if (!parentModel) {
            sError << "Failed to get the parent model of joint '"
                   << this->name() << "' for inserting the "
                   << "JointController" << std::endl;
            return false;
        }

        // Insert the plugin if the model does not have it already
        if (!m_ecm->EntityHasComponentType(
                parentModel->entity(),
                ignition::gazebo::components::JointController::typeId)) {

            sDebug << "Loading JointController plugin for model '"
                   << parentModel->name() << "'" << std::endl;

            // Load the JointController plugin
            if (!parentModel->insertModelPlugin(
                    "JointController",
                    "scenario::plugins::gazebo::JointController")) {
                sError << "Failed to insert JointController plugin for model '"
                       << parentModel->name() << "'" << std::endl;
                return false;
            }
        }
    }

    utils::setExistingComponentData<
        ignition::gazebo::components::JointControlMode>(m_ecm, m_entity, mode);

    // Delete the existing targets if they exist
    sDebug << "Deleting existing targets after changing control mode"
           << std::endl;
    m_ecm->RemoveComponent(
        m_entity, ignition::gazebo::components::JointPositionTarget::typeId);
    m_ecm->RemoveComponent(
        m_entity, ignition::gazebo::components::JointVelocityTarget::typeId);
    m_ecm->RemoveComponent(
        m_entity,
        ignition::gazebo::components::JointAccelerationTarget::typeId);
    m_ecm->RemoveComponent(
        m_entity, ignition::gazebo::components::JointVelocityCmd::typeId);
    m_ecm->RemoveComponent(m_entity,
                           ignition::gazebo::components::JointForceCmd::typeId);

    // Initialize the target as the current position / velocity
    switch (mode) {
        case core::JointControlMode::Position:
        case core::JointControlMode::PositionInterpolated:
            utils::setComponentData<
                ignition::gazebo::components::JointPositionTarget>(
                m_ecm, m_entity, this->jointPosition());
            break;
        case core::JointControlMode::Velocity:
        case core::JointControlMode::VelocityFollowerDart:
            utils::setComponentData<
                ignition::gazebo::components::JointVelocityTarget>(
                m_ecm, m_entity, this->jointVelocity());
            break;
        case core::JointControlMode::Idle:
        case core::JointControlMode::Force:
            utils::setComponentData<
                ignition::gazebo::components::JointForceCmd>(
                m_ecm, m_entity, std::vector<double>(this->dofs(), 0.0));
            break;
        case core::JointControlMode::Invalid:
            sError << "You cannot set the Invalid control mode" << std::endl;
            return false;
    }

    // Reset the PID
    auto& pid = utils::getExistingComponentData< //
        ignition::gazebo::components::JointPID>(m_ecm, m_entity);
    pid.Reset();

    return true;
}

double Joint::controllerPeriod() const
{
    auto duration = utils::getExistingComponentData< //
        ignition::gazebo::components::JointControllerPeriod>(
        m_ecm, m_ecm->ParentEntity(m_entity));

    return utils::steadyClockDurationToDouble(duration);
}

scenario::core::PID Joint::pid() const
{
    const ignition::math::PID& pid = utils::getExistingComponentData< //
        ignition::gazebo::components::JointPID>(m_ecm, m_entity);

    return utils::fromIgnitionPID(pid);
}

bool Joint::setPID(const scenario::core::PID& pid)
{
    auto eqOp = [](const ignition::math::PID& a,
                   const ignition::math::PID& b) -> bool {
        bool equal = true;
        const double epsilon = std::numeric_limits<double>::epsilon();

        equal = equal && std::abs(a.PGain() - b.PGain()) > epsilon;
        equal = equal && std::abs(a.IGain() - b.IGain()) > epsilon;
        equal = equal && std::abs(a.DGain() - b.DGain()) > epsilon;
        equal = equal && std::abs(a.IMin() - b.IMin()) > epsilon;
        equal = equal && std::abs(a.IMax() - b.IMax()) > epsilon;
        equal = equal && std::abs(a.CmdMin() - b.CmdMin()) > epsilon;
        equal = equal && std::abs(a.CmdMax() - b.CmdMax()) > epsilon;
        equal = equal && std::abs(a.CmdOffset() - b.CmdOffset()) > epsilon;
        return equal;
    };

    if (this->dofs() > 1) {
        sError << "Setting PIDs of joints with more than 1 DoF is not "
               << "currently supported" << std::endl;
        return false;
    }

    scenario::core::PID pidParams = pid;
    const auto minForce = -this->maxGeneralizedForce(0);
    const auto maxForce = this->maxGeneralizedForce(0);

    if (pidParams.cmdMin < minForce || pidParams.cmdMax > maxForce) {
        sWarning << "The output limits of the PID are less limiting than "
                 << "the maximum force that can be exerted on the joint. "
                 << "Ignoring the specified PID limits." << std::endl;
        pidParams.cmdMin = minForce;
        pidParams.cmdMax = maxForce;
    }

    // Create the new PID
    const ignition::math::PID& pidIgnitionMath =
        utils::toIgnitionPID(pidParams);

    // Store the new PID in the ECM
    utils::setExistingComponentData<ignition::gazebo::components::JointPID,
                                    ignition::math::PID>(
        m_ecm, m_entity, pidIgnitionMath, eqOp);

    return true;
}

bool Joint::historyOfAppliedJointForcesEnabled() const
{
    return m_ecm->EntityHasComponentType(
        m_entity,
        ignition::gazebo::components::HistoryOfAppliedJointForces::typeId);
}

bool Joint::enableHistoryOfAppliedJointForces(const bool enable,
                                              const size_t maxHistorySize)
{
    if (enable) {
        // If the component already exists, its value is not overridden
        utils::getComponent<
            ignition::gazebo::components::HistoryOfAppliedJointForces>(
            m_ecm, m_entity, utils::FixedSizeQueue(maxHistorySize));
    }
    else {
        m_ecm->RemoveComponent(
            m_entity,
            ignition::gazebo::components::HistoryOfAppliedJointForces::typeId);
    }

    return true;
}

std::vector<double> Joint::historyOfAppliedJointForces() const
{
    if (!this->historyOfAppliedJointForcesEnabled()) {
        return {};
    }

    const auto& fixedSizeQueue = utils::getExistingComponentData<
        ignition::gazebo::components::HistoryOfAppliedJointForces>(m_ecm,
                                                                   m_entity);

    return fixedSizeQueue.toStdVector();
}

double Joint::coulombFriction() const
{
    switch (this->type()) {
        case core::JointType::Revolute:
        case core::JointType::Prismatic:
        case core::JointType::Ball: {
            const sdf::JointAxis& axis = utils::getExistingComponentData< //
                ignition::gazebo::components::JointAxis>(m_ecm, m_entity);
            return axis.Friction();
        }
        case core::JointType::Fixed:
        case core::JointType::Invalid:
            sWarning << "Fixed and Invalid joints have no friction defined."
                     << std::endl;
            return 0.0;
    }

    assert(false);
    return 0.0;
}

double Joint::viscousFriction() const
{
    switch (this->type()) {
        case core::JointType::Revolute:
        case core::JointType::Prismatic:
        case core::JointType::Ball: {
            const sdf::JointAxis& axis = utils::getExistingComponentData< //
                ignition::gazebo::components::JointAxis>(m_ecm, m_entity);
            return axis.Damping();
        }
        case core::JointType::Fixed:
        case core::JointType::Invalid:
            sWarning << "Fixed and Invalid joints have no friction defined."
                     << std::endl;
            return 0.0;
    }

    assert(false);
    return 0.0;
}

scenario::core::Limit Joint::positionLimit(const size_t dof) const
{
    if (dof >= this->dofs()) {
        throw exceptions::DOFMismatch(this->dofs(), dof, this->name());
    }

    const auto& jointLimit = this->jointPositionLimit();
    assert(dof < jointLimit.min.size());
    assert(dof < jointLimit.max.size());

    return core::Limit(jointLimit.min[dof], jointLimit.max[dof]);
}

scenario::core::Limit Joint::velocityLimit(const size_t dof) const
{
    if (dof >= this->dofs()) {
        throw exceptions::DOFMismatch(this->dofs(), dof, this->name());
    }

    const auto& jointLimit = this->jointVelocityLimit();
    assert(dof < jointLimit.min.size());
    assert(dof < jointLimit.max.size());

    return core::Limit(jointLimit.min[dof], jointLimit.max[dof]);
}

bool Joint::setVelocityLimit(const double maxVelocity, const size_t dof)
{
    if (dof >= this->dofs()) {
        throw exceptions::DOFMismatch(this->dofs(), dof, this->name());
    }
    auto velocityLimit = this->jointVelocityLimit();
    velocityLimit.max[dof] = maxVelocity;
    return this->setJointVelocityLimit(velocityLimit.max);
}

double Joint::maxGeneralizedForce(const size_t dof) const
{
    if (dof >= this->dofs()) {
        throw exceptions::DOFMismatch(this->dofs(), dof, this->name());
    }

    const std::vector<double>& maxForce = this->jointMaxGeneralizedForce();
    return maxForce[dof];
}

bool Joint::setMaxGeneralizedForce(const double maxForce, const size_t dof)
{
    if (dof >= this->dofs()) {
        sError << "Joint '" << this->name() << "' does not have DoF#" << dof
               << std::endl;
        return false;
    }

    auto maxGeneralizedForce = this->jointMaxGeneralizedForce();
    maxGeneralizedForce[dof] = maxForce;
    return this->setJointMaxGeneralizedForce(maxGeneralizedForce);
}

double Joint::position(const size_t dof) const
{
    if (dof >= this->dofs()) {
        throw exceptions::DOFMismatch(this->dofs(), dof, this->name());
    }

    const std::vector<double>& position = this->jointPosition();
    return position[dof];
}

double Joint::velocity(const size_t dof) const
{
    if (dof >= this->dofs()) {
        throw exceptions::DOFMismatch(this->dofs(), dof, this->name());
    }

    const std::vector<double>& velocity = this->jointVelocity();
    return velocity[dof];
}

double Joint::acceleration(const size_t dof) const
{
    if (dof >= this->dofs()) {
        throw exceptions::DOFMismatch(this->dofs(), dof, this->name());
    }

    const std::vector<double>& acceleration = this->jointAcceleration();
    return acceleration[dof];
}

double Joint::generalizedForce(const size_t dof) const
{
    if (dof >= this->dofs()) {
        throw exceptions::DOFMismatch(this->dofs(), dof, this->name());
    }

    const std::vector<double>& force = this->jointGeneralizedForce();
    return force[dof];
}

bool Joint::setPositionTarget(const double position, const size_t dof)
{
    const std::vector<core::JointControlMode> allowedControlModes = {
        core::JointControlMode::Position,
        core::JointControlMode::PositionInterpolated,
        core::JointControlMode::Idle,
        core::JointControlMode::Force};

    auto it = std::find(allowedControlModes.begin(),
                        allowedControlModes.end(),
                        this->controlMode());

    if (it == allowedControlModes.end()) {
        sError << "The active joint control mode does not accept a "
               << "position target" << std::endl;
        return false;
    }

    if (dof >= this->dofs()) {
        sError << "Joint '" << this->name() << "' does not have DoF#" << dof
               << std::endl;
        return false;
    }

    auto& jointPositionTarget = utils::getComponentData< //
        ignition::gazebo::components::JointPositionTarget>(m_ecm, m_entity);

    if (jointPositionTarget.size() != this->dofs()) {
        assert(jointPositionTarget.size() == 0);
        jointPositionTarget = std::vector<double>(this->dofs(), 0.0);
    }

    jointPositionTarget[dof] = position;
    return true;
}

bool Joint::setVelocityTarget(const double velocity, const size_t dof)
{
    if (!(this->controlMode() == core::JointControlMode::Velocity
          || this->controlMode() == core::JointControlMode::VelocityFollowerDart
          || this->controlMode() == core::JointControlMode::Force)) {
        sError << "The active joint control mode does not accept a "
               << "velocity target" << std::endl;
        return false;
    }

    if (dof >= this->dofs()) {
        sError << "Joint '" << this->name() << "' does not have DoF#" << dof
               << std::endl;
        return false;
    }

    auto& jointVelocityTarget = utils::getComponentData< //
        ignition::gazebo::components::JointVelocityTarget>(m_ecm, m_entity);

    if (jointVelocityTarget.size() != this->dofs()) {
        assert(jointVelocityTarget.size() == 0);
        jointVelocityTarget = std::vector<double>(this->dofs(), 0.0);
    }

    jointVelocityTarget[dof] = velocity;
    return true;
}

bool Joint::setAccelerationTarget(const double acceleration, const size_t dof)
{
    if (!(this->controlMode() == core::JointControlMode::Idle
          || this->controlMode() == core::JointControlMode::Force)) {
        sError << "The active joint control mode does not accept an "
               << "acceleration target" << std::endl;
        return false;
    }

    if (dof >= this->dofs()) {
        sError << "Joint '" << this->name() << "' does not have DoF#" << dof
               << std::endl;
        return false;
    }

    auto& jointAccelerationTarget = utils::getComponentData< //
        ignition::gazebo::components::JointAccelerationTarget>(m_ecm, m_entity);

    if (jointAccelerationTarget.size() != this->dofs()) {
        assert(jointAccelerationTarget.size() == 0);
        jointAccelerationTarget = std::vector<double>(this->dofs(), 0.0);
    }

    jointAccelerationTarget[dof] = acceleration;
    return true;
}

bool Joint::setGeneralizedForceTarget(const double force, const size_t dof)
{
    const std::vector<core::JointControlMode> allowedControlModes = {
        core::JointControlMode::Force,
        core::JointControlMode::Position,
        core::JointControlMode::PositionInterpolated,
        core::JointControlMode::Velocity};

    auto it = std::find(allowedControlModes.begin(),
                        allowedControlModes.end(),
                        this->controlMode());

    if (it == allowedControlModes.end()) {
        sError << "The active joint control mode does not accept a force "
               << "target" << std::endl;
        return false;
    }

    if (dof >= this->dofs()) {
        sError << "Joint '" << this->name() << "' does not have DoF#" << dof
               << std::endl;
        return false;
    }

    auto& jointForce = utils::getComponentData< //
        ignition::gazebo::components::JointForceCmd>(m_ecm, m_entity);

    if (jointForce.size() != this->dofs()) {
        assert(jointForce.size() == 0);
        jointForce = std::vector<double>(this->dofs(), 0.0);
    }

    if (std::abs(force) > this->maxGeneralizedForce(dof)) {
        sWarning << "The force target is higher than the limit. "
                 << "The physics engine might clip it." << std::endl;
    }

    // Set the component data
    jointForce[dof] = force;

    return true;
}

double Joint::positionTarget(const size_t dof) const
{
    if (dof >= this->dofs()) {
        throw exceptions::DOFMismatch(this->dofs(), dof, this->name());
    }

    const std::vector<double>& positionTarget = this->jointPositionTarget();
    return positionTarget[dof];
}

double Joint::velocityTarget(const size_t dof) const
{
    if (dof >= this->dofs()) {
        throw exceptions::DOFMismatch(this->dofs(), dof, this->name());
    }

    const std::vector<double>& velocityTarget = this->jointVelocityTarget();
    return velocityTarget[dof];
}

double Joint::accelerationTarget(const size_t dof) const
{
    if (dof >= this->dofs()) {
        throw exceptions::DOFMismatch(this->dofs(), dof, this->name());
    }

    const std::vector<double>& accelerationTarget =
        this->jointAccelerationTarget();
    return accelerationTarget[dof];
}

double Joint::generalizedForceTarget(const size_t dof) const
{
    if (dof >= this->dofs()) {
        throw exceptions::DOFMismatch(this->dofs(), dof, this->name());
    }

    const std::vector<double>& force = this->jointGeneralizedForceTarget();
    return force[dof];
}

scenario::core::JointLimit Joint::jointPositionLimit() const
{
    core::JointLimit jointLimit(this->dofs());

    switch (this->type()) {
        case core::JointType::Revolute:
        case core::JointType::Prismatic: {
            sdf::JointAxis& axis = utils::getExistingComponentData< //
                ignition::gazebo::components::JointAxis>(m_ecm, m_entity);
            jointLimit.min[0] = axis.Lower();
            jointLimit.max[0] = axis.Upper();
            break;
        }
        case core::JointType::Fixed:
            sWarning << "Fixed joints do not have DOFs, limits are not defined"
                     << std::endl;
            break;
        case core::JointType::Invalid:
        case core::JointType::Ball:
            sWarning << "Type of Joint '" << this->name() << "' has no limits"
                     << std::endl;
            break;
    }

    return jointLimit;
}

scenario::core::JointLimit Joint::jointVelocityLimit() const
{
    core::JointLimit jointLimit(this->dofs());

    switch (this->type()) {
        case core::JointType::Revolute:
        case core::JointType::Prismatic: {
            sdf::JointAxis& axis = utils::getExistingComponentData< //
                ignition::gazebo::components::JointAxis>(m_ecm, m_entity);
            jointLimit.min[0] = -axis.MaxVelocity();
            jointLimit.max[0] = axis.MaxVelocity();
            break;
        }
        case core::JointType::Fixed:
            sWarning << "Fixed joints do not have DOFs, limits are not defined"
                     << std::endl;
            break;
        case core::JointType::Invalid:
        case core::JointType::Ball:
            sWarning << "Type of Joint '" << this->name() << "' has no limits"
                     << std::endl;
            break;
    }

    return jointLimit;
}

bool Joint::setJointVelocityLimit(const std::vector<double>& maxVelocity)
{
    if (!utils::parentModelJustCreated(*this)) {
        sError << "The model has been already processed and its "
               << "parameters cannot be modified" << std::endl;
        return false;
    }

    if (maxVelocity.size() != this->dofs()) {
        sError << "Wrong number of elements (joint_dofs=" << this->dofs() << ")"
               << std::endl;
        return false;
    }

    switch (this->type()) {
        case core::JointType::Revolute:
        case core::JointType::Prismatic: {
            sdf::JointAxis& axis = utils::getExistingComponentData< //
                ignition::gazebo::components::JointAxis>(m_ecm, m_entity);
            axis.SetMaxVelocity(maxVelocity[0]);
            return true;
        }
        case core::JointType::Ball: {
            const auto maxVelocity0 = maxVelocity[0];

            for (const auto max : maxVelocity) {
                if (max != maxVelocity0) {
                    sWarning << "Setting different velocity limits for each "
                             << "DOF is not supported. "
                             << "Using the limit of the first DOF."
                             << std::endl;
                    break;
                }
            }

            sdf::JointAxis& axis = utils::getExistingComponentData< //
                ignition::gazebo::components::JointAxis>(m_ecm, m_entity);
            axis.SetMaxVelocity(maxVelocity0);
            return true;
        }
        case core::JointType::Fixed:
        case core::JointType::Invalid:
            sWarning << "Fixed and Invalid joints have no friction defined."
                     << std::endl;
            return false;
    }

    return false;
}

std::vector<double> Joint::jointMaxGeneralizedForce() const
{
    std::vector<double> maxGeneralizedForce;

    switch (this->type()) {
        case core::JointType::Revolute:
        case core::JointType::Prismatic: {
            const sdf::JointAxis& axis = utils::getExistingComponentData< //
                ignition::gazebo::components::JointAxis>(m_ecm, m_entity);
            maxGeneralizedForce = {axis.Effort()};
            break;
        }
        case core::JointType::Fixed:
        case core::JointType::Invalid:
        case core::JointType::Ball:
            sWarning << "Type of Joint '" << this->name()
                     << "' has no max effort defined" << std::endl;
            break;
    }

    return maxGeneralizedForce;
}

bool Joint::setJointMaxGeneralizedForce(const std::vector<double>& maxForce)
{
    if (!utils::parentModelJustCreated(*this)) {
        sError << "The model has been already processed and its "
               << "parameters cannot be modified" << std::endl;
        return false;
    }

    if (maxForce.size() != this->dofs()) {
        sError << "Wrong number of elements (joint_dofs=" << this->dofs() << ")"
               << std::endl;
        return false;
    }

    switch (this->type()) {
        case core::JointType::Revolute:
        case core::JointType::Prismatic:
        case core::JointType::Ball: {
            sdf::JointAxis& axis = utils::getExistingComponentData< //
                ignition::gazebo::components::JointAxis>(m_ecm, m_entity);
            assert(maxForce.size() == 1);
            axis.SetEffort(maxForce[0]);
            return true;
        }
        case core::JointType::Fixed:
        case core::JointType::Invalid:
            sWarning << "Fixed and Invalid joints have no maxim effort defined."
                     << std::endl;
            return false;
    }

    return false;
}

std::vector<double> Joint::jointPosition() const
{
    const std::vector<double>& jointPosition = utils::getExistingComponentData<
        ignition::gazebo::components::JointPosition>(m_ecm, m_entity);

    if (jointPosition.size() != this->dofs()) {
        throw exceptions::DOFMismatch(
            this->dofs(), jointPosition.size(), this->name());
    }

    return jointPosition;
}

std::vector<double> Joint::jointVelocity() const
{
    const std::vector<double>& jointVelocity = utils::getExistingComponentData<
        ignition::gazebo::components::JointVelocity>(m_ecm, m_entity);

    if (jointVelocity.size() != this->dofs()) {
        throw exceptions::DOFMismatch(
            this->dofs(), jointVelocity.size(), this->name());
    }

    return jointVelocity;
}

std::vector<double> Joint::jointAcceleration() const
{
    const std::vector<double>& jointAcceleration =
        utils::getExistingComponentData< //
            ignition::gazebo::components::JointAcceleration>(m_ecm, m_entity);

    if (jointAcceleration.size() != this->dofs()) {
        throw exceptions::DOFMismatch(
            this->dofs(), jointAcceleration.size(), this->name());
    }

    return jointAcceleration;
}

std::vector<double> Joint::jointGeneralizedForce() const
{
    const std::vector<double>& jointGeneralizedForce =
        utils::getExistingComponentData<
            ignition::gazebo::components::JointForce>(m_ecm, m_entity);

    if (jointGeneralizedForce.size() != this->dofs()) {
        throw exceptions::DOFMismatch(
            this->dofs(), jointGeneralizedForce.size(), this->name());
    }

    return jointGeneralizedForce;
}

bool Joint::setJointPositionTarget(const std::vector<double>& position)
{
    if (position.size() != this->dofs()) {
        sError << "Wrong number of elements (joint_dofs=" << this->dofs() << ")"
               << std::endl;
        return false;
    }

    auto& jointPositionTarget = utils::getComponentData< //
        ignition::gazebo::components::JointPositionTarget>(m_ecm, m_entity);

    jointPositionTarget = position;
    return true;
}

bool Joint::setJointVelocityTarget(const std::vector<double>& velocity)
{
    if (velocity.size() != this->dofs()) {
        sError << "Wrong number of elements (joint_dofs=" << this->dofs() << ")"
               << std::endl;
        return false;
    }

    auto& jointVelocityTarget = utils::getComponentData< //
        ignition::gazebo::components::JointVelocityTarget>(m_ecm, m_entity);

    jointVelocityTarget = velocity;
    return true;
}

bool Joint::setJointAccelerationTarget(const std::vector<double>& acceleration)
{
    if (acceleration.size() != this->dofs()) {
        sError << "Wrong number of elements (joint_dofs=" << this->dofs() << ")"
               << std::endl;
        return false;
    }

    auto& jointAccelerationTarget = utils::getComponentData< //
        ignition::gazebo::components::JointAccelerationTarget>(m_ecm, m_entity);

    jointAccelerationTarget = acceleration;
    return true;
}

bool Joint::setJointGeneralizedForceTarget(const std::vector<double>& force)
{
    if (force.size() != this->dofs()) {
        sError << "Wrong number of elements (joint_dofs=" << this->dofs() << ")"
               << std::endl;
        return false;
    }

    auto& jointForceTarget = utils::getComponentData< //
        ignition::gazebo::components::JointForceCmd>(m_ecm, m_entity);

    const std::vector<double>& maxForce = this->jointMaxGeneralizedForce();

    for (size_t dof = 0; dof < this->dofs(); ++dof) {
        if (std::abs(force[dof]) > maxForce[dof]) {
            sWarning << "The force target is higher than the limit. "
                     << "The physics engine might clip it." << std::endl;
        }
    }

    // Set the component data
    jointForceTarget = force;
    return true;
}

std::vector<double> Joint::jointPositionTarget() const
{
    const std::vector<double>& target = utils::getExistingComponentData<
        ignition::gazebo::components::JointPositionTarget>(m_ecm, m_entity);

    if (target.size() != this->dofs()) {
        throw exceptions::DOFMismatch(
            this->dofs(), target.size(), this->name());
    }

    return target;
}

std::vector<double> Joint::jointVelocityTarget() const
{
    const std::vector<double>& target = utils::getExistingComponentData<
        ignition::gazebo::components::JointVelocityTarget>(m_ecm, m_entity);

    if (target.size() != this->dofs()) {
        throw exceptions::DOFMismatch(
            this->dofs(), target.size(), this->name());
    }

    return target;
}

std::vector<double> Joint::jointAccelerationTarget() const
{
    const std::vector<double>& target = utils::getExistingComponentData<
        ignition::gazebo::components::JointAccelerationTarget>(m_ecm, m_entity);

    if (target.size() != this->dofs()) {
        throw exceptions::DOFMismatch(
            this->dofs(), target.size(), this->name());
    }

    return target;
}

std::vector<double> Joint::jointGeneralizedForceTarget() const
{
    const std::vector<double>& target = utils::getExistingComponentData< //
        ignition::gazebo::components::JointForceCmd>(m_ecm, m_entity);

    if (target.size() != this->dofs()) {
        throw exceptions::DOFMismatch(
            this->dofs(), target.size(), this->name());
    }

    return target;
}
