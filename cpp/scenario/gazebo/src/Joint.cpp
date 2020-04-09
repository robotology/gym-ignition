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
#include "scenario/gazebo/components/JointAccelerationTarget.h"
#include "scenario/gazebo/components/JointControlMode.h"
#include "scenario/gazebo/components/JointControllerPeriod.h"
#include "scenario/gazebo/components/JointPID.h"
#include "scenario/gazebo/components/JointPositionReset.h"
#include "scenario/gazebo/components/JointPositionTarget.h"
#include "scenario/gazebo/components/JointVelocityReset.h"
#include "scenario/gazebo/components/JointVelocityTarget.h"
#include "scenario/gazebo/components/MaxJointForce.h"
#include "scenario/gazebo/exceptions.h"
#include "scenario/gazebo/helpers.h"

#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/components/JointForce.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointType.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/math/PID.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>

#include <cassert>
#include <utility>

using namespace scenario::gazebo;
const ignition::math::PID DefaultPID(1, 0.1, 0.01, 1, -1, 10000, -10000);

class Joint::Impl
{
public:
    ignition::gazebo::EventManager* eventManager = nullptr;
    ignition::gazebo::EntityComponentManager* ecm = nullptr;

    ignition::gazebo::Entity jointEntity = ignition::gazebo::kNullEntity;
};

Joint::Joint()
    : pImpl{std::make_unique<Impl>()}
{}

uint64_t Joint::id() const
{
    // Get the parent world
    WorldPtr parentWorld = utils::getParentWorld(
        pImpl->ecm, pImpl->eventManager, pImpl->jointEntity);
    assert(parentWorld);

    // Get the parent model
    ModelPtr parentModel = utils::getParentModel(
        pImpl->ecm, pImpl->eventManager, pImpl->jointEntity);
    assert(parentModel);

    // Build a unique string identifier of this joint
    std::string scopedJointName =
        parentWorld->name() + "::" + parentModel->name() + "::" + this->name();

    // Return the hashed string
    return std::hash<std::string>{}(scopedJointName);
}

Joint::~Joint() = default;

bool Joint::initialize(const ignition::gazebo::Entity jointEntity,
                       ignition::gazebo::EntityComponentManager* ecm,
                       ignition::gazebo::EventManager* eventManager)
{
    if (jointEntity == ignition::gazebo::kNullEntity || !ecm || !eventManager) {
        gymppError << "Failed to initialize Joint" << std::endl;
        return false;
    }

    pImpl->ecm = ecm;
    pImpl->jointEntity = jointEntity;
    pImpl->eventManager = eventManager;

    if (this->dofs() > 1) {
        gymppError << "Joints with DoFs > 1 are not currently supported"
                   << std::endl;
        return false;
    }

    return true;
}

bool Joint::createECMResources()
{
    gymppMessage << "  [" << pImpl->jointEntity << "] " << this->name()
                 << std::endl;

    using namespace ignition::gazebo;

    const std::vector<double> zero(this->dofs(), 0.0);
    const std::vector<double> infinity( //
        this->dofs(),
        std::numeric_limits<double>::infinity());

    // Create required components
    pImpl->ecm->CreateComponent(pImpl->jointEntity,
                                components::JointForce(zero));
    pImpl->ecm->CreateComponent(pImpl->jointEntity,
                                components::JointPosition(zero));
    pImpl->ecm->CreateComponent(pImpl->jointEntity,
                                components::JointVelocity(zero));
    pImpl->ecm->CreateComponent(pImpl->jointEntity,
                                components::JointPID(DefaultPID));
    pImpl->ecm->CreateComponent(
        pImpl->jointEntity,
        components::JointControlMode(base::JointControlMode::Idle));
    pImpl->ecm->CreateComponent(pImpl->jointEntity,
                                components::MaxJointForce(infinity));

    return true;
}

bool Joint::historyOfAppliedJointForcesEnabled() const
{
    return pImpl->ecm->EntityHasComponentType(
        pImpl->jointEntity,
        ignition::gazebo::components::HistoryOfAppliedJointForces().TypeId());
}

bool Joint::enableHistoryOfAppliedJointForces(const bool enable,
                                              const size_t maxHistorySize)
{
    if (enable) {
        // If the component already exists, its value is not overridden
        utils::getComponent<
            ignition::gazebo::components::HistoryOfAppliedJointForces>(
            pImpl->ecm,
            pImpl->jointEntity,
            utils::FixedSizeQueue(maxHistorySize));
    }
    else {
        pImpl->ecm->RemoveComponent(
            pImpl->jointEntity,
            ignition::gazebo::components::HistoryOfAppliedJointForces()
                .TypeId());
    }

    return true;
}

std::vector<double> Joint::historyOfAppliedJointForces() const
{
    if (!this->historyOfAppliedJointForcesEnabled()) {
        return {};
    }

    auto fixedSizeQueue = utils::getExistingComponentData<
        ignition::gazebo::components::HistoryOfAppliedJointForces>(
        pImpl->ecm, pImpl->jointEntity);

    return fixedSizeQueue.toStdVector();
}

size_t Joint::dofs() const
{
    switch (this->type()) {
        case base::JointType::Invalid:
            return 0;
        case base::JointType::Fixed:
            return 0;
        case base::JointType::Revolute:
            return 1;
        case base::JointType::Prismatic:
            return 1;
        case base::JointType::Ball:
            return 3;
    }

    assert(false);
    return 0;
}

std::string Joint::name() const
{
    std::string name = utils::getExistingComponentData< //
        ignition::gazebo::components::Name>(pImpl->ecm, pImpl->jointEntity);

    return name;
}

scenario::base::JointType Joint::type() const
{
    // Get the joint type
    sdf::JointType sdfType = utils::getExistingComponentData< //
        ignition::gazebo::components::JointType>(pImpl->ecm,
                                                 pImpl->jointEntity);

    // Convert the joint type
    base::JointType type = utils::fromSdf(sdfType);

    return type;
}

scenario::base::JointControlMode Joint::controlMode() const
{
    base::JointControlMode jointControlMode = utils::getExistingComponentData<
        ignition::gazebo::components::JointControlMode>(pImpl->ecm,
                                                        pImpl->jointEntity);

    return jointControlMode;
}

bool Joint::setControlMode(const scenario::base::JointControlMode mode)
{
    if (mode == base::JointControlMode::PositionInterpolated) {
        gymppError << "PositionInterpolated not yet supported" << std::endl;
        return false;
    }

    utils::setExistingComponentData<
        ignition::gazebo::components::JointControlMode>(
        pImpl->ecm, pImpl->jointEntity, mode);

    // Delete the existing targets if they exist
    gymppDebug << "Deleting existing position and velocity targets after "
               << "changing control mode" << std::endl;
    pImpl->ecm->RemoveComponent(
        pImpl->jointEntity,
        ignition::gazebo::components::JointPositionTarget().TypeId());
    pImpl->ecm->RemoveComponent(
        pImpl->jointEntity,
        ignition::gazebo::components::JointVelocityTarget().TypeId());

    // Initialize the target as the current position / velocity
    switch (mode) {
        case base::JointControlMode::Position:
        case base::JointControlMode::PositionInterpolated:
            pImpl->ecm->CreateComponent(
                pImpl->jointEntity,
                ignition::gazebo::components::JointPositionTarget(
                    this->jointPosition()));
            break;
        case base::JointControlMode::Velocity:
            pImpl->ecm->CreateComponent(
                pImpl->jointEntity,
                ignition::gazebo::components::JointVelocityTarget(
                    this->jointVelocity()));
            break;
        case base::JointControlMode::Idle:
        case base::JointControlMode::Force:
            break;
    }

    // Get the PID
    ignition::math::PID& pid = utils::getExistingComponentData< //
        ignition::gazebo::components::JointPID>(pImpl->ecm, pImpl->jointEntity);

    // Reset the PID
    pid.Reset();

    return true;
}

double Joint::controllerPeriod() const
{
    auto duration = utils::getExistingComponentData< //
        ignition::gazebo::components::JointControllerPeriod>(
        pImpl->ecm, pImpl->ecm->ParentEntity(pImpl->jointEntity));

    return utils::steadyClockDurationToDouble(duration);
}

scenario::base::PID Joint::pid() const
{
    const ignition::math::PID& pid = utils::getExistingComponentData< //
        ignition::gazebo::components::JointPID>(pImpl->ecm, pImpl->jointEntity);

    return utils::fromIgnitionPID(pid);
}

bool Joint::setPID(const scenario::base::PID& pid)
{
    auto eqOp = [](const ignition::math::PID& a,
                   const ignition::math::PID& b) -> bool {
        bool equal = true;
        double epsilon = std::numeric_limits<double>::epsilon();

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
        gymppError << "Setting PIDs of joints with more than 1 DoF is not "
                   << "currently supported" << std::endl;
        return false;
    }

    scenario::base::PID pidParams = pid;
    auto minForce = -this->maxGeneralizedForce(0);
    auto maxForce = this->maxGeneralizedForce(0);

    if (pidParams.cmdMin < minForce || pidParams.cmdMax > maxForce) {
        gymppWarning << "The output limits of the PID are less limiting than "
                     << "the maximum force that can be exerted on the joint. "
                     << "Ignoring the specified PID limits." << std::endl;
        pidParams.cmdMin = minForce;
        pidParams.cmdMax = maxForce;
    }

    // Create the new PID
    ignition::math::PID pidIgnitionMath = utils::toIgnitionPID(pidParams);

    // Store the new PID in the ECM
    utils::setExistingComponentData<ignition::gazebo::components::JointPID,
                                    ignition::math::PID>(
        pImpl->ecm, pImpl->jointEntity, pidIgnitionMath, eqOp);

    return true;
}

scenario::base::Limit Joint::positionLimit(const size_t dof) const
{
    if (dof >= this->dofs()) {
        gymppError << "Joint '" << this->name() << "' does not have DoF#" << dof
                   << std::endl;
        return base::Limit(0, 0);
    }

    auto jointLimit = this->jointPositionLimit();
    assert(dof < jointLimit.min.size());
    assert(dof < jointLimit.max.size());

    return base::Limit(jointLimit.min[dof], jointLimit.max[dof]);
}

double Joint::maxGeneralizedForce(const size_t dof) const
{
    std::vector<double> maxForce = this->jointMaxGeneralizedForce();
    return maxForce[dof];
}

bool Joint::setMaxGeneralizedForce(const double maxForce, const size_t dof)
{
    if (dof >= this->dofs()) {
        gymppError << "Joint '" << this->name() << "' does not have DoF#" << dof
                   << std::endl;
        return false;
    }

    auto& maxJointForce = utils::getComponentData< //
        ignition::gazebo::components::MaxJointForce>(pImpl->ecm,
                                                     pImpl->jointEntity);

    if (maxJointForce.size() != this->dofs()) {
        assert(maxJointForce.size() == 0);
        maxJointForce = std::vector<double>(this->dofs(), 0.0);
    }

    maxJointForce[dof] = maxForce;
    return true;
}

double Joint::position(const size_t dof) const
{
    std::vector<double> position = this->jointPosition();
    return position[dof];
}

double Joint::velocity(const size_t dof) const
{
    std::vector<double> velocity = this->jointVelocity();
    return velocity[dof];
}

bool Joint::setPositionTarget(const double position, const size_t dof)
{
    const std::vector<base::JointControlMode> allowedControlModes = {
        base::JointControlMode::Position,
        base::JointControlMode::PositionInterpolated,
        base::JointControlMode::Idle,
        base::JointControlMode::Force};

    auto it = std::find(allowedControlModes.begin(),
                        allowedControlModes.end(),
                        this->controlMode());

    if (it == allowedControlModes.end()) {
        gymppError
            << "The active joint control mode does not accept a position target"
            << std::endl;
        return false;
    }

    if (dof >= this->dofs()) {
        gymppError << "Joint '" << this->name() << "' does not have DoF#" << dof
                   << std::endl;
        return false;
    }

    auto& jointPositionTarget = utils::getComponentData< //
        ignition::gazebo::components::JointPositionTarget>(pImpl->ecm,
                                                           pImpl->jointEntity);

    if (jointPositionTarget.size() != this->dofs()) {
        assert(jointPositionTarget.size() == 0);
        jointPositionTarget = std::vector<double>(this->dofs(), 0.0);
    }

    jointPositionTarget[dof] = position;
    return true;
}

bool Joint::setVelocityTarget(const double velocity, const size_t dof)
{
    if (!(this->controlMode() == base::JointControlMode::Velocity
          || this->controlMode() == base::JointControlMode::Idle
          || this->controlMode() == base::JointControlMode::Force)) {
        gymppError
            << "The active joint control mode does not accept a velocity target"
            << std::endl;
        return false;
    }

    if (dof >= this->dofs()) {
        gymppError << "Joint '" << this->name() << "' does not have DoF#" << dof
                   << std::endl;
        return false;
    }

    auto& jointVelocityTarget = utils::getComponentData< //
        ignition::gazebo::components::JointVelocityTarget>(pImpl->ecm,
                                                           pImpl->jointEntity);

    if (jointVelocityTarget.size() != this->dofs()) {
        assert(jointVelocityTarget.size() == 0);
        jointVelocityTarget = std::vector<double>(this->dofs(), 0.0);
    }

    jointVelocityTarget[dof] = velocity;
    return true;
}

bool Joint::setAccelerationTarget(const double acceleration, const size_t dof)
{
    if (!(this->controlMode() == base::JointControlMode::Idle
          || this->controlMode() == base::JointControlMode::Force)) {
        gymppError
            << "The active joint control mode does not accept a velocity target"
            << std::endl;
        return false;
    }

    if (dof >= this->dofs()) {
        gymppError << "Joint '" << this->name() << "' does not have DoF#" << dof
                   << std::endl;
        return false;
    }

    auto& jointAccelerationTarget = utils::getComponentData< //
        ignition::gazebo::components::JointAccelerationTarget>(
        pImpl->ecm, pImpl->jointEntity);

    if (jointAccelerationTarget.size() != this->dofs()) {
        assert(jointAccelerationTarget.size() == 0);
        jointAccelerationTarget = std::vector<double>(this->dofs(), 0.0);
    }

    jointAccelerationTarget[dof] = acceleration;
    return true;
}

bool Joint::setGeneralizedForceTarget(const double force, const size_t dof)
{
    if (this->controlMode() != base::JointControlMode::Force) {
        gymppError << "The active joint control mode does not accept a force"
                   << std::endl;
        return false;
    }

    if (dof >= this->dofs()) {
        gymppError << "Joint '" << this->name() << "' does not have DoF#" << dof
                   << std::endl;
        return false;
    }

    auto& jointForce = utils::getComponentData< //
        ignition::gazebo::components::JointForceCmd>(pImpl->ecm,
                                                     pImpl->jointEntity);

    if (jointForce.size() != this->dofs()) {
        assert(jointForce.size() == 0);
        jointForce = std::vector<double>(this->dofs(), 0.0);
    }

    double forceClipped = force;
    double maxForce = this->maxGeneralizedForce(dof);

    forceClipped = std::min(forceClipped, maxForce);
    forceClipped = std::max(forceClipped, -maxForce);

    jointForce[dof] = forceClipped;
    return true;
}

double Joint::positionTarget(const size_t dof) const
{
    std::vector<double> positionTarget = this->jointPositionTarget();
    return positionTarget[dof];
}

double Joint::velocityTarget(const size_t dof) const
{
    std::vector<double> velocityTarget = this->jointVelocityTarget();
    return velocityTarget[dof];
}

double Joint::accelerationTarget(const size_t dof) const
{
    std::vector<double> accelerationTarget = this->jointAccelerationTarget();
    return accelerationTarget[dof];
}

double Joint::generalizedForceTarget(const size_t dof) const
{
    std::vector<double> force = this->jointGeneralizedForceTarget();
    return force[dof];
}

bool Joint::resetPosition(const double position, size_t dof)
{
    if (dof >= this->dofs()) {
        gymppError << "Joint '" << this->name() << "' does not have DoF#" << dof
                   << std::endl;
        return false;
    }

    auto& jointPositionReset = utils::getComponentData< //
        ignition::gazebo::components::JointPositionReset>(pImpl->ecm,
                                                          pImpl->jointEntity);

    if (jointPositionReset.size() != this->dofs()) {
        assert(jointPositionReset.size() == 0);
        jointPositionReset = std::vector<double>(this->dofs(), 0.0);
    }

    jointPositionReset[dof] = position;
    return true;
}

bool Joint::resetVelocity(const double velocity, const size_t dof)
{
    if (dof >= this->dofs()) {
        gymppError << "Joint '" << this->name() << " does not have DoF#" << dof
                   << std::endl;
        return false;
    }

    auto& jointVelocityReset = utils::getComponentData< //
        ignition::gazebo::components::JointVelocityReset>(pImpl->ecm,
                                                          pImpl->jointEntity);

    if (jointVelocityReset.size() != this->dofs()) {
        assert(jointVelocityReset.size() == 0);
        jointVelocityReset = std::vector<double>(this->dofs(), 0.0);
    }

    jointVelocityReset[dof] = velocity;
    return true;
}

bool Joint::reset(const double position, const double velocity, size_t dof)
{
    bool ok = true;

    ok = ok && this->resetPosition(position, dof);
    ok = ok && this->resetVelocity(velocity, dof);

    if (!ok) {
        gymppError << "Failed to reset state of joint '" << this->name() << "'"
                   << std::endl;
        return false;
    }

    // Reset the PID
    auto JointPIDComponent = utils::getExistingComponent< //
        ignition::gazebo::components::JointPID>(pImpl->ecm, pImpl->jointEntity);
    JointPIDComponent->Data().Reset();

    return true;
}

scenario::base::JointLimit Joint::jointPositionLimit() const
{
    base::JointLimit jointLimit(this->dofs());

    switch (this->type()) {
        case base::JointType::Revolute:
        case base::JointType::Prismatic: {
            sdf::JointAxis& axis = utils::getExistingComponentData< //
                ignition::gazebo::components::JointAxis>(pImpl->ecm,
                                                         pImpl->jointEntity);
            jointLimit.min[0] = axis.Lower();
            jointLimit.max[0] = axis.Upper();
            break;
        }
        case base::JointType::Fixed:
            gymppWarning
                << "Fixed joints do not have DOFs, limits are not defined"
                << std::endl;
            break;
        case base::JointType::Invalid:
        case base::JointType::Ball:
            gymppWarning << "Type of Joint '" << this->name()
                         << "' has no limits" << std::endl;
            break;
    }

    return jointLimit;
}

std::vector<double> Joint::jointMaxGeneralizedForce() const
{
    std::vector<double>& maxJointForce = utils::getExistingComponentData< //
        ignition::gazebo::components::MaxJointForce>(pImpl->ecm,
                                                     pImpl->jointEntity);

    return maxJointForce;
}

bool Joint::setJointMaxGeneralizedForce(const std::vector<double>& maxForce)
{
    if (maxForce.size() != this->dofs()) {
        gymppError << "Wrong number of elements (joint_dofs=" << this->dofs()
                   << ")" << std::endl;
        return false;
    }

    auto& maxJointForce = utils::getComponentData< //
        ignition::gazebo::components::MaxJointForce>(pImpl->ecm,
                                                     pImpl->jointEntity);

    maxJointForce = maxForce;
    return true;
}

std::vector<double> Joint::jointPosition() const
{
    std::vector<double>& jointPosition = utils::getExistingComponentData< //
        ignition::gazebo::components::JointPosition>(pImpl->ecm,
                                                     pImpl->jointEntity);

    if (jointPosition.size() != this->dofs()) {
        throw exceptions::DOFMismatch(
            this->dofs(), jointPosition.size(), this->name());
    }

    return jointPosition;
}

std::vector<double> Joint::jointVelocity() const
{
    std::vector<double>& jointVelocity = utils::getExistingComponentData< //
        ignition::gazebo::components::JointVelocity>(pImpl->ecm,
                                                     pImpl->jointEntity);

    if (jointVelocity.size() != this->dofs()) {
        throw exceptions::DOFMismatch(
            this->dofs(), jointVelocity.size(), this->name());
    }

    return jointVelocity;
}

bool Joint::setJointPositionTarget(const std::vector<double>& position)
{
    if (position.size() != this->dofs()) {
        gymppError << "Wrong number of elements (joint_dofs=" << this->dofs()
                   << ")" << std::endl;
        return false;
    }

    auto& jointPositionTarget = utils::getComponentData< //
        ignition::gazebo::components::JointPositionTarget>(pImpl->ecm,
                                                           pImpl->jointEntity);

    jointPositionTarget = position;
    return true;
}

bool Joint::setJointVelocityTarget(const std::vector<double>& velocity)
{
    if (velocity.size() != this->dofs()) {
        gymppError << "Wrong number of elements (joint_dofs=" << this->dofs()
                   << ")" << std::endl;
        return false;
    }

    auto& jointVelocityTarget = utils::getComponentData< //
        ignition::gazebo::components::JointVelocityTarget>(pImpl->ecm,
                                                           pImpl->jointEntity);

    jointVelocityTarget = velocity;
    return true;
}

bool Joint::setJointAccelerationTarget(const std::vector<double>& acceleration)
{
    if (acceleration.size() != this->dofs()) {
        gymppError << "Wrong number of elements (joint_dofs=" << this->dofs()
                   << ")" << std::endl;
        return false;
    }

    auto& jointAccelerationTarget = utils::getComponentData< //
        ignition::gazebo::components::JointAccelerationTarget>(
        pImpl->ecm, pImpl->jointEntity);

    jointAccelerationTarget = acceleration;
    return true;
}

bool Joint::setJointGeneralizedForceTarget(const std::vector<double>& force)
{
    if (force.size() != this->dofs()) {
        gymppError << "Wrong number of elements (joint_dofs=" << this->dofs()
                   << ")" << std::endl;
        return false;
    }

    auto& jointForceTarget = utils::getComponentData< //
        ignition::gazebo::components::JointForceCmd>(pImpl->ecm,
                                                     pImpl->jointEntity);

    std::vector<double> clippedForce = std::move(force);
    const std::vector<double> maxForce = this->jointMaxGeneralizedForce();

    for (size_t dof = 0; dof < this->dofs(); ++dof) {
        clippedForce[dof] = std::min(clippedForce[dof], maxForce[dof]);
        clippedForce[dof] = std::max(clippedForce[dof], -maxForce[dof]);
    }

    jointForceTarget = std::move(clippedForce);
    return true;
}

std::vector<double> Joint::jointPositionTarget() const
{
    std::vector<double>& jointPositionTarget = utils::getExistingComponentData<
        ignition::gazebo::components::JointPositionTarget>(pImpl->ecm,
                                                           pImpl->jointEntity);

    if (jointPositionTarget.size() != this->dofs()) {
        throw exceptions::DOFMismatch(
            this->dofs(), jointPositionTarget.size(), this->name());
    }

    return jointPositionTarget;
}

std::vector<double> Joint::jointVelocityTarget() const
{
    std::vector<double>& jointVelocityTarget = utils::getExistingComponentData<
        ignition::gazebo::components::JointVelocityTarget>(pImpl->ecm,
                                                           pImpl->jointEntity);

    if (jointVelocityTarget.size() != this->dofs()) {
        throw exceptions::DOFMismatch(
            this->dofs(), jointVelocityTarget.size(), this->name());
    }

    return jointVelocityTarget;
}

std::vector<double> Joint::jointAccelerationTarget() const
{
    std::vector<double>& jointAccelTarget = utils::getExistingComponentData<
        ignition::gazebo::components::JointAccelerationTarget>(
        pImpl->ecm, pImpl->jointEntity);

    if (jointAccelTarget.size() != this->dofs()) {
        throw exceptions::DOFMismatch(
            this->dofs(), jointAccelTarget.size(), this->name());
    }

    return jointAccelTarget;
}

std::vector<double> Joint::jointGeneralizedForceTarget() const
{
    std::vector<double>& jointForceTarget = utils::getExistingComponentData< //
        ignition::gazebo::components::JointForce>(pImpl->ecm,
                                                  pImpl->jointEntity);

    if (jointForceTarget.size() != this->dofs()) {
        throw exceptions::DOFMismatch(
            this->dofs(), jointForceTarget.size(), this->name());
    }

    return jointForceTarget;
}

bool Joint::resetJointPosition(const std::vector<double>& position)
{
    if (position.size() != this->dofs()) {
        gymppError << "Wrong number of elements (joint_dofs=" << this->dofs()
                   << ")" << std::endl;
        return false;
    }

    auto& jointPositionReset = utils::getComponentData< //
        ignition::gazebo::components::JointPositionReset>(pImpl->ecm,
                                                          pImpl->jointEntity);

    // Update the position
    jointPositionReset = position;

    // Get the PID
    ignition::math::PID& pid = utils::getExistingComponentData< //
        ignition::gazebo::components::JointPID>(pImpl->ecm, pImpl->jointEntity);

    // Reset the PID
    pid.Reset();

    return true;
}

bool Joint::resetJointVelocity(const std::vector<double>& velocity)
{
    if (velocity.size() != this->dofs()) {
        gymppError << "Wrong number of elements (joint_dofs=" << this->dofs()
                   << ")" << std::endl;
        return false;
    }

    auto& jointVelocityReset = utils::getComponentData< //
        ignition::gazebo::components::JointVelocityReset>(pImpl->ecm,
                                                          pImpl->jointEntity);

    // Update the velocity
    jointVelocityReset = velocity;

    // Get the PID
    ignition::math::PID& pid = utils::getExistingComponentData< //
        ignition::gazebo::components::JointPID>(pImpl->ecm, pImpl->jointEntity);

    // Reset the PID
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
        gymppError << "Failed to reset joint '" << this->name() << "'"
                   << std::endl;
        return false;
    }

    return true;
}
