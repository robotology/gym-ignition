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

#ifndef SCENARIO_GAZEBO_JOINT_H
#define SCENARIO_GAZEBO_JOINT_H

#include "scenario/core/Joint.h"
#include "scenario/gazebo/GazeboEntity.h"

#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>

#include <memory>
#include <string>
#include <vector>

namespace scenario::gazebo {
    class Joint;
} // namespace scenario::gazebo

class scenario::gazebo::Joint final
    : public scenario::core::Joint
    , public scenario::gazebo::GazeboEntity
    , public std::enable_shared_from_this<scenario::gazebo::Joint>
{
public:
    Joint();
    virtual ~Joint();

    // =============
    // Gazebo Entity
    // =============

    uint64_t id() const override;

    bool initialize(const ignition::gazebo::Entity jointEntity,
                    ignition::gazebo::EntityComponentManager* ecm,
                    ignition::gazebo::EventManager* eventManager) override;

    bool createECMResources() override;

    // ============
    // Gazebo Joint
    // ============

    /**
     * Reset the position of a joint DOF.
     *
     * @param position The desired position.
     * @param dof The index of the DOF.
     * @return True for success, false otherwise.
     */
    bool resetPosition(const double position = 0, const size_t dof = 0);

    /**
     * Reset the velocity of a joint DOF.
     *
     * @param velocity The desired velocity.
     * @param dof The index of the DOF.
     * @return True for success, false otherwise.
     */
    bool resetVelocity(const double velocity = 0, const size_t dof = 0);

    /**
     * Reset the state of a joint DOF.
     *
     * This method also resets the PID state of the joint.
     *
     * @param position The desired position.
     * @param velocity The desired velocity.
     * @param dof The index of the DOF.
     * @return True for success, false otherwise.
     */
    bool reset(const double position = 0,
               const double velocity = 0,
               const size_t dof = 0);

    /**
     * Reset the position of the joint.
     *
     * This method also resets the PID state of the joint.
     *
     * @param position The desired position.
     * @return True for success, false otherwise.
     */
    bool resetJointPosition(const std::vector<double>& position);

    /**
     * Reset the velocity of the joint.
     *
     * This method also resets the PID state of the joint.
     *
     * @param velocity The desired velocity.
     * @return True for success, false otherwise.
     */
    bool resetJointVelocity(const std::vector<double>& velocity);

    /**
     * Reset the state of the joint.
     *
     * This method also resets the PID state of the joint.
     *
     * @param position The desired position.
     * @param velocity The desired velocity.
     * @return True for success, false otherwise.
     */
    bool resetJoint(const std::vector<double>& position,
                    const std::vector<double>& velocity);

    /**
     * Set the Coulomb friction parameter of the joint.
     *
     * @note Friction can be changed only before the first simulated step
     * after model insertion.
     *
     * @param value The new Coulomb friction value.
     * @return True for success, false otherwise.
     */
    bool setCoulombFriction(const double value);

    /**
     * Set the viscous friction parameter of the joint.
     *
     * @note Friction can be changed only before the first simulated step
     * after model insertion.
     *
     * @param value The new viscous friction value.
     * @return True for success, false otherwise.
     */
    bool setViscousFriction(const double value);

    // ==========
    // Joint Core
    // ==========

    bool valid() const override;

    size_t dofs() const override;

    std::string name(const bool scoped = false) const override;

    core::JointType type() const override;

    core::JointControlMode controlMode() const override;

    bool setControlMode(const core::JointControlMode mode) override;

    double controllerPeriod() const override;

    core::PID pid() const override;

    bool setPID(const core::PID& pid) override;

    bool historyOfAppliedJointForcesEnabled() const override;

    bool enableHistoryOfAppliedJointForces(
        const bool enable = true,
        const size_t maxHistorySize = 100) override;

    std::vector<double> historyOfAppliedJointForces() const override;

    double coulombFriction() const override;

    double viscousFriction() const override;

    // ==================
    // Single DOF methods
    // ==================

    core::Limit positionLimit(const size_t dof = 0) const override;

    double maxGeneralizedForce(const size_t dof = 0) const override;

    bool setMaxGeneralizedForce(const double maxForce,
                                const size_t dof = 0) override;

    double position(const size_t dof = 0) const override;

    double velocity(const size_t dof = 0) const override;

    double acceleration(const size_t dof = 0) const override;

    double generalizedForce(const size_t dof = 0) const override;

    bool setPositionTarget(const double position,
                           const size_t dof = 0) override;

    bool setVelocityTarget(const double velocity,
                           const size_t dof = 0) override;

    bool setAccelerationTarget(const double acceleration,
                               const size_t dof = 0) override;

    bool setGeneralizedForceTarget(const double force,
                                   const size_t dof = 0) override;

    double positionTarget(const size_t dof = 0) const override;

    double velocityTarget(const size_t dof = 0) const override;

    double accelerationTarget(const size_t dof = 0) const override;

    double generalizedForceTarget(const size_t dof = 0) const override;

    // =================
    // Multi DOF methods
    // =================

    core::JointLimit jointPositionLimit() const override;

    std::vector<double> jointMaxGeneralizedForce() const override;

    bool setJointMaxGeneralizedForce( //
        const std::vector<double>& maxForce) override;

    std::vector<double> jointPosition() const override;

    std::vector<double> jointVelocity() const override;

    std::vector<double> jointAcceleration() const override;

    std::vector<double> jointGeneralizedForce() const override;

    bool setJointPositionTarget(const std::vector<double>& position) override;

    bool setJointVelocityTarget(const std::vector<double>& velocity) override;

    bool setJointAccelerationTarget(
        const std::vector<double>& acceleration) override;

    bool setJointGeneralizedForceTarget( //
        const std::vector<double>& force) override;

    std::vector<double> jointPositionTarget() const override;

    std::vector<double> jointVelocityTarget() const override;

    std::vector<double> jointAccelerationTarget() const override;

    std::vector<double> jointGeneralizedForceTarget() const override;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

#endif // SCENARIO_GAZEBO_JOINT_H
