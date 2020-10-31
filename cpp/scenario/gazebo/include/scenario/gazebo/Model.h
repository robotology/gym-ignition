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

#ifndef SCENARIO_GAZEBO_MODEL_H
#define SCENARIO_GAZEBO_MODEL_H

#include "scenario/core/Model.h"
#include "scenario/gazebo/GazeboEntity.h"

#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace scenario::gazebo {
    class Model;
} // namespace scenario::gazebo

class scenario::gazebo::Model final
    : public scenario::core::Model
    , public scenario::gazebo::GazeboEntity
    , public std::enable_shared_from_this<scenario::gazebo::Model>
{
public:
    Model();
    virtual ~Model();

    // =============
    // Gazebo Entity
    // =============

    uint64_t id() const override;

    bool initialize(const ignition::gazebo::Entity modelEntity,
                    ignition::gazebo::EntityComponentManager* ecm,
                    ignition::gazebo::EventManager* eventManager) override;
    bool createECMResources() override;

    // ============
    // Gazebo Model
    // ============

    /**
     * Insert a Ignition Gazebo plugin to the model.
     *
     * @param libName The library name of the plugin.
     * @param className The class name (or alias) of the plugin.
     * @param context Optional XML plugin context.
     * @return True for success, false otherwise.
     */
    bool insertModelPlugin(const std::string& libName,
                           const std::string& className,
                           const std::string& context = {});

    /**
     * Reset the positions of the joints.
     *
     * @param positions The desired new joint positions.
     * @param jointNames Optional vector of considered joints. By default,
     * ``Model::jointNames`` is used.
     * @return True for success, false otherwise.
     */
    bool resetJointPositions( //
        const std::vector<double>& positions,
        const std::vector<std::string>& jointNames = {});

    /**
     * Reset the velocities of the joints.
     *
     * @param velocities The desired new velocities positions.
     * @param jointNames Optional vector of considered joints. By default,
     * ``Model::jointNames`` is used.
     * @return True for success, false otherwise.
     */
    bool resetJointVelocities( //
        const std::vector<double>& velocities,
        const std::vector<std::string>& jointNames = {});

    /**
     * Reset the pose of the base link.
     *
     * @param position The desired position of the base link in world
     * coordinates.
     * @param orientation The wxyz quaternion defining the desired orientation
     * of the base link wrt the world frame.
     * @return True for success, false otherwise.
     */
    bool resetBasePose(const std::array<double, 3>& position = {0, 0, 0},
                       const std::array<double, 4>& orientation = {0, 0, 0, 0});

    /**
     * Reset the position of the base link.
     *
     * @param position The desired position of the base link in world
     * coordinates.
     * @return True for success, false otherwise.
     */
    bool resetBasePosition(const std::array<double, 3>& position = {0, 0, 0});

    /**
     * Reset the orientation of the base link.
     *
     * @param orientation The wxyz quaternion defining the desired orientation
     * of the base link wrt the world frame.
     * @return True for success, false otherwise.
     */
    bool resetBaseOrientation(
        const std::array<double, 4>& orientation = {0, 0, 0, 0});

    /**
     * Reset the linear mixed velocity of the base link.
     *
     * @param linear The desired linear mixed velocity of the base link.
     * @return True for success, false otherwise.
     */
    bool resetBaseWorldLinearVelocity(
        const std::array<double, 3>& linear = {0, 0, 0});

    /**
     * Reset the angular mixed velocity of the base link.
     *
     * @param angular The desired angular mixed velocity of the base link.
     * @return True for success, false otherwise.
     */
    bool resetBaseWorldAngularVelocity(
        const std::array<double, 3>& angular = {0, 0, 0});

    /**
     * Reset the mixed velocity of the base link.
     *
     * @param linear The desired linear mixed velocity of the base link.
     * @param angular The desired angular mixed velocity of the base link.
     * @return True for success, false otherwise.
     */
    bool resetBaseWorldVelocity( //
        const std::array<double, 3>& linear = {0, 0, 0},
        const std::array<double, 3>& angular = {0, 0, 0});

    // ==========
    // Model Core
    // ==========

    bool valid() const override;

    size_t dofs(const std::vector<std::string>& jointNames = {}) const override;

    std::string name() const override;

    size_t nrOfLinks() const override;

    size_t nrOfJoints() const override;

    double
    totalMass(const std::vector<std::string>& linkNames = {}) const override;

    core::LinkPtr getLink(const std::string& linkName) const override;

    core::JointPtr getJoint(const std::string& jointName) const override;

    std::vector<std::string>
    linkNames(const bool scoped = false) const override;

    std::vector<std::string>
    jointNames(const bool scoped = false) const override;

    double controllerPeriod() const override;

    bool setControllerPeriod(const double period) override;

    bool enableHistoryOfAppliedJointForces(
        const bool enable = true,
        const size_t maxHistorySizePerJoint = 100,
        const std::vector<std::string>& jointNames = {}) override;

    bool historyOfAppliedJointForcesEnabled(
        const std::vector<std::string>& jointNames = {}) const override;

    std::vector<double> historyOfAppliedJointForces(
        const std::vector<std::string>& jointNames = {}) const override;

    // ========
    // Contacts
    // ========

    bool contactsEnabled() const override;

    bool enableContacts(const bool enable = true) override;

    bool selfCollisionsEnabled() const override;

    bool enableSelfCollisions(const bool enable = true) override;

    std::vector<std::string> linksInContact() const override;

    std::vector<core::Contact>
    contacts(const std::vector<std::string>& linkNames = {}) const override;

    // ==================
    // Vectorized Methods
    // ==================

    std::vector<double> jointPositions( //
        const std::vector<std::string>& jointNames = {}) const override;

    std::vector<double> jointVelocities( //
        const std::vector<std::string>& jointNames = {}) const override;

    std::vector<double> jointAccelerations( //
        const std::vector<std::string>& jointNames = {}) const override;

    std::vector<double> jointGeneralizedForces( //
        const std::vector<std::string>& jointNames = {}) const override;

    core::JointLimit jointLimits( //
        const std::vector<std::string>& jointNames = {}) const override;

    bool setJointControlMode( //
        const core::JointControlMode mode,
        const std::vector<std::string>& jointNames = {}) override;

    std::vector<core::LinkPtr> links( //
        const std::vector<std::string>& linkNames = {}) const override;

    std::vector<core::JointPtr> joints( //
        const std::vector<std::string>& jointNames = {}) const override;

    // =========================
    // Vectorized Target Methods
    // =========================

    bool setJointPositionTargets( //
        const std::vector<double>& positions,
        const std::vector<std::string>& jointNames = {}) override;

    bool setJointVelocityTargets( //
        const std::vector<double>& velocities,
        const std::vector<std::string>& jointNames = {}) override;

    bool setJointAccelerationTargets( //
        const std::vector<double>& accelerations,
        const std::vector<std::string>& jointNames = {}) override;

    bool setJointGeneralizedForceTargets( //
        const std::vector<double>& forces,
        const std::vector<std::string>& jointNames = {}) override;

    std::vector<double> jointPositionTargets( //
        const std::vector<std::string>& jointNames = {}) const override;

    std::vector<double> jointVelocityTargets( //
        const std::vector<std::string>& jointNames = {}) const override;

    std::vector<double> jointAccelerationTargets( //
        const std::vector<std::string>& jointNames = {}) const override;

    std::vector<double> jointGeneralizedForceTargets( //
        const std::vector<std::string>& jointNames = {}) const override;

    // =========
    // Base Link
    // =========

    std::string baseFrame() const override;

    std::array<double, 3> basePosition() const override;

    std::array<double, 4> baseOrientation() const override;

    std::array<double, 3> baseBodyLinearVelocity() const override;

    std::array<double, 3> baseBodyAngularVelocity() const override;

    std::array<double, 3> baseWorldLinearVelocity() const override;

    std::array<double, 3> baseWorldAngularVelocity() const override;

    // =================
    // Base Link Targets
    // =================

    bool setBasePoseTarget( //
        const std::array<double, 3>& position,
        const std::array<double, 4>& orientation) override;

    bool setBasePositionTarget( //
        const std::array<double, 3>& position) override;

    bool setBaseOrientationTarget( //
        const std::array<double, 4>& orientation) override;

    bool setBaseWorldVelocityTarget( //
        const std::array<double, 3>& linear,
        const std::array<double, 3>& angular) override;

    bool setBaseWorldLinearVelocityTarget( //
        const std::array<double, 3>& linear) override;

    bool setBaseWorldAngularVelocityTarget( //
        const std::array<double, 3>& angular) override;

    bool setBaseWorldLinearAccelerationTarget( //
        const std::array<double, 3>& linear) override;

    bool setBaseWorldAngularAccelerationTarget( //
        const std::array<double, 3>& angular) override;

    std::array<double, 3> basePositionTarget() const override;

    std::array<double, 4> baseOrientationTarget() const override;

    std::array<double, 3> baseWorldLinearVelocityTarget() const override;

    std::array<double, 3> baseWorldAngularVelocityTarget() const override;

    std::array<double, 3> baseWorldLinearAccelerationTarget() const override;

    std::array<double, 3> baseWorldAngularAccelerationTarget() const override;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

#endif // SCENARIO_GAZEBO_MODEL_H
