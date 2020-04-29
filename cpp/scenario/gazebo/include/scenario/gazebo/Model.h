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

#include "scenario/gazebo/Joint.h"

#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace scenario {
    namespace base {
        struct ContactData;
    } // namespace base
    namespace gazebo {
        class Link;
        class Model;
        using LinkPtr = std::shared_ptr<Link>;
        using JointPtr = std::shared_ptr<Joint>;
        using ModelPtr = std::shared_ptr<Model>;
    } // namespace gazebo
} // namespace scenario

class scenario::gazebo::Model
{
public:
    Model();
    virtual ~Model();

    // ============
    // Gazebo Model
    // ============

    uint64_t id() const;
    bool initialize(const ignition::gazebo::Entity modelEntity,
                    ignition::gazebo::EntityComponentManager* ecm,
                    ignition::gazebo::EventManager* eventManager);
    bool createECMResources();

    bool insertModelPlugin(const std::string& libName,
                           const std::string& className,
                           const std::string& context = {});

    bool historyOfAppliedJointForcesEnabled(
        const std::vector<std::string>& jointNames = {}) const;
    bool enableHistoryOfAppliedJointForces( //
        const bool enable = true,
        const size_t maxHistorySizePerJoint = 100,
        const std::vector<std::string>& jointNames = {});
    std::vector<double> historyOfAppliedJointForces(
        const std::vector<std::string>& jointNames = {}) const;

    // ==========
    // Model Core
    // ==========

    bool valid() const;

    size_t dofs(const std::vector<std::string>& jointNames = {}) const;
    std::string name() const;

    size_t nrOfLinks() const;
    size_t nrOfJoints() const;

    LinkPtr getLink(const std::string& linkName) const;
    JointPtr getJoint(const std::string& jointName) const;

    std::vector<std::string> linkNames(const bool scoped = false) const;
    std::vector<std::string> jointNames(const bool scoped = false) const;

    double controllerPeriod() const;
    bool setControllerPeriod(const double period);

    // ========
    // Contacts
    // ========

    bool contactsEnabled() const;
    bool enableContacts(const bool enable = true);

    bool selfCollisions() const;
    bool enableSelfCollisions(const bool enable = true);

    std::vector<std::string> linksInContact() const;

    std::vector<base::ContactData>
    contacts(const std::vector<std::string>& linkNames = {}) const;

    // ==================
    // Vectorized Methods
    // ==================

    std::vector<double> jointPositions( //
        const std::vector<std::string>& jointNames = {}) const;
    std::vector<double> jointVelocities( //
        const std::vector<std::string>& jointNames = {}) const;

    base::JointLimit jointLimits( //
        const std::vector<std::string>& jointNames = {}) const;

    bool setJointControlMode(const base::JointControlMode mode,
                             const std::vector<std::string>& jointNames = {});

    std::vector<LinkPtr> links( //
        const std::vector<std::string>& linkNames = {}) const;
    std::vector<JointPtr> joints( //
        const std::vector<std::string>& jointNames = {}) const;

    // =========================
    // Vectorized Target Methods
    // =========================

    bool setJointPositionTargets( //
        const std::vector<double>& positions,
        const std::vector<std::string>& jointNames = {});
    bool setJointVelocityTargets( //
        const std::vector<double>& velocities,
        const std::vector<std::string>& jointNames = {});
    bool setJointAccelerationTargets( //
        const std::vector<double>& velocities,
        const std::vector<std::string>& jointNames = {});
    bool setJointGeneralizedForceTargets( //
        const std::vector<double>& forces,
        const std::vector<std::string>& jointNames = {});

    bool resetJointPositions( //
        const std::vector<double>& positions,
        const std::vector<std::string>& jointNames = {});
    bool resetJointVelocities( //
        const std::vector<double>& velocities,
        const std::vector<std::string>& jointNames = {});

    std::vector<double> jointPositionTargets( //
        const std::vector<std::string>& jointNames = {}) const;
    std::vector<double> jointVelocityTargets( //
        const std::vector<std::string>& jointNames = {}) const;
    std::vector<double> jointAccelerationTargets( //
        const std::vector<std::string>& jointNames = {}) const;
    std::vector<double> jointGeneralizedForceTargets( //
        const std::vector<std::string>& jointNames = {}) const;

    // =========
    // Base Link
    // =========

    std::string baseFrame() const;
    bool setBaseFrame(const std::string& frameName);

    bool fixedBase() const;
    bool setAsFixedBase(const bool fixedBase = true);

    std::array<double, 3> basePosition() const;
    std::array<double, 4> baseOrientation() const;
    std::array<double, 3> baseBodyLinearVelocity() const;
    std::array<double, 3> baseBodyAngularVelocity() const;
    std::array<double, 3> baseWorldLinearVelocity() const;
    std::array<double, 3> baseWorldAngularVelocity() const;

    bool resetBaseWorldLinearVelocity(
        const std::array<double, 3>& linear = {0, 0, 0});
    bool resetBaseWorldAngularVelocity(
        const std::array<double, 3>& angular = {0, 0, 0});
    bool resetBaseWorldVelocity( //
        const std::array<double, 3>& linear = {0, 0, 0},
        const std::array<double, 3>& angular = {0, 0, 0});

    bool resetBasePose(const std::array<double, 3>& position = {0, 0, 0},
                       const std::array<double, 4>& orientation = {0, 0, 0, 0});
    bool resetBasePosition(const std::array<double, 3>& position = {0, 0, 0});
    bool resetBaseOrientation(
        const std::array<double, 4>& orientation = {0, 0, 0, 0});

    // =================
    // Base Link Targets
    // =================

    bool setBasePoseTarget(const std::array<double, 3>& position,
                           const std::array<double, 4>& orientation);
    bool setBasePositionTarget(const std::array<double, 3>& position);
    bool setBaseOrientationTarget(const std::array<double, 4>& orientation);

    bool setBaseWorldVelocityTarget(const std::array<double, 3>& linear,
                                    const std::array<double, 3>& angular);
    bool setBaseWorldLinearVelocityTarget(const std::array<double, 3>& linear);
    bool setBaseWorldAngularVelocityTarget( //
        const std::array<double, 3>& angular);
    bool setBaseWorldLinearAccelerationTarget( //
        const std::array<double, 3>& linear);
    bool setBaseWorldAngularAccelerationTarget( //
        const std::array<double, 3>& angular);

    std::array<double, 3> basePositionTarget() const;
    std::array<double, 4> baseOrientationTarget() const;
    std::array<double, 3> baseWorldLinearVelocityTarget() const;
    std::array<double, 3> baseWorldAngularVelocityTarget() const;
    std::array<double, 3> baseWorldLinearAccelerationTarget() const;
    std::array<double, 3> baseWorldAngularAccelerationTarget() const;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

#endif // SCENARIO_GAZEBO_MODEL_H
