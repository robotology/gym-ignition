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

#include "scenario/gazebo/Log.h"

#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace scenario {
    namespace base {
        struct PID;
        struct Limit;
        struct JointLimit;
        enum class JointType
        {
            Invalid,
            Fixed,
            Revolute,
            Prismatic,
            Ball,
        };
        enum class JointControlMode
        {
            Idle,
            Force,
            Velocity,
            Position,
            PositionInterpolated,
        };
    } // namespace base
    namespace gazebo {
        class Joint;
        class Model;
    } // namespace gazebo
} // namespace scenario

class scenario::gazebo::Joint
{
public:
    Joint();
    virtual ~Joint();

    // ============
    // Gazebo Joint
    // ============

    uint64_t id() const;
    bool initialize(const ignition::gazebo::Entity jointEntity,
                    ignition::gazebo::EntityComponentManager* ecm,
                    ignition::gazebo::EventManager* eventManager);
    bool createECMResources();

    bool historyOfAppliedJointForcesEnabled() const;
    bool enableHistoryOfAppliedJointForces( //
        const bool enable = true,
        const size_t maxHistorySize = 100);
    std::vector<double> historyOfAppliedJointForces() const;

    // ==========
    // Joint Core
    // ==========

    size_t dofs() const;

    /**
     * Get the name of the joint.
     *
     * @param scoped If true, the scoped name of the joint is returned.
     * @return The name of the joint.
     */
    std::string name(const bool scoped = false) const;

    base::JointType type() const;

    base::JointControlMode controlMode() const;
    bool setControlMode(const base::JointControlMode mode);

    double controllerPeriod() const;

    base::PID pid() const;
    bool setPID(const base::PID& pid);

    // ==================
    // Single DOF methods
    // ==================

    base::Limit positionLimit(const size_t dof = 0) const;

    double maxGeneralizedForce(const size_t dof = 0) const;
    bool setMaxGeneralizedForce(const double maxForce, const size_t dof = 0);

    double position(const size_t dof = 0) const;
    double velocity(const size_t dof = 0) const;

    bool setPositionTarget(const double position, const size_t dof = 0);
    bool setVelocityTarget(const double velocity, const size_t dof = 0);
    bool setAccelerationTarget(const double acceleration, const size_t dof = 0);
    bool setGeneralizedForceTarget(const double force, const size_t dof = 0);

    double positionTarget(const size_t dof = 0) const;
    double velocityTarget(const size_t dof = 0) const;
    double accelerationTarget(const size_t dof = 0) const;
    double generalizedForceTarget(const size_t dof = 0) const;

    bool resetPosition(const double position = 0, const size_t dof = 0);
    bool resetVelocity(const double velocity = 0, const size_t dof = 0);
    bool reset(const double position = 0, //
               const double velocity = 0,
               const size_t dof = 0);

    // =================
    // Multi DOF methods
    // =================

    base::JointLimit jointPositionLimit() const;

    std::vector<double> jointMaxGeneralizedForce() const;
    bool setJointMaxGeneralizedForce(const std::vector<double>& maxForce);

    std::vector<double> jointPosition() const;
    std::vector<double> jointVelocity() const;

    bool setJointPositionTarget(const std::vector<double>& position);
    bool setJointVelocityTarget(const std::vector<double>& velocity);
    bool setJointAccelerationTarget(const std::vector<double>& acceleration);
    bool setJointGeneralizedForceTarget(const std::vector<double>& force);

    std::vector<double> jointPositionTarget() const;
    std::vector<double> jointVelocityTarget() const;
    std::vector<double> jointAccelerationTarget() const;
    std::vector<double> jointGeneralizedForceTarget() const;

    bool resetJointPosition(const std::vector<double>& position = {0});
    bool resetJointVelocity(const std::vector<double>& velocity = {0});
    bool resetJoint(const std::vector<double>& position = {0},
                    const std::vector<double>& velocity = {0});

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

struct scenario::base::PID
{
    PID() = default;
    PID(const double _p, const double _i, const double _d)
        : p(_p)
        , i(_i)
        , d(_d)
    {}

    double p = 0;
    double i = 0;
    double d = 0;
    double cmdMin = std::numeric_limits<double>::lowest();
    double cmdMax = std::numeric_limits<double>::max();
    double cmdOffset = 0;
    double iMin = std::numeric_limits<double>::lowest();
    double iMax = std::numeric_limits<double>::max();
};

struct scenario::base::Limit
{
    Limit() = default;
    Limit(const double _min, const double _max)
        : min(_min)
        , max(_max)
    {}

    double min = std::numeric_limits<double>::lowest();
    double max = std::numeric_limits<double>::max();
};

struct scenario::base::JointLimit
{
    JointLimit(const size_t dofs)
    {
        constexpr double m = std::numeric_limits<double>::lowest();
        constexpr double M = std::numeric_limits<double>::max();

        min = std::vector<double>(dofs, m);
        max = std::vector<double>(dofs, M);
    }

    JointLimit(const std::vector<double>& _min, const std::vector<double>& _max)
        : JointLimit(std::min(_min.size(), _max.size()))
    {
        if (_min.size() != _max.size()) {
            sWarning << "The max and min limits have different size. "
                     << "Ignoring the limits and using the smaller dimension."
                     << std::endl;
            return;
        }

        min = _min;
        max = _max;
    }

    std::vector<double> min;
    std::vector<double> max;
};

#endif // SCENARIO_GAZEBO_JOINT_H
