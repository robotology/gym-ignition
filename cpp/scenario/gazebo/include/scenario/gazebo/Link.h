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

#ifndef SCENARIO_GAZEBO_LINK_H
#define SCENARIO_GAZEBO_LINK_H

#include "scenario/core/Link.h"
#include "scenario/gazebo/GazeboEntity.h"

#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>

#include <array>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace scenario::gazebo {
    class Link;
} // namespace scenario::gazebo

class scenario::gazebo::Link final
    : public scenario::core::Link
    , public scenario::gazebo::GazeboEntity
    , public std::enable_shared_from_this<scenario::gazebo::Link>
{
public:
    Link();
    virtual ~Link();

    // =============
    // Gazebo Entity
    // =============

    uint64_t id() const override;

    bool initialize(const ignition::gazebo::Entity linkEntity,
                    ignition::gazebo::EntityComponentManager* ecm,
                    ignition::gazebo::EventManager* eventManager) override;

    bool createECMResources() override;

    // =========
    // Link Core
    // =========

    bool valid() const override;

    std::string name(const bool scoped = false) const override;

    double mass() const override;

    std::array<double, 3> position() const override;

    std::array<double, 4> orientation() const override;

    std::array<double, 3> worldLinearVelocity() const override;

    std::array<double, 3> worldAngularVelocity() const override;

    std::array<double, 3> bodyLinearVelocity() const override;

    std::array<double, 3> bodyAngularVelocity() const override;

    std::array<double, 3> worldLinearAcceleration() const override;

    std::array<double, 3> worldAngularAcceleration() const override;

    std::array<double, 3> bodyLinearAcceleration() const override;

    std::array<double, 3> bodyAngularAcceleration() const override;

    bool contactsEnabled() const override;

    bool enableContactDetection(const bool enable) override;

    bool inContact() const override;

    std::vector<core::Contact> contacts() const override;

    std::array<double, 6> contactWrench() const override;

    // ===========
    // Gazebo Link
    // ===========

    /**
     * Apply a force to the link.
     *
     * The force is applied to the origin of the link frame.
     *
     * @param force The force to apply expressed in world coordinates.
     * @param duration The duration of the application of the force.
     * By default the force is applied for a single physics step.
     * @return True for success, false otherwise.
     */
    bool applyWorldForce(const std::array<double, 3>& force,
                         const double duration = 0.0);

    /**
     * Apply a torque to the link.
     *
     * The force is applied to the origin of the link frame.
     *
     * @param torque The torque to apply expressed in world coordinates.
     * @param duration The duration of the application of the torque.
     * By default the torque is applied for a single physics step.
     * @return True for success, false otherwise.
     */
    bool applyWorldTorque(const std::array<double, 3>& torque,
                          const double duration = 0.0);

    /**
     * Apply a wrench to the link.
     *
     * The force is applied to the origin of the link frame.
     *
     * @param force The force to apply expressed in world coordinates.
     * @param torque The torque to apply expressed in world coordinates.
     * @param duration The duration of the application of the wrench.
     * By default the wrench is applied for a single physics step.
     * @return True for success, false otherwise.
     */
    bool applyWorldWrench(const std::array<double, 3>& force,
                          const std::array<double, 3>& torque,
                          const double duration = 0.0);

    /**
     * Apply a wrench to the CoM of the link.
     *
     * @note This method considers the CoM being positioned in the origin of
     * the inertial frame as it is defined in the SDF description of the model.
     * Note that if not explicitly specified, inertial and link frames match.
     * In this case, `applyWorldWrench` and `applyWorldWrenchToCoM` effects
     * will be the same.
     *
     * @param force The force to apply expressed in world coordinates.
     * @param torque The torque to apply expressed in world coordinates.
     * @param duration The duration of the application of the wrench.
     * By default the wrench is applied for a single physics step.
     * @return True for success, false otherwise.
     */
    bool applyWorldWrenchToCoM(const std::array<double, 3>& force,
                               const std::array<double, 3>& torque,
                               const double duration = 0.0);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

#endif // SCENARIO_GAZEBO_LINK_H
