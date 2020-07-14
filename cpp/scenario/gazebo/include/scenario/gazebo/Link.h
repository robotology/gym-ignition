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

#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>

#include <array>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace scenario {
    namespace base {
        struct Pose;
        struct Contact;
        struct ContactPoint;
    } // namespace base
    namespace gazebo {
        class Link;
        class Model;
    } // namespace gazebo
} // namespace scenario

class scenario::gazebo::Link
{
public:
    Link();
    virtual ~Link();

    // ===========
    // Gazebo Link
    // ===========

    uint64_t id() const;
    bool initialize(const ignition::gazebo::Entity linkEntity,
                    ignition::gazebo::EntityComponentManager* ecm,
                    ignition::gazebo::EventManager* eventManager);
    bool createECMResources();

    // =========
    // Link Core
    // =========

    /**
     * Get the name of the link.
     *
     * @param scoped If true, the scoped name of the link is returned.
     * @return The name of the link.
     */
    std::string name(const bool scoped = false) const;
    double mass() const;

    std::array<double, 3> position() const;
    std::array<double, 4> orientation() const;

    std::array<double, 3> worldLinearVelocity() const;
    std::array<double, 3> worldAngularVelocity() const;
    std::array<double, 3> bodyLinearVelocity() const;
    std::array<double, 3> bodyAngularVelocity() const;

    std::array<double, 3> worldLinearAcceleration() const;
    std::array<double, 3> worldAngularAcceleration() const;
    std::array<double, 3> bodyLinearAcceleration() const;
    std::array<double, 3> bodyAngularAcceleration() const;

    bool contactsEnabled() const;
    bool enableContactDetection(const bool enable);

    bool inContact() const;
    std::vector<base::Contact> contacts() const;
    std::array<double, 6> contactWrench() const;

    bool applyWorldForce(const std::array<double, 3>& force,
                         const double duration = 0.0);
    bool applyWorldTorque(const std::array<double, 3>& torque,
                          const double duration = 0.0);
    bool applyWorldWrench(const std::array<double, 3>& force,
                          const std::array<double, 3>& torque,
                          const double duration = 0.0);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

struct scenario::base::Pose
{
    Pose() = default;
    Pose(std::array<double, 3> p, std::array<double, 4> o)
        : position(p)
        , orientation(o)
    {}
    Pose(std::pair<std::array<double, 3>, std::array<double, 4>> pose)
        : position(pose.first)
        , orientation(pose.second)
    {}

    static scenario::base::Pose Identity() { return {}; }

    bool operator==(const Pose& other) const
    {
        return this->position == other.position
               && this->orientation == other.orientation;
    }

    bool operator!=(const Pose& other) const { return !(*this == other); }

    std::array<double, 3> position = {0, 0, 0};
    std::array<double, 4> orientation = {1, 0, 0, 0};
};

struct scenario::base::ContactPoint
{
    double depth;
    std::array<double, 3> force;
    std::array<double, 3> torque;
    std::array<double, 3> normal;
    std::array<double, 3> position;
};

struct scenario::base::Contact
{
    std::string bodyA;
    std::string bodyB;
    std::vector<ContactPoint> points;
};

#endif // SCENARIO_GAZEBO_LINK_H
