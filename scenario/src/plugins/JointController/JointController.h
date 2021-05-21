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

#ifndef SCENARIO_PLUGINS_GAZEBO_JOINTCONTROLLER_H
#define SCENARIO_PLUGINS_GAZEBO_JOINTCONTROLLER_H

#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>
#include <ignition/gazebo/System.hh>
#include <sdf/Element.hh>

#include <memory>

namespace scenario::plugins::gazebo {
    class JointController;
} // namespace scenario::plugins::gazebo

class scenario::plugins::gazebo::JointController final
    : public ignition::gazebo::System
    , public ignition::gazebo::ISystemConfigure
    , public ignition::gazebo::ISystemPreUpdate
{
private:
    class Impl;
    std::unique_ptr<Impl> pImpl = nullptr;

public:
    JointController();
    ~JointController() override;

    void Configure(const ignition::gazebo::Entity& entity,
                   const std::shared_ptr<const sdf::Element>& sdf,
                   ignition::gazebo::EntityComponentManager& ecm,
                   ignition::gazebo::EventManager& eventMgr) override;

    void PreUpdate(const ignition::gazebo::UpdateInfo& info,
                   ignition::gazebo::EntityComponentManager& ecm) override;
};

#endif // SCENARIO_PLUGINS_GAZEBO_JOINTCONTROLLER_H
