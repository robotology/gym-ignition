/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 * All rights reserved.
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

#ifndef SCENARIO_PLUGINS_GAZEBO_PHYSICS
#define SCENARIO_PLUGINS_GAZEBO_PHYSICS

#include <ignition/gazebo/System.hh>
#include <memory>

namespace scenario {
    namespace plugins {
        namespace gazebo {
            class Physics;
        } // namespace gazebo
    } // namespace plugins
} // namespace scenario

class scenario::plugins::gazebo::Physics final
    : public ignition::gazebo::System
    , public ignition::gazebo::ISystemUpdate
{
public:
    explicit Physics();
    ~Physics() override;

    void Update(const ignition::gazebo::UpdateInfo& info,
                ignition::gazebo::EntityComponentManager& ecm) override;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

#endif // SCENARIO_PLUGINS_GAZEBO_PHYSICS
