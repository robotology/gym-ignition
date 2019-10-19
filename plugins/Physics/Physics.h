/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 *
 * ==================================================
 *
 * Copyright (C) 2018 Open Source Robotics Foundation
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
 *
 */

#ifndef GYMPP_PLUGINS_PHYSICS
#define GYMPP_PLUGINS_PHYSICS

#include <ignition/gazebo/System.hh>
#include <memory>

namespace gympp {
    namespace plugins {
        class Physics;
    } // namespace plugins
} // namespace gympp

class gympp::plugins::Physics final
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

#endif // GYMPP_PLUGINS_PHYSICS
