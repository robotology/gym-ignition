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

#ifndef IGNITION_GAZEBO_COMPONENTS_JOINTCONTROLLERPERIOD_H
#define IGNITION_GAZEBO_COMPONENTS_JOINTCONTROLLERPERIOD_H

#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Serialization.hh>
#include <ignition/gazebo/config.hh>

#include <chrono>

namespace ignition::gazebo {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
        namespace components {
            /// \brief Joint controller period in seconds.
            using JointControllerPeriod =
                Component<std::chrono::steady_clock::duration,
                          class JointControllerPeriodTag>;
            IGN_GAZEBO_REGISTER_COMPONENT(
                "ign_gazebo_components.JointControllerPeriod",
                JointControllerPeriod)
        } // namespace components
    } // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
} // namespace ignition::gazebo

#endif // IGNITION_GAZEBO_COMPONENTS_JOINTCONTROLLERPERIOD_H
