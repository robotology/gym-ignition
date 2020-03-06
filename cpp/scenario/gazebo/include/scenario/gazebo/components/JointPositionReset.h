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
 */

#ifndef IGNITION_GAZEBO_COMPONENTS_JOINTPOSITIONRESET_HH_
#define IGNITION_GAZEBO_COMPONENTS_JOINTPOSITIONRESET_HH_

#include <vector>

#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Serialization.hh>
#include <ignition/gazebo/config.hh>

namespace ignition {
    namespace gazebo {
        // Inline bracket to help doxygen filtering.
        inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
            namespace components {
                /// \brief Joint positions in SI units (rad for revolute, m for prismatic).
                ///
                /// The component wraps a std::vector of size equal to the degrees of freedom
                /// of the joint.
                using JointPositionReset = Component<std::vector<double>,
                                                     class JointPositionResetTag,
                                                     serializers::VectorDoubleSerializer>;
                IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.JointPositionReset",
                                              JointPositionReset)
            } // namespace components
        } // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
    } // namespace gazebo
} // namespace ignition

#endif
