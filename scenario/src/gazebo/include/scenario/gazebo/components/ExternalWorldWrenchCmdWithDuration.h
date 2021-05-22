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

#ifndef IGNITION_GAZEBO_COMPONENTS_EXTERNALWORLDWRENCHCMDWITHDURATION_H
#define IGNITION_GAZEBO_COMPONENTS_EXTERNALWORLDWRENCHCMDWITHDURATION_H

#include "scenario/gazebo/helpers.h"

#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/config.hh>

#include <chrono>

namespace ignition::gazebo {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
        namespace components {
            /// \brief A component type that contains the external wrench to be
            ///        applied for a given duration on an entity expressed in
            ///        the world frame and represented by
            ///        ignition::msgs::Wrench. Currently this is used for
            ///        applying wrenches on links. Although the msg::Wrench type
            ///        has a force_offset member, the value is currently
            ///        ignored. Instead, the force is applied at the link
            ///        origin. The wrench uses SI units (N for force and N⋅m for
            ///        torque).
            using ExternalWorldWrenchCmdWithDuration =
                Component<scenario::gazebo::utils::LinkWrenchCmd,
                          class ExternalWorldWrenchCmdWithDurationTag>;
            IGN_GAZEBO_REGISTER_COMPONENT(
                "ign_gazebo_components.ExternalWorldWrenchCmdWithDuration",
                ExternalWorldWrenchCmdWithDuration)
        } // namespace components
    } // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
} // namespace ignition::gazebo

#endif // IGNITION_GAZEBO_COMPONENTS_EXTERNALWORLDWRENCHCMDWITHDURATION_H
