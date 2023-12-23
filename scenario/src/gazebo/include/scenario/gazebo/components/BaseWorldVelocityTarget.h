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

#ifndef GZ_SIM_COMPONENTS_BASEWORLDVELOCITYTARGET_H
#define GZ_SIM_COMPONENTS_BASEWORLDVELOCITYTARGET_H

#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Serialization.hh>
#include <gz/sim/config.hh>
#include <gz/math/Vector3.hh>

namespace gz::sim {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {
        namespace components {
            /// \brief Base world linear velocity target used by
            ///        floating base controllers.
            using BaseWorldLinearVelocityTarget =
                Component<math::Vector3d,
                          class BaseWorldLinearVelocityTargetTag>;
            GZ_SIM_REGISTER_COMPONENT(
                "gz_sim_components.BaseWorldLinearVelocityTarget",
                BaseWorldLinearVelocityTarget)

            /// \brief Base world angular velocity target used by
            ///        floating base controllers.
            using BaseWorldAngularVelocityTarget =
                Component<math::Vector3d,
                          class BaseWorldAngularVelocityTargetTag>;
            GZ_SIM_REGISTER_COMPONENT(
                "gz_sim_components."
                "BaseWorldAngularVelocityTargetTarget",
                BaseWorldAngularVelocityTarget)
        } // namespace components
    } // namespace GZ_SIM_VERSION_NAMESPACE
} // namespace gz::sim

#endif // GZ_SIM_COMPONENTS_BASEWORLDVELOCITYTARGET_H
