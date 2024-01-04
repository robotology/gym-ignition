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

#ifndef GZ_SIM_COMPONENTS_JOINTVELOCITYTARGET_H
#define GZ_SIM_COMPONENTS_JOINTVELOCITYTARGET_H

#include <vector>

#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Serialization.hh>
#include <gz/sim/config.hh>

namespace gz::sim {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {
        namespace components {
            /// \brief Joint velocity target in SI units (rad/s for
            ///        revolute, m/s for prismatic) used by joint
            ///        controllers.
            ///
            /// The component wraps a std::vector of size equal to the
            /// degrees of freedom of the joint.
            using JointVelocityTarget =
                Component<std::vector<double>,
                          class JointVelocityTargetTag,
                          serializers::VectorDoubleSerializer>;
            GZ_SIM_REGISTER_COMPONENT(
                "gz_sim_components.JointVelocityTarget",
                JointVelocityTarget)
        } // namespace components
    } // namespace GZ_SIM_VERSION_NAMESPACE
} // namespace gz::sim

#endif // GZ_SIM_COMPONENTS_JOINTVELOCITYTARGET_H
