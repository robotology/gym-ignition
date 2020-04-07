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

#ifndef SCENARIO_CONTROLLERS_COMPUTEDTORQUEFIXEDBASE_H
#define SCENARIO_CONTROLLERS_COMPUTEDTORQUEFIXEDBASE_H

#include "scenario/controllers/Controller.h"
#include "scenario/controllers/References.h"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace scenario {
    namespace gazebo {
        class Model;
    } // namespace gazebo
    namespace controllers {
        class ControllersFactory;
        class ComputedTorqueFixedBase;
    } // namespace controllers
} // namespace scenario

class scenario::controllers::ComputedTorqueFixedBase final
    : public scenario::controllers::Controller
    , public scenario::controllers::UseScenarioModel
    , public scenario::controllers::SetJointReferences
{
public:
    ComputedTorqueFixedBase() = delete;
    ComputedTorqueFixedBase(const std::string& urdfFile,
                            std::shared_ptr<gazebo::Model> model,
                            const std::vector<double>& kp,
                            const std::vector<double>& kd,
                            const std::vector<std::string>& controlledJoints,
                            const std::array<double, 3> gravity = g);
    ~ComputedTorqueFixedBase() override;

    bool initialize() override;
    bool step(const StepSize& dt) override;
    bool terminate() override;

    bool updateStateFromModel() override;

    const std::vector<std::string>& controlledJoints() override;
    bool setJointReferences(const JointReferences& jointReferences) override;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

#endif // SCENARIO_CONTROLLERS_COMPUTEDTORQUEFIXEDBASE_H
