/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef SCENARIO_CONTROLLERS_COMPUTEDTORQUEFIXEDBASE_H
#define SCENARIO_CONTROLLERS_COMPUTEDTORQUEFIXEDBASE_H

#include "scenario/controllers/Controller.h"
#include "scenario/controllers/References.h"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace scenario::core {
    class Model;
} // namespace scenario::core

namespace scenario::controllers {
    class ControllersFactory;
    class ComputedTorqueFixedBase;
} // namespace scenario::controllers

class scenario::controllers::ComputedTorqueFixedBase final
    : public scenario::controllers::Controller
    , public scenario::controllers::UseScenarioModel
    , public scenario::controllers::SetJointReferences
{
public:
    ComputedTorqueFixedBase() = delete;
    ComputedTorqueFixedBase(const std::string& urdfFile,
                            std::shared_ptr<core::Model> model,
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
