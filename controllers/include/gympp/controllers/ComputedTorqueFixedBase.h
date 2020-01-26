/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef GYMPP_CONTROLLERS_COMPUTEDTORQUEFIXEDBASE_H
#define GYMPP_CONTROLLERS_COMPUTEDTORQUEFIXEDBASE_H

#include "gympp/Robot.h"
#include "gympp/controllers/Controller.h"
#include "gympp/controllers/PositionController.h"

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace gympp {
    namespace controllers {
        class ComputedTorqueFixedBase;
    } // namespace controllers
} // namespace gympp

class gympp::controllers::ComputedTorqueFixedBase
    : public gympp::controllers::Controller
    , public gympp::controllers::PositionController
{
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

public:
    ComputedTorqueFixedBase() = delete;
    ComputedTorqueFixedBase(const std::string& urdfFile,
                            gympp::RobotPtr robot,
                            const std::vector<double>& kp,
                            const std::vector<double>& kd,
                            const std::vector<std::string>& controlledJoints);
    ~ComputedTorqueFixedBase() override;

    bool initialize() override;
    std::optional<std::vector<double>> step() override;
    bool terminate() override;

    bool setReferences(const gympp::controllers::PositionControllerReferences& references) override;
};

#endif // GYMPP_CONTROLLERS_COMPUTEDTORQUEFIXEDBASE_H
