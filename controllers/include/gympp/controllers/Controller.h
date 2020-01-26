/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef GYMPP_CONTROLLERS_CONTROLLER_H
#define GYMPP_CONTROLLERS_CONTROLLER_H

#include <chrono>
#include <optional>
#include <vector>

namespace gympp {
    class Robot;
    namespace controllers {
        class Controller;
    } // namespace controllers
} // namespace gympp

class gympp::controllers::Controller
{
public:
    using StepSize = std::chrono::duration<double>;

    virtual ~Controller() = default;

    virtual bool initialize() = 0;
    virtual std::optional<std::vector<double>> step() = 0;
    virtual bool terminate() = 0;
};

#endif // GYMPP_CONTROLLERS_CONTROLLER_H
