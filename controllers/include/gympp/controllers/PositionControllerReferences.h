/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef GYMPP_CONTROLLERS_POSITIONREFERENCES_H
#define GYMPP_CONTROLLERS_POSITIONREFERENCES_H

#include <array>
#include <string>
#include <vector>

namespace gympp {
    class Robot;
    namespace controllers {
        struct BaseReferences;
        struct JointReferences;
        class PositionControllerReferences;
    } // namespace controllers
} // namespace gympp

struct gympp::controllers::BaseReferences
{
    std::array<double, 3> position = {0, 0, 0};
    std::array<double, 4> orientation = {1, 0, 0, 0};
    std::array<double, 3> linearVelocity = {0, 0, 0};
    std::array<double, 3> angularVelocity = {0, 0, 0};
};

struct gympp::controllers::JointReferences
{
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> acceleration;
};

class gympp::controllers::PositionControllerReferences
{
public:
    PositionControllerReferences(const size_t controlledDofs = 0)
    {
        const std::vector<double> zeros(controlledDofs, 0.0);
        joints.position = zeros;
        joints.velocity = zeros;
        joints.acceleration = zeros;
    }

    BaseReferences base = {};
    JointReferences joints = {};

    inline bool valid() const
    {
        size_t dofs = joints.position.size();
        return dofs > 0 && joints.velocity.size() == dofs && joints.acceleration.size() == dofs;
    }

    // TODO: flatten()
    // TODO: import(const std::vector<double>& input, size_t options)
};

#endif // GYMPP_CONTROLLERS_POSITIONREFERENCES_H
