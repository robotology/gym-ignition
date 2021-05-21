/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef SCENARIO_CONTROLLERS_REFERENCES_H
#define SCENARIO_CONTROLLERS_REFERENCES_H

#include <array>
#include <vector>

namespace scenario::controllers {
    struct BaseReferences;
    struct JointReferences;
} // namespace scenario::controllers

struct scenario::controllers::BaseReferences
{
    std::array<double, 3> position = {0, 0, 0};
    std::array<double, 4> orientation = {1, 0, 0, 0};
    std::array<double, 3> linearVelocity = {0, 0, 0};
    std::array<double, 3> angularVelocity = {0, 0, 0};
    std::array<double, 3> linearAcceleration = {0, 0, 0};
    std::array<double, 3> angularAcceleration = {0, 0, 0};
};

struct scenario::controllers::JointReferences
{
    JointReferences(const size_t controlledDofs = 0)
    {
        position = std::vector<double>(controlledDofs, 0.0);
        velocity = std::vector<double>(controlledDofs, 0.0);
        acceleration = std::vector<double>(controlledDofs, 0.0);
    }

    inline bool valid() const
    {
        size_t dofs = position.size();
        return dofs > 0 && velocity.size() == dofs
               && acceleration.size() == dofs;
    }

    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> acceleration;
};

#endif // SCENARIO_CONTROLLERS_REFERENCES_H
