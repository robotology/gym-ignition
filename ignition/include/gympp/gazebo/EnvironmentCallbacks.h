/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef GYMPP_GAZEBO_ENVIRONMENTCALLBACKS_H
#define GYMPP_GAZEBO_ENVIRONMENTCALLBACKS_H

#include "gympp/Environment.h"
#include <optional>

namespace gympp {
    namespace gazebo {
        class EnvironmentCallbacks;
    } // namespace gazebo
} // namespace gympp

class gympp::gazebo::EnvironmentCallbacks
{
public:
    using Action = gympp::Environment::Action;
    using Observation = gympp::Environment::Observation;
    using Reward = gympp::Environment::Reward;

    virtual ~EnvironmentCallbacks() = default;

    virtual bool isDone() = 0;
    virtual bool reset() = 0;
    virtual bool setAction(const Action& action) = 0;
    virtual std::optional<Reward> computeReward() = 0;
    virtual std::optional<Observation> getObservation() = 0;
};

#endif // GYMPP_GAZEBO_ENVIRONMENTCALLBACKS_H
