/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "CartPolePlugin.h"
#include "gympp/Common.h"
#include "gympp/Log.h"
#include "gympp/Random.h"
#include "gympp/Robot.h"
#include "gympp/gazebo/RobotSingleton.h"

#include <ignition/plugin/Register.hh>

#include <cmath>
#include <mutex>

using namespace gympp::plugins;

using ActionDataType = int;
using ActionSample = gympp::BufferContainer<ActionDataType>::type;

using ObservationDataType = double;
using ObservationSample = gympp::BufferContainer<ObservationDataType>::type;

const unsigned PolePositionObservationIndex = 0;
const unsigned CartPositionObservationIndex = 1;

enum CartPoleAction
{
    MOVE_RIGHT,
    MOVE_LEFT,
    DONT_MOVE,
};

class CartPole::Impl
{
private:
    gympp::RobotPtr robot = nullptr;

public:
    unsigned seed;
    bool firstRun = true;
    mutable std::mutex mutex;

    double cartPositionReference;
    ObservationSample observationBuffer;
    std::optional<CartPoleAction> action;

    double getRandomThetaInRad()
    {
        std::uniform_real_distribution<> distr(-25.0 * (M_PI / 180.0), 25.0 * (M_PI / 180.0));
        return distr(gympp::Random::engine());
    }

    gympp::RobotPtr getRobot()
    {
        if (!robot) {
            auto& robotSingleton = gympp::gazebo::RobotSingleton::get();
            robot = robotSingleton.getRobot("cartpole_xacro");
        }

        assert(robot);
        assert(robot->valid());
        return robot;
    }
};

CartPole::CartPole()
    : System()
    , pImpl{new Impl(), [](Impl* impl) { delete impl; }}
{
    // - Pole angular position [deg]
    // - Cart linear position [m]
    pImpl->observationBuffer.resize(2);
}

void CartPole::PreUpdate(const ignition::gazebo::UpdateInfo& info,
                         ignition::gazebo::EntityComponentManager& /*manager*/)
{
    if (info.paused) {
        return;
    }

    // Get the pointer to the Robot interface
    gympp::RobotPtr robot = pImpl->getRobot();
    assert(robot);

    if (pImpl->firstRun) {
        pImpl->firstRun = false;

        // Initialize the PID
        if (!robot->setJointPID("linear", {10000, 50, 200})) {
            gymppError << "Failed to set the PID of joint 'linear'" << std::endl;
            return;
        }

        // Stop here and run the first simulation step
        return;
    }

    // Set the step size
    if (!robot->setdt(info.dt)) {
        gymppError << "Failed to set the step size" << std::endl;
        return;
    }

    // Actuate the action
    // ==================

    {
        double appliedForce = 0;
        std::lock_guard lock(pImpl->mutex);

        if (pImpl->action) {
            switch (*pImpl->action) {
                case MOVE_LEFT:
                    appliedForce = 10;
                    break;
                case MOVE_RIGHT:
                    appliedForce = -10;
                    break;
                case DONT_MOVE:
                    break;
            }

            // Reset the action. This allows to perform multiple simulator iterations for each
            // environment step.
            pImpl->action.reset();

            if (!robot->setJointForce("linear", -20)) {
                gymppError << "Failed to set the force to joint 'linear'" << std::endl;
                return;
            }
        }
    }
}

void CartPole::PostUpdate(const ignition::gazebo::UpdateInfo& info,
                          const ignition::gazebo::EntityComponentManager& /*manager*/)
{
    if (info.paused) {
        return;
    }

    // Get the pointer to the Robot interface
    gympp::RobotPtr robot = pImpl->getRobot();
    assert(robot);

    double poleJointPosition = robot->jointPosition("pivot");
    double cartJointPosition = robot->jointPosition("linear");

    {
        std::lock_guard lock(pImpl->mutex);
        pImpl->observationBuffer[PolePositionObservationIndex] = (180.0 / M_PI) * poleJointPosition;
        pImpl->observationBuffer[CartPositionObservationIndex] = cartJointPosition;
    }
}

bool CartPole::isDone()
{
    std::lock_guard lock(pImpl->mutex);

    if (std::abs(pImpl->observationBuffer[PolePositionObservationIndex]) > 60.0
        || std::abs(pImpl->observationBuffer[CartPositionObservationIndex]) > 0.95) {
        return true;
    }

    return false;
}

bool CartPole::reset()
{
    // Get the pointer to the Robot interface
    gympp::RobotPtr robot = pImpl->getRobot();
    assert(robot);

    if (!robot) {
        gymppError << "Failed to get pointer to the robot interface" << std::endl;
        return false;
    }

    // Joint positions
    auto x0 = 0.0;
    auto theta0 = pImpl->getRandomThetaInRad();

    // Set the random pole angle
    if (!robot->resetJoint("pivot", theta0)) {
        gymppError << "Failed to reset the position of joint 'pivot'" << std::endl;
        return false;
    }

    // Reset the cart position
    if (!robot->resetJoint("linear", x0)) {
        gymppError << "Failed to reset the position of joint 'linear'" << std::endl;
        return false;
    }

    // Notify that the next iteration is equivalent to the first run
    pImpl->firstRun = true;

    {
        // Update the observation. This is required because the Environment::reset()
        // method returns the new observation.
        std::lock_guard lock(pImpl->mutex);

        pImpl->observationBuffer[PolePositionObservationIndex] = (180.0 / M_PI) * theta0;
        pImpl->observationBuffer[CartPositionObservationIndex] = x0;
    }

    return true;
}

bool CartPole::setAction(const EnvironmentBehavior::Action& action)
{
    std::lock_guard lock(pImpl->mutex);

    assert(action.get<ActionDataType>(0));
    ActionDataType actionValue = action.get<ActionDataType>(0).value();

    if (actionValue == 0) {
        pImpl->action = MOVE_LEFT;
    }
    else if (actionValue == 1) {
        pImpl->action = MOVE_RIGHT;
    }
    else if (actionValue == 2) {
        pImpl->action = DONT_MOVE;
    }
    else {
        return false;
    }

    return true;
}

std::optional<gympp::gazebo::EnvironmentBehavior::Reward> CartPole::computeReward()
{
    std::lock_guard lock(pImpl->mutex);
    return 1.0; // TODO
}

std::optional<gympp::gazebo::EnvironmentBehavior::Observation> CartPole::getObservation()
{
    std::lock_guard lock(pImpl->mutex);
    return Observation(pImpl->observationBuffer);
}

IGNITION_ADD_PLUGIN(gympp::plugins::CartPole,
                    gympp::plugins::CartPole::System,
                    gympp::plugins::CartPole::ISystemPreUpdate,
                    gympp::plugins::CartPole::ISystemPostUpdate,
                    gympp::gazebo::EnvironmentBehavior)
