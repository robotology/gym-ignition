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
#include "gympp/gazebo/EnvironmentCallbacksSingleton.h"
#include "gympp/gazebo/IgnitionRobot.h"
#include "gympp/gazebo/RobotSingleton.h"

#include <ignition/plugin/Register.hh>

#include <cmath>
#include <mutex>

using namespace gympp::gazebo;
using namespace gympp::plugins;

using ActionDataType = int;
using ActionSample = gympp::BufferContainer<ActionDataType>::type;

using ObservationDataType = double;
using ObservationSample = gympp::BufferContainer<ObservationDataType>::type;

const unsigned CartPositionObservationIndex = 0;
const unsigned CartVelocityObservationIndex = 1;
const unsigned PolePositionObservationIndex = 2;
const unsigned PoleVelocityObservationIndex = 3;

const size_t MaxEpisodeLength = 200;
const double XThreshold = 2.4;
const double ThetaThresholdDeg = 12;
const double AppliedForce = 10;
const double MaxTheta0Rad = 10 * (M_PI / 180.0);

enum CartPoleAction
{
    MOVE_RIGHT,
    MOVE_LEFT,
    DONT_MOVE,
};

class CartPole::Impl
{
public:
    unsigned seed;
    bool firstRun = true;
    mutable std::mutex mutex;

    size_t iterations;
    double cartPositionReference;
    ObservationSample observationBuffer;
    std::optional<CartPoleAction> action;

    std::shared_ptr<gympp::Robot> robot = nullptr;

    double getRandomThetaInRad()
    {
        std::uniform_real_distribution<> distr(-MaxTheta0Rad, MaxTheta0Rad);
        return distr(gympp::Random::engine());
    }
};

CartPole::CartPole()
    : System()
    , pImpl{new Impl(), [](Impl* impl) { delete impl; }}
{
    // - Cart linear position [m]
    // - Cart linear velocity [m/s]
    // - Pole angular position [deg]
    // - Pole angular velocity [deg/s]
    pImpl->observationBuffer.resize(4);
}

void CartPole::Configure(const ignition::gazebo::Entity& entity,
                         const std::shared_ptr<const sdf::Element>& sdf,
                         ignition::gazebo::EntityComponentManager& ecm,
                         ignition::gazebo::EventManager& /*eventMgr*/)
{
    // Create a gympp::IgnitionRobot object from the ecm
    auto ignRobot = std::make_shared<gympp::gazebo::IgnitionRobot>();
    if (!ignRobot->configureECM(entity, sdf, ecm)) {
        gymppError << "Failed to configure the Robot interface" << std::endl;
        return;
    }

    if (!ignRobot->valid()) {
        gymppError << "The Robot interface is not valid" << std::endl;
        return;
    }

    // Store a pointer to gympp::Robot
    pImpl->robot = ignRobot;

    // TODO: Right now the gympp::Robot interface is only used inside this plugin.
    //       Since we want to expose the robot also to python (in order to read and
    //       modify the state) it can be registered in the RobotSingleton using the
    //       scoped name.

    // Auto-register the environment callbacks
    gymppDebug << "Registering environment callbacks for robot '" << ignRobot->name() << "'"
               << std::endl;
    auto ecSingleton = EnvironmentCallbacksSingleton::Instance();
    bool registered = ecSingleton->storeEnvironmentCallback(ignRobot->name(), this);
    assert(registered);
}

void CartPole::PreUpdate(const ignition::gazebo::UpdateInfo& info,
                         ignition::gazebo::EntityComponentManager& /*manager*/)
{
    if (info.paused) {
        return;
    }

    assert(pImpl->robot);

    if (pImpl->firstRun) {
        pImpl->firstRun = false;

        // Initialize the PID
        if (!pImpl->robot->setJointPID("linear", {10000, 50, 200})) {
            gymppError << "Failed to set the PID of joint 'linear'" << std::endl;
            return;
        }

        // Stop here and run the first simulation step
        return;
    }

    // Set the step size
    if (!pImpl->robot->setdt(info.dt)) {
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
                    appliedForce = AppliedForce;
                    break;
                case MOVE_RIGHT:
                    appliedForce = -AppliedForce;
                    break;
                case DONT_MOVE:
                    break;
            }

            // Reset the action. This allows to perform multiple simulator iterations for each
            // environment step.
            pImpl->action.reset();
        }

        if (!pImpl->robot->setJointForce("linear", appliedForce)) {
            gymppError << "Failed to set the force to joint 'linear'" << std::endl;
            return;
        }
    }
}

void CartPole::PostUpdate(const ignition::gazebo::UpdateInfo& info,
                          const ignition::gazebo::EntityComponentManager& /*manager*/)
{
    if (info.paused) {
        return;
    }

    assert(pImpl->robot);

    double cartJointPosition = pImpl->robot->jointPosition("linear");
    double poleJointPosition = pImpl->robot->jointPosition("pivot");

    double cartJointVelocity = pImpl->robot->jointVelocity("linear");
    double poleJointVelocity = pImpl->robot->jointVelocity("pivot");

    {
        std::lock_guard lock(pImpl->mutex);
        pImpl->observationBuffer[CartPositionObservationIndex] = cartJointPosition;
        pImpl->observationBuffer[CartVelocityObservationIndex] = cartJointVelocity;
        pImpl->observationBuffer[PolePositionObservationIndex] = (180.0 / M_PI) * poleJointPosition;
        pImpl->observationBuffer[PoleVelocityObservationIndex] = (180.0 / M_PI) * poleJointVelocity;
    }
}

bool CartPole::isDone()
{
    std::lock_guard lock(pImpl->mutex);

    if (pImpl->iterations >= MaxEpisodeLength
        || std::abs(pImpl->observationBuffer[PolePositionObservationIndex]) > ThetaThresholdDeg
        || std::abs(pImpl->observationBuffer[CartPositionObservationIndex]) > XThreshold) {
        return true;
    }

    return false;
}

bool CartPole::reset()
{
    assert(pImpl->robot);

    // Reset the number of iterations
    pImpl->iterations = 0;

    // Joint positions
    auto x0 = 0.0;
    auto v0 = 0.0;
    auto theta0 = pImpl->getRandomThetaInRad();

    // Set the random pole angle
    if (!pImpl->robot->resetJoint("pivot", theta0, v0)) {
        gymppError << "Failed to reset the position of joint 'pivot'" << std::endl;
        return false;
    }

    // Reset the cart position
    if (!pImpl->robot->resetJoint("linear", x0, v0)) {
        gymppError << "Failed to reset the position of joint 'linear'" << std::endl;
        return false;
    }

    // Notify that the next iteration is equivalent to the first run
    pImpl->firstRun = true;

    {
        // Update the observation. This is required because the Environment::reset()
        // method returns the new observation.
        std::lock_guard lock(pImpl->mutex);

        pImpl->observationBuffer[CartPositionObservationIndex] = x0;
        pImpl->observationBuffer[CartVelocityObservationIndex] = v0;
        pImpl->observationBuffer[PolePositionObservationIndex] = (180.0 / M_PI) * theta0;
        pImpl->observationBuffer[PoleVelocityObservationIndex] = v0;
    }

    return true;
}

bool CartPole::setAction(const EnvironmentCallbacks::Action& action)
{
    std::lock_guard lock(pImpl->mutex);

    // This method is called only once every iteration.
    // It is the right place where to increase the counter.
    pImpl->iterations++;

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

std::optional<gympp::gazebo::EnvironmentCallbacks::Reward> CartPole::computeReward()
{
    std::lock_guard lock(pImpl->mutex);
    return 1.0;
}

std::optional<gympp::gazebo::EnvironmentCallbacks::Observation> CartPole::getObservation()
{
    std::lock_guard lock(pImpl->mutex);
    return Observation(pImpl->observationBuffer);
}

IGNITION_ADD_PLUGIN(gympp::plugins::CartPole,
                    gympp::plugins::CartPole::System,
                    gympp::plugins::CartPole::ISystemConfigure,
                    gympp::plugins::CartPole::ISystemPreUpdate,
                    gympp::plugins::CartPole::ISystemPostUpdate,
                    gympp::gazebo::EnvironmentCallbacks)
