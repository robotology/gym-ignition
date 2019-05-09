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

#include <ignition/plugin/Register.hh>

#include <cassert>
#include <cmath>
#include <mutex>
#include <ostream>
#include <random>
#include <string>

using namespace gympp::gazebo;
using namespace gympp::plugins;

using ActionDataType = int;
using ActionSample = gympp::BufferContainer<ActionDataType>::type;

using ObservationDataType = double;
using ObservationSample = gympp::BufferContainer<ObservationDataType>::type;

enum ObservationIndex
{
    CartPosition = 0,
    CartVelocity = 1,
    PolePosition = 2,
    PoleVelocity = 3,
};

const double DefaultControllerRate = 1000;
const size_t MaxEpisodeLength = 20000;
const double XThreshold = 2.4;
const double ThetaThresholdDeg = 12;
const double AppliedForce = 10;
const double MaxTheta0Rad = 10 * (M_PI / 180.0);

enum CartPoleAction
{
    MOVE_LEFT,
    MOVE_RIGHT,
    DONT_MOVE,
};

class CartPole::Impl
{
public:
    unsigned seed;
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

CartPole::~CartPole()
{
    auto* ecSingleton = EnvironmentCallbacksSingleton::Instance();

    if (bool removed = ecSingleton->deleteEnvironmentCallback(pImpl->robot->name()); !removed) {
        gymppError << "Failed to unregister the environment callbacks";
        assert(removed);
    }
}

void CartPole::Configure(const ignition::gazebo::Entity& entity,
                         const std::shared_ptr<const sdf::Element>& sdf,
                         ignition::gazebo::EntityComponentManager& ecm,
                         ignition::gazebo::EventManager& /*eventMgr*/)
{
    // Create an IgnitionRobot object from the ecm
    auto ignRobot = std::make_shared<gympp::gazebo::IgnitionRobot>();
    if (!ignRobot->configureECM(entity, sdf, ecm)) {
        gymppError << "Failed to configure the Robot interface" << std::endl;
        return;
    }

    // Read the optional update_rate option from the sdf
    if (sdf->HasElement("update_rate")) {
        double rate = sdf->Get<double>("update_rate");
        ignRobot->setdt(std::chrono::duration<double>(1 / rate));
        gymppDebug << "Setting plugin rate " << rate << " Hz" << std::endl;
    }
    else {
        ignRobot->setdt(std::chrono::duration<double>(1 / DefaultControllerRate));
        gymppDebug << "Setting plugin rate " << DefaultControllerRate << " Hz" << std::endl;
    }

    if (!ignRobot->valid()) {
        gymppError << "The Robot interface is not valid" << std::endl;
        return;
    }

    // Store a pointer to gympp::Robot
    pImpl->robot = ignRobot;

    // Auto-register the environment callbacks
    gymppDebug << "Registering environment callbacks for robot '" << ignRobot->name() << "'"
               << std::endl;
    auto* ecSingleton = EnvironmentCallbacksSingleton::Instance();

    if (bool registered = ecSingleton->storeEnvironmentCallback(ignRobot->name(), this);
        !registered) {
        gymppError << "Failed to register the environment callbacks";
        assert(registered);
        return;
    }
}

void CartPole::PreUpdate(const ignition::gazebo::UpdateInfo& info,
                         ignition::gazebo::EntityComponentManager& /*manager*/)
{
    if (info.paused) {
        return;
    }

    assert(pImpl->robot);

    // Actuate the action
    // ==================

    {
        std::lock_guard lock(pImpl->mutex);

        if (pImpl->action) {
            double appliedForce;

            switch (*pImpl->action) {
                case MOVE_LEFT:
                    appliedForce = AppliedForce;
                    break;
                case MOVE_RIGHT:
                    appliedForce = -AppliedForce;
                    break;
                case DONT_MOVE:
                    appliedForce = 0;
                    break;
            }

            // Reset the action. This allows to perform multiple simulator iterations for each
            // environment step.
            pImpl->action.reset();

            if (!pImpl->robot->setJointForce("linear", appliedForce)) {
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

    assert(pImpl->robot);

    double cartJointPosition = pImpl->robot->jointPosition("linear");
    double poleJointPosition = pImpl->robot->jointPosition("pivot");

    double cartJointVelocity = pImpl->robot->jointVelocity("linear");
    double poleJointVelocity = pImpl->robot->jointVelocity("pivot");

    {
        std::lock_guard lock(pImpl->mutex);
        pImpl->observationBuffer[ObservationIndex::CartPosition] = cartJointPosition;
        pImpl->observationBuffer[ObservationIndex::CartVelocity] = cartJointVelocity;
        pImpl->observationBuffer[ObservationIndex::PolePosition] =
            (180.0 / M_PI) * poleJointPosition;
        pImpl->observationBuffer[ObservationIndex::PoleVelocity] =
            (180.0 / M_PI) * poleJointVelocity;
    }
}

bool CartPole::isDone()
{
    std::lock_guard lock(pImpl->mutex);

    if (pImpl->iterations >= MaxEpisodeLength
        || std::abs(pImpl->observationBuffer[ObservationIndex::PolePosition]) > ThetaThresholdDeg
        || std::abs(pImpl->observationBuffer[ObservationIndex::CartPosition]) > XThreshold) {
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
    double x0 = 0.0;
    double v0 = 0.0;
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

    {
        // Update the observation. This is required because the Environment::reset()
        // method returns the new observation.
        std::lock_guard lock(pImpl->mutex);

        pImpl->observationBuffer[ObservationIndex::CartPosition] = x0;
        pImpl->observationBuffer[ObservationIndex::CartVelocity] = v0;
        pImpl->observationBuffer[ObservationIndex::PolePosition] = (180.0 / M_PI) * theta0;
        pImpl->observationBuffer[ObservationIndex::PoleVelocity] = v0;
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
