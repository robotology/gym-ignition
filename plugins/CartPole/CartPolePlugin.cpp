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
#include "gympp/gazebo/TaskSingleton.h"

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
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

    std::string modelName;
    gympp::RobotPtr robot;
    static gympp::RobotPtr getRobotPtr(const std::string& robotName);

    double getRandomThetaInRad()
    {
        std::uniform_real_distribution<> distr(-MaxTheta0Rad, MaxTheta0Rad);
        return distr(gympp::Random::engine());
    }
};

gympp::RobotPtr CartPole::Impl::getRobotPtr(const std::string& robotName)
{
    // Get the robot interface
    auto robotPtr = RobotSingleton::get().getRobot(robotName).lock();

    // Check that is not a nullptr
    if (!robotPtr) {
        gymppError << "Failed to get the robot '" << robotName << "' from the singleton"
                   << std::endl;
        return {};
    }

    if (!robotPtr->valid()) {
        gymppError << "The robot interface is not valid" << std::endl;
        return {};
    }

    return robotPtr;
}

// ========
// CARTPOLE
// ========

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
    if (!TaskSingleton::get().removeTask(pImpl->robot->name())) {
        gymppError << "Failed to unregister the Task interface";
        assert(false);
    }
}

void CartPole::Configure(const ignition::gazebo::Entity& entity,
                         const std::shared_ptr<const sdf::Element>& /*sdf*/,
                         ignition::gazebo::EntityComponentManager& ecm,
                         ignition::gazebo::EventManager& /*eventMgr*/)
{
    // Create a model and check its validity
    auto model = ignition::gazebo::Model(entity);
    if (!model.Valid(ecm)) {
        gymppError << "The entity of the parent model of the gympp plugin is not valid"
                   << std::endl;
        assert(false);
        return;
    }

    // Get the model name and ask the robot interface
    pImpl->modelName = model.Name(ecm);

    // Auto-register the task
    gymppDebug << "Registering the Task interface for robot '" << pImpl->modelName << "'"
               << std::endl;
    auto& taskSingleton = TaskSingleton::get();

    if (!taskSingleton.storeTask(pImpl->modelName, dynamic_cast<gympp::gazebo::Task*>(this))) {
        gymppError << "Failed to register the Task interface";
        assert(false);
        return;
    }
}

void CartPole::PreUpdate(const ignition::gazebo::UpdateInfo& info,
                         ignition::gazebo::EntityComponentManager& /*ecm*/)
{
    if (info.paused) {
        return;
    }

    // During the process of model creation, this plugin is loaded right after the creation of all
    // the entities related to links, joints, etc, and before storing the robot interface in the
    // singleton. This means that asking for the robot interface during the Configure step is still
    // early. We do it lazily here at the first PreUpdate call.
    if (!pImpl->robot) {
        pImpl->robot = pImpl->getRobotPtr(pImpl->modelName);
        assert(pImpl->robot);
    }

    // std::cerr << std::chrono::duration_cast<std::chrono::microseconds>(info.dt).count() << " ms"
    //           << std::endl;

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

bool CartPole::resetTask()
{
    if (!pImpl->robot) {
        pImpl->robot = pImpl->getRobotPtr(pImpl->modelName);
        assert(pImpl->robot);
    }

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

bool CartPole::setAction(const Task::Action& action)
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

std::optional<gympp::gazebo::Task::Reward> CartPole::computeReward()
{
    std::lock_guard lock(pImpl->mutex);
    return 1.0;
    //    return 1.0 - std::abs(pImpl->observationBuffer[ObservationIndex::CartPosition])
    //           - std::abs(pImpl->observationBuffer[ObservationIndex::CartVelocity])
    //           - std::abs(pImpl->observationBuffer[ObservationIndex::PoleVelocity]);
}

std::optional<gympp::gazebo::Task::Observation> CartPole::getObservation()
{
    std::lock_guard lock(pImpl->mutex);
    return Observation(pImpl->observationBuffer);
}

IGNITION_ADD_PLUGIN(gympp::plugins::CartPole,
                    gympp::plugins::CartPole::System,
                    gympp::plugins::CartPole::ISystemConfigure,
                    gympp::plugins::CartPole::ISystemPreUpdate,
                    gympp::plugins::CartPole::ISystemPostUpdate,
                    gympp::gazebo::Task)
