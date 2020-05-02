/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "CartPolePlugin.h"
#include "gympp/base/Common.h"
#include "gympp/base/Log.h"
#include "gympp/base/Random.h"
#include "gympp/base/TaskSingleton.h"
#include "scenario/gazebo/Joint.h"
#include "scenario/gazebo/Model.h"
#include "scenario/plugins/gazebo/ECMSingleton.h"

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

using namespace gympp::plugins;

using ActionDataType = int;
using ActionSample = gympp::base::BufferContainer<ActionDataType>::type;

using ObservationDataType = double;
using ObservationSample =
    gympp::base::BufferContainer<ObservationDataType>::type;

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
    ObservationSample observationBuffer;
    std::optional<CartPoleAction> action;

    std::string modelName;
    scenario::gazebo::ModelPtr model;

    inline double getRandomThetaInRad()
    {
        std::uniform_real_distribution<> distr(-MaxTheta0Rad, MaxTheta0Rad);
        return distr(gympp::base::Random::engine());
    }
};

// ========
// CARTPOLE
// ========

CartPole::CartPole()
    : System()
    , pImpl{new Impl()}
{
    // - Cart linear position [m]
    // - Cart linear velocity [m/s]
    // - Pole angular position [deg]
    // - Pole angular velocity [deg/s]
    pImpl->observationBuffer.resize(4);
}

CartPole::~CartPole()
{
    auto& taskSingleton = base::TaskSingleton::get();

    if (pImpl->model && taskSingleton.hasTask(pImpl->modelName)
        && !taskSingleton.removeTask(pImpl->modelName)) {
        gymppError << "Failed to unregister the Task interface";
        assert(false);
    }
}

void CartPole::Configure(const ignition::gazebo::Entity& entity,
                         const std::shared_ptr<const sdf::Element>& /*sdf*/,
                         ignition::gazebo::EntityComponentManager& ecm,
                         ignition::gazebo::EventManager& eventMgr)
{
    pImpl->model = std::make_shared<scenario::gazebo::Model>();

    if (!pImpl->model->initialize(entity, &ecm, &eventMgr)) {
        gymppError << "Failed to initialize task's model" << std::endl;
        assert(false);
        return;
    }

    // Save the model name
    pImpl->modelName = pImpl->model->name();

    // Auto-register the task
    gymppDebug << "Registering the Task interface for robot '"
               << pImpl->modelName << "'" << std::endl;
    auto& taskSingleton = base::TaskSingleton::get();

    if (!taskSingleton.storeTask(pImpl->modelName,
                                 dynamic_cast<gympp::base::Task*>(this))) {
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

    if (!pImpl->model) {
        return;
    }

    if (!base::TaskSingleton::get().hasTask(pImpl->modelName)) {
        return;
    }

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

            // Reset the action. This allows to perform multiple simulator
            // iterations for each environment step.
            pImpl->action.reset();

            scenario::gazebo::JointPtr linear =
                pImpl->model->getJoint("linear");

            if (!linear) {
                gymppError << "Failed to get 'linear' joint" << std::endl;
                return;
            }

            if (!linear->setGeneralizedForceTarget(appliedForce)) {
                gymppError << "Failed to set the force to joint 'linear'"
                           << std::endl;
                return;
            }
        }
    }
}

void CartPole::PostUpdate(
    const ignition::gazebo::UpdateInfo& info,
    const ignition::gazebo::EntityComponentManager& /*manager*/)
{
    if (info.paused) {
        return;
    }

    if (!pImpl->model) {
        return;
    }

    if (!base::TaskSingleton::get().hasTask(pImpl->modelName)) {
        return;
    }

    auto pivot = pImpl->model->getJoint("pivot");
    auto linear = pImpl->model->getJoint("linear");

    double poleJointPosition = pivot->position();
    double cartJointPosition = linear->position();

    double poleJointVelocity = pivot->velocity();
    double cartJointVelocity = linear->velocity();

    {
        std::lock_guard lock(pImpl->mutex);
        pImpl->observationBuffer[ObservationIndex::CartPosition] =
            cartJointPosition;
        pImpl->observationBuffer[ObservationIndex::CartVelocity] =
            cartJointVelocity;
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
        || std::abs(pImpl->observationBuffer[ObservationIndex::PolePosition])
               > ThetaThresholdDeg
        || std::abs(pImpl->observationBuffer[ObservationIndex::CartPosition])
               > XThreshold) {
        return true;
    }

    return false;
}

bool CartPole::resetTask()
{
    if (!pImpl->model) {
        return false;
    }

    // Reset the number of iterations
    pImpl->iterations = 0;

    // Joint positions
    double x0 = 0.0;
    double v0 = 0.0;
    auto theta0 = pImpl->getRandomThetaInRad();

    auto pivot = pImpl->model->getJoint("pivot");
    auto linear = pImpl->model->getJoint("linear");

    // Reset the pendulum
    if (!pivot->reset(theta0, v0)) {
        gymppError << "Failed to reset the state of joint 'pivot'" << std::endl;
        return false;
    }

    // Reset the cart
    if (!linear->reset(x0, v0)) {
        gymppError << "Failed to reset the state of joint 'linear'"
                   << std::endl;
        return false;
    }

    // Set the control mode
    if (!linear->setControlMode(scenario::base::JointControlMode::Force)) {
        gymppError << "Failed to set the control mode" << std::endl;
        return false;
    }

    {
        // Update the observation. This is required because the
        // Environment::reset() method returns the new observation.
        std::lock_guard lock(pImpl->mutex);

        pImpl->observationBuffer[ObservationIndex::CartPosition] = x0;
        pImpl->observationBuffer[ObservationIndex::CartVelocity] = v0;
        pImpl->observationBuffer[ObservationIndex::PolePosition] =
            (180.0 / M_PI) * theta0;
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

std::optional<gympp::base::Task::Reward> CartPole::computeReward()
{
    std::lock_guard lock(pImpl->mutex);
    return 1.0;
}

std::optional<gympp::base::Task::Observation> CartPole::getObservation()
{
    std::lock_guard lock(pImpl->mutex);
    return Observation(pImpl->observationBuffer);
}

IGNITION_ADD_PLUGIN(gympp::plugins::CartPole,
                    gympp::plugins::CartPole::System,
                    gympp::plugins::CartPole::ISystemConfigure,
                    gympp::plugins::CartPole::ISystemPreUpdate,
                    gympp::plugins::CartPole::ISystemPostUpdate,
                    gympp::base::Task)
