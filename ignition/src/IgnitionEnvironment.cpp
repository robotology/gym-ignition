/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "gympp/gazebo/IgnitionEnvironment.h"
#include "gympp/Log.h"
#include "gympp/Random.h"
#include "gympp/Space.h"
#include "gympp/gazebo/EnvironmentCallbacks.h"
#include "gympp/gazebo/EnvironmentCallbacksSingleton.h"

#include <cassert>
#include <ostream>

using namespace gympp::gazebo;

class IgnitionEnvironment::Impl
{
public:
    size_t id;
    gympp::gazebo::EnvironmentCallbacks* cb = nullptr;
};

// ====================
// IGNITION ENVIRONMENT
// ====================

EnvironmentCallbacks* IgnitionEnvironment::envCallbacks()
{
    if (!pImpl->cb) {
        auto* ecSingleton = EnvironmentCallbacksSingleton::Instance();
        auto modelNames = getModelNames();
        assert(modelNames.size() == 1);
        std::string scopedModelName = modelNames.front();
        pImpl->cb = ecSingleton->get(scopedModelName);
        assert(pImpl->cb);
    }

    return pImpl->cb;
}

IgnitionEnvironment::IgnitionEnvironment(const ActionSpacePtr aSpace,
                                         const ObservationSpacePtr oSpace,
                                         double updateRate,
                                         uint64_t iterations)
    : Environment(aSpace, oSpace)
    , GazeboWrapper(updateRate, iterations)
    , pImpl{new IgnitionEnvironment::Impl, [](Impl* impl) { delete impl; }}
{}

std::optional<IgnitionEnvironment::State> IgnitionEnvironment::step(const Action& action)
{
    assert(action_space);
    assert(observation_space);

    // Check if the gazebo server is running. It reset() is executed as first method,
    // the server is initialized lazily.
    if (!GazeboWrapper::initialize()) {
        gymppError << "Failed to either initialize gazebo or gather the server" << std::endl;
        return {};
    }

    // Get the environment callbacks
    auto callbacks = envCallbacks();
    if (!callbacks) {
        gymppError << "Failed to get the environment callbacks from the plugin" << std::endl;
        return {};
    }

    if (!this->action_space->contains(action)) {
        gymppError << "The input action does not belong to the action space" << std::endl;
        return {};
    }

    // Set the action to the environment
    if (!callbacks->setAction(action)) {
        gymppError << "Failed to set the action" << std::endl;
        return {};
    }

    // TODO rename method
    if (!GazeboWrapper::run()) {
        gymppError << "Failed to step gazebo" << std::endl;
        return {};
    }

    // Get the observation from the environment
    std::optional<Observation> observation = callbacks->getObservation();

    if (!observation) {
        gymppError << "The gympp plugin didn't return the observation" << std::endl;
        return {};
    }

    if (!this->observation_space->contains(observation.value())) {
        gymppError << "The returned observation does not belong to the observation space"
                   << std::endl;
        return {};
    }

    // Get the reward from the environment
    std::optional<Reward> reward = callbacks->computeReward();

    if (!reward) {
        gymppError << "The gympp plugin didn't return the reward" << std::endl;
        return {};
    }

    if (!this->reward_range.contains(reward.value())) {
        gymppError << "The returned reward (" << reward.value()
                   << ") does not belong to the reward space" << std::endl;
        return {};
    }

    return IgnitionEnvironment::State{callbacks->isDone(), {}, reward.value(), observation.value()};
}

std::vector<size_t> IgnitionEnvironment::seed(size_t seed)
{
    if (seed != 0) {
        gympp::Random::setSeed(seed);
    }

    return {seed};
}

gympp::EnvironmentPtr IgnitionEnvironment::env()
{
    return shared_from_this();
}

std::optional<IgnitionEnvironment::Observation> IgnitionEnvironment::reset()
{
    // Check if the gazebo server is running. It reset() is executed as first method,
    // the server is initialized lazily.
    if (!GazeboWrapper::initialize()) {
        gymppError << "Failed to either initialize gazebo or gather the server" << std::endl;
        return {};
    }

    // Get the environment callbacks
    auto* callbacks = envCallbacks();
    if (!callbacks) {
        gymppError << "Failed to get the environment callbacks from the plugin" << std::endl;
        return {};
    }

    if (!callbacks->reset()) {
        gymppError << "Failed to reset plugin" << std::endl;
        return {};
    }

    gymppDebug << "Retrieving the initial observation after reset" << std::endl;
    return callbacks->getObservation();
}

bool IgnitionEnvironment::render(RenderMode mode)
{
    gymppDebug << "Rendering the environment" << std::endl;

    if (mode == RenderMode::HUMAN) {
        return GazeboWrapper::gui();
    }

    return false;
}
