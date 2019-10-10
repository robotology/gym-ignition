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
#include "gympp/gazebo/Task.h"
#include "gympp/gazebo/TaskSingleton.h"

#include <sdf/Element.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>

#include <cassert>
#include <ostream>

using namespace gympp::gazebo;

class IgnitionEnvironment::Impl
{
public:
    size_t id;
    gympp::gazebo::Task* task = nullptr;

    PluginData pluginData;
    gympp::gazebo::ModelInitData modelData;
};

// ====================
// IGNITION ENVIRONMENT
// ====================

bool IgnitionEnvironment::initializeSimulation()
{
    // If the server is already running it means that the simulation has been already initialized
    if (GazeboWrapper::initialized()) {
        return true;
    }

    gymppDebug << "Initializing the simulation" << std::endl;

    // Initialize gazebo and load the world file
    if (!GazeboWrapper::initialize()) {
        gymppError << "Failed to either initialize gazebo or gather the server" << std::endl;
        return false;
    }

    // Use the pointer to this object (which is unique and for sure is not shared with any other
    // environment objects) as prefix.
    std::string prefix = std::to_string(reinterpret_cast<int64_t>(this));

    // Initialize the model name with the name specified in the initialization data
    std::string desiredModelNameWithoutPrefix = pImpl->modelData.modelName;

    // Use the name from the SDF if the initialization data does not specify it
    if (desiredModelNameWithoutPrefix.empty()) {
        // Get the name from the model SDF
        desiredModelNameWithoutPrefix = getModelNameFromSDF(pImpl->modelData.sdfString);

        if (desiredModelNameWithoutPrefix.empty()) {
            gymppError << "Failed to extract the model name from the model SDF" << std::endl;
            return false;
        }
    }

    // Final model name.
    // Note that the real name substitutin will be done by the insertModel() method.
    pImpl->modelData.modelName = prefix + "::" + desiredModelNameWithoutPrefix;

    // Insert the model in the world
    if (!insertModel(pImpl->modelData, pImpl->pluginData)) {
        gymppError << "Failed to insert the model while resetting the environment" << std::endl;
        return false;
    }

    gymppDebug << "Simulation initialized" << std::endl;
    return true;
}

Task* IgnitionEnvironment::getTask()
{
    if (!pImpl->task) {
        auto& taskSingleton = TaskSingleton::get();
        pImpl->task = taskSingleton.getTask(pImpl->modelData.modelName);
    }

    return pImpl->task;
}

void IgnitionEnvironment::storeModelData(const gympp::gazebo::ModelInitData& modelData)
{
    pImpl->modelData = modelData;
}

void IgnitionEnvironment::storePluginData(const PluginData& pluginData)
{
    pImpl->pluginData = pluginData;
}

IgnitionEnvironment::IgnitionEnvironment(const ActionSpacePtr aSpace,
                                         const ObservationSpacePtr oSpace,
                                         const double agentUpdateRate,
                                         const double realTimeFactor,
                                         const double physicsUpdateRate)
    : Environment(aSpace, oSpace)
    , GazeboWrapper(static_cast<unsigned>(physicsUpdateRate / agentUpdateRate),
                    realTimeFactor,
                    physicsUpdateRate)
    , pImpl{new IgnitionEnvironment::Impl, [](Impl* impl) { delete impl; }}
{
    gymppDebug << "Configuring gazebo for an agent running at " << agentUpdateRate << " Hz"
               << std::endl;

    // Update the number of iterations accordingly to the simulation step and plugin update
    double rateRatio = physicsUpdateRate / agentUpdateRate;
    auto numOfSimulationIterationsPerStep = static_cast<size_t>(rateRatio);

    if (rateRatio != numOfSimulationIterationsPerStep) {
        gymppWarning << "Rounding the number of iterations to " << numOfSimulationIterationsPerStep
                     << " from the nominal " << rateRatio << std::endl;
    }
}

std::optional<IgnitionEnvironment::State> IgnitionEnvironment::step(const Action& action)
{
    assert(action_space);
    assert(observation_space);

    // Check if the gazebo server is running. If reset() is executed as first method,
    // the server is initialized lazily.
    if (!initializeSimulation()) {
        gymppError << "Failed to initialize the simulation" << std::endl;
        assert(false);
        return {};
    }

    // Get the task
    auto task = getTask();
    if (!task) {
        gymppError << "Failed to get the Task interface from the plugin" << std::endl;
        return {};
    }

    if (!this->action_space->contains(action)) {
        gymppError << "The input action does not belong to the action space" << std::endl;
        return {};
    }

    // Set the action to the environment
    if (!task->setAction(action)) {
        gymppError << "Failed to set the action" << std::endl;
        return {};
    }

    // TODO rename method
    if (!GazeboWrapper::run()) {
        gymppError << "Failed to step gazebo" << std::endl;
        return {};
    }

    // Get the observation from the environment
    std::optional<Observation> observation = task->getObservation();

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
    std::optional<Reward> reward = task->computeReward();

    if (!reward) {
        gymppError << "The gympp plugin didn't return the reward" << std::endl;
        return {};
    }

    if (!this->reward_range.contains(reward.value())) {
        gymppError << "The returned reward (" << reward.value()
                   << ") does not belong to the reward space" << std::endl;
        return {};
    }

    return IgnitionEnvironment::State{task->isDone(), {}, reward.value(), observation.value()};
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
    // Check if the gazebo server is running. If reset() is executed as first method,
    // the server is initialized lazily.
    if (!initializeSimulation()) {
        gymppError << "Failed to initialize the simulation" << std::endl;
        assert(false);
        return {};
    }

    // Get the task
    auto* task = getTask();
    if (!task) {
        gymppError << "Failed to get the Task interface from the plugin" << std::endl;
        return {};
    }

    if (!task->resetTask()) {
        gymppError << "Failed to reset plugin" << std::endl;
        return {};
    }

    gymppDebug << "Retrieving the initial observation after reset" << std::endl;
    return task->getObservation();
}

bool IgnitionEnvironment::render(RenderMode mode)
{
    gymppDebug << "Rendering the environment" << std::endl;

    // Check if the gazebo server is running. If render() is executed as first method,
    // the server is initialized lazily.
    if (!initializeSimulation()) {
        gymppError << "Failed to initialize the simulation" << std::endl;
        assert(false);
        return {};
    }

    if (mode == RenderMode::HUMAN) {
        return GazeboWrapper::gui();
    }

    return false;
}
