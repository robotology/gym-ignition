/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "gympp/gazebo/GazeboEnvironment.h"
#include "gympp/base/Log.h"
#include "gympp/base/Random.h"
#include "gympp/base/Space.h"
#include "gympp/base/Task.h"
#include "gympp/base/TaskSingleton.h"
#include "scenario/gazebo/Model.h"
#include "scenario/gazebo/World.h"
#include "scenario/gazebo/utils.h"

#include <sdf/Element.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>

#include <cassert>
#include <cmath>
#include <ostream>

using namespace gympp::gazebo;

class GazeboEnvironment::Impl
{
public:
    size_t id;
    gympp::base::Task* task = nullptr;
    scenario::gazebo::WorldPtr world = nullptr;

    PluginData pluginData;
    ModelInitData modelData;

    std::string modelName;
    static std::string getUniqueModelName(const std::string& modelName,
                                          const std::string& modelFile);
};

gympp::base::Task* GazeboEnvironment::getTask()
{
    if (!pImpl->task) {
        auto& taskSingleton = base::TaskSingleton::get();
        pImpl->task = taskSingleton.getTask(pImpl->modelName);
    }

    return pImpl->task;
}

void GazeboEnvironment::storeModelData(const ModelInitData& modelData)
{
    pImpl->modelData = modelData;
}

void GazeboEnvironment::storePluginData(const PluginData& pluginData)
{
    pImpl->pluginData = pluginData;
}

GazeboEnvironment::GazeboEnvironment(const ActionSpacePtr aSpace,
                                     const ObservationSpacePtr oSpace,
                                     const double agentUpdateRate,
                                     const double realTimeFactor,
                                     const double physicsUpdateRate)
    : Environment(aSpace, oSpace)
    , GazeboSimulator(
          1 / physicsUpdateRate,
          realTimeFactor,
          static_cast<unsigned>(physicsUpdateRate / agentUpdateRate))
    , pImpl{std::make_unique<GazeboEnvironment::Impl>()}
{
    gymppDebug << "Configuring gazebo for an agent running at "
               << agentUpdateRate << " Hz" << std::endl;

    // Update the number of iterations accordingly to the simulation step and
    // plugin update
    double rateRatio = physicsUpdateRate / agentUpdateRate;
    auto numOfSimulationIterationsPerStep = static_cast<size_t>(rateRatio);

    if (fmod(rateRatio, 1.0) != 0.0) {
        gymppWarning << "Rounding the number of iterations to "
                     << numOfSimulationIterationsPerStep << " from the nominal "
                     << rateRatio << std::endl;
    }
}

GazeboEnvironment::~GazeboEnvironment() = default;

std::optional<GazeboEnvironment::State>
GazeboEnvironment::step(const Action& action)
{
    assert(action_space);
    assert(observation_space);

    // Check if the gazebo server is running. If reset() is executed as first
    // method, the server is initialized lazily.
    if (!this->initializeSimulation()) {
        gymppError << "Failed to initialize the simulation" << std::endl;
        assert(false);
        return {};
    }

    // Get the task
    auto task = this->getTask();
    if (!task) {
        gymppError << "Failed to get the Task interface from the plugin"
                   << std::endl;
        return {};
    }

    if (!this->action_space->contains(action)) {
        gymppError << "The input action does not belong to the action space"
                   << std::endl;
        return {};
    }

    // Set the action to the environment
    if (!task->setAction(action)) {
        gymppError << "Failed to set the action" << std::endl;
        return {};
    }

    if (!this->run()) {
        gymppError << "Failed to step gazebo" << std::endl;
        return {};
    }

    // Get the observation from the environment
    std::optional<Observation> observation = task->getObservation();

    if (!observation) {
        gymppError << "The gympp plugin didn't return the observation"
                   << std::endl;
        return {};
    }

    if (!this->observation_space->contains(observation.value())) {
        gymppError << "The returned observation does not belong to the "
                      "observation space"
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

    return GazeboEnvironment::State{
        task->isDone(), {}, reward.value(), observation.value()};
}

std::vector<size_t> GazeboEnvironment::seed(size_t seed)
{
    if (seed != 0) {
        gympp::base::Random::setSeed(seed);
    }

    return {seed};
}

gympp::base::EnvironmentPtr GazeboEnvironment::env()
{
    return shared_from_this();
}

std::optional<GazeboEnvironment::Observation> GazeboEnvironment::reset()
{
    // Check if the gazebo server is running. If reset() is executed as first
    // method, the server is initialized lazily.
    if (!this->initializeSimulation()) {
        gymppError << "Failed to initialize the simulation" << std::endl;
        assert(false);
        return {};
    }

    // Get the world
    pImpl->world = this->getWorld();

    if (!pImpl->world) {
        gymppError << "Failed to get the world" << std::endl;
        return {};
    }

    const auto modelNames = pImpl->world->modelNames();
    auto it = std::find(modelNames.begin(), modelNames.end(), pImpl->modelName);

    if (it != modelNames.end()) {

        auto& taskSingleton = base::TaskSingleton::get();

        // Unregister the task
        if (!taskSingleton.removeTask(pImpl->modelName)) {
            gymppError << "Failed to remove the task" << std::endl;
            return {};
        }

        // Remove the model
        if (!pImpl->world->removeModel(pImpl->modelName)) {
            gymppError << "Failed to remove the model from the world"
                       << std::endl;
            return {};
        }

        pImpl->task = nullptr;
        this->run(/*paused=*/true);
    }

    // Get an unique model name
    pImpl->modelName = pImpl->getUniqueModelName(pImpl->modelData.modelName,
                                                 pImpl->modelData.modelFile);

    if (pImpl->modelName.empty()) {
        gymppError << "Failed to compute a unique model name" << std::endl;
        return {};
    }

    // Insert the model in the world
    // pImpl->modelData.modelFile
    if (!pImpl->world->insertModel(
            pImpl->modelData.modelFile,
            {pImpl->modelData.position, pImpl->modelData.orientation},
            pImpl->modelName)) {
        gymppError << "Failed to insert the model while resetting "
                   << "the environment" << std::endl;
        return {};
    }

    // Get the model
    auto model = pImpl->world->getModel(pImpl->modelName);

    if (!model) {
        gymppError << "Failed to get the model" << std::endl;
    }

    // Insert the model plugin
    if (!model->insertModelPlugin(pImpl->pluginData.libName,
                                  pImpl->pluginData.className)) {
        gymppError << "Failed to insert model plugin" << std::endl;
        return {};
    }

    // Get the task
    auto* task = this->getTask();
    if (!task) {
        gymppError << "Failed to get the Task interface from the plugin"
                   << std::endl;
        return {};
    }

    if (!task->resetTask()) {
        gymppError << "Failed to reset plugin" << std::endl;
        return {};
    }

    gymppDebug << "Retrieving the initial observation after reset" << std::endl;
    return task->getObservation();
}

bool GazeboEnvironment::render(RenderMode mode)
{
    gymppDebug << "Rendering the environment" << std::endl;

    // Check if the gazebo server is running. If render() is executed as first
    // method, the server is initialized lazily.
    if (!this->initializeSimulation()) {
        gymppError << "Failed to initialize the simulation" << std::endl;
        assert(false);
        return {};
    }

    if (mode == RenderMode::HUMAN) {
        return GazeboSimulator::gui();
    }

    return false;
}

bool GazeboEnvironment::initializeSimulation()
{
    // If the server is already running it means that the simulation has been
    // already initialized
    if (this->initialized()) {
        return true;
    }

    gymppDebug << "Initializing the simulation" << std::endl;

    // Initialize gazebo and load the world file
    if (!this->initialize()) {
        gymppError << "Failed to either initialize gazebo or gather the server"
                   << std::endl;
        return false;
    }

    gymppDebug << "Simulation initialized" << std::endl;
    return true;
}

std::string
GazeboEnvironment::Impl::getUniqueModelName(const std::string& modelName,
                                            const std::string& modelFile)
{
    std::string prefix = scenario::gazebo::utils::getRandomString(8);

    std::string modelNameWithoutPrefix = modelName;

    if (modelNameWithoutPrefix.empty()) {
        modelNameWithoutPrefix =
            scenario::gazebo::utils::getModelNameFromSdf(modelFile);
    }

    return prefix + "::" + modelNameWithoutPrefix;
}
