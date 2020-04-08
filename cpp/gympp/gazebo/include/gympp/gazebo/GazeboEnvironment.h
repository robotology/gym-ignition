/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef GYMPP_GAZEBO_GAZEBOENVIRONMENT
#define GYMPP_GAZEBO_GAZEBOENVIRONMENT

#include "gympp/base/Environment.h"
#include "gympp/gazebo/Metadata.h"
#include "scenario/gazebo/GazeboSimulator.h"

#include <memory>
#include <optional>
#include <vector>

namespace gympp {
    namespace base {
        class Task;
    } // namespace base
    namespace gazebo {
        class GymFactory;
        class GazeboEnvironment;
    } // namespace gazebo
} // namespace gympp

class gympp::gazebo::GazeboEnvironment
    : public gympp::base::Environment
    , public scenario::gazebo::GazeboSimulator
    , public std::enable_shared_from_this<gympp::base::Environment>
{
public:
    using Environment = gympp::base::Environment;
    using Environment::Action;
    using Environment::Observation;
    using Environment::RenderMode;
    using Environment::Reward;
    using Environment::State;

    GazeboEnvironment() = delete;
    GazeboEnvironment(const ActionSpacePtr aSpace,
                      const ObservationSpacePtr oSpace,
                      const double agentUpdateRate,
                      const double realTimeFactor = 1,
                      const double physicsUpdateRate = 1000);
    ~GazeboEnvironment() override;

    bool render(RenderMode mode) override;
    std::optional<Observation> reset() override;
    std::optional<State> step(const Action& action) override;
    std::vector<size_t> seed(size_t seed = 0) override;

    base::EnvironmentPtr env();

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
    gympp::base::Task* getTask();

    friend class gympp::gazebo::GymFactory;
    bool initializeSimulation();
    void storeSDFModelFile(const std::string& modelSDF);
    void storeModelData(const ModelInitData& modelData);
    void storePluginData(const PluginData& pluginData);
};

#endif // GYMPP_GAZEBO_GAZEBOENVIRONMENT
