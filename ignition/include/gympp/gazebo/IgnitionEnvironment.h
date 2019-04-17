/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef GYMPP_GYMS_IGNITION
#define GYMPP_GYMS_IGNITION

#include "gympp/Environment.h"

#include <array>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#ifdef NDEBUG
#define DEFAULT_VERBOSITY 2
#else
#define DEFAULT_VERBOSITY 4
#endif

namespace gympp {
    namespace gazebo {
        class IgnitionEnvironment;
    } // namespace gazebo
} // namespace gympp

class gympp::gazebo::IgnitionEnvironment
    : public gympp::Environment
    , public std::enable_shared_from_this<gympp::Environment>
{
private:
    class Impl;
    std::unique_ptr<Impl, std::function<void(Impl*)>> pImpl = nullptr;

protected:
public:
    using Environment = gympp::Environment;
    using Environment::Action;
    using Environment::Observation;
    using Environment::RenderMode;
    using Environment::Reward;
    using Environment::State;

    static size_t EnvironmentId;

    IgnitionEnvironment() = delete;
    IgnitionEnvironment(const ActionSpacePtr aSpace,
                        const ObservationSpacePtr oSpace,
                        double updateRate,
                        uint64_t iterations = 1);
    ~IgnitionEnvironment() override;

    bool render(RenderMode mode) override;
    std::optional<Observation> reset() override;
    std::optional<State> step(const Action& action) override;
    std::vector<size_t> seed(size_t seed = 0) override;

    // Public APIs
    EnvironmentPtr env();
    static void setVerbosity(int level = DEFAULT_VERBOSITY);

    bool setupGazeboModel(const std::string& modelFile,
                          std::array<double, 6> pose = {0, 0, 0, 0, 0, 0});
    bool setupGazeboWorld(const std::string& worldFile);

    bool setupIgnitionPlugin(const std::string& libName, const std::string& className);
};

#endif // GYMPP_GYMS_IGNITION
