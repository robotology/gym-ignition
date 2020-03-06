/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef GYMPP_BASE_ENVIRONMENT
#define GYMPP_BASE_ENVIRONMENT

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "gympp/base/Common.h"
#include "gympp/base/Space.h"

namespace gympp {
    namespace base {
        class Environment;
        using EnvironmentName = std::string;
        using EnvironmentPtr = std::shared_ptr<Environment>;

        struct State;
        using Action = data::Sample;
        using Reward = DataSupport;
        using Observation = data::Sample;
    } // namespace base
} // namespace gympp

struct gympp::base::State
{
    bool done;
    std::string info;
    gympp::base::Reward reward;
    gympp::base::Observation observation;
};

// TODO: https://hub.packtpub.com/openai-gym-environments-wrappers-and-monitors-tutorial/
//       These C++ and their mapping to python / julia should allow using the Wrapper method

// From https://github.com/openai/gym/blob/master/gym/core.py
class gympp::base::Environment
{
public:
    using Action = gympp::base::Action;
    using Reward = gympp::base::Reward;
    using Observation = gympp::base::Observation;
    using State = gympp::base::State;

    enum class RenderMode
    {
        HUMAN,
        RGB_ARRAY,
        ANSI,
    };

    using ActionSpace = gympp::base::spaces::Space;
    using ObservationSpace = gympp::base::spaces::Space;
    using ActionSpacePtr = std::shared_ptr<ActionSpace>;
    using ObservationSpacePtr = std::shared_ptr<ObservationSpace>;
    using RewardRange = gympp::base::Range;

public:
    ActionSpacePtr action_space;
    ObservationSpacePtr observation_space;
    RewardRange reward_range;

public:
    Environment() = delete;
    virtual ~Environment() = default;

    Environment(const ActionSpacePtr aSpace,
                const ObservationSpacePtr oSpace,
                const RewardRange& rRange = {})
        : action_space(aSpace)
        , observation_space(oSpace)
        , reward_range(rRange)
    {}

    virtual std::optional<State> step(const Action& action) = 0;
    virtual std::optional<Observation> reset() = 0;
    virtual bool render(RenderMode mode) = 0;
    virtual std::vector<size_t> seed(size_t seed = 0) = 0;
    // TODO: close()
};

#endif // GYMPP_BASE_ENVIRONMENT
