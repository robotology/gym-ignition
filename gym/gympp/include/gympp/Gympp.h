#ifndef GYMPP_GYMPP
#define GYMPP_GYMPP

#include <memory>
#include <optional>
#include <string>
#include <valarray>
#include <vector>

#include "gympp/common.h"
#include "gympp/spaces/Space.h"

namespace gympp {

    class Environment;
    using EnvironmentName = std::string;

    namespace sar {
        struct State;

        using Action = data::Sample;
        using Observation = data::Sample;

        using Reward = float;
        using RewardRange = Range<float>;
    } // namespace sar

} // namespace gympp

// template <typename OType>
struct gympp::sar::State
{
    // public:
    bool done;
    std::string info;
    gympp::sar::Reward reward;
    gympp::sar::Observation observation;
};

// TODO: https://hub.packtpub.com/openai-gym-environments-wrappers-and-monitors-tutorial/
//       These C++ and their mapping to python / julia should allow using the Wrapper method

// From https://github.com/openai/gym/blob/master/gym/core.py
// template <typename AType, typename OType>
class gympp::Environment
{
public:
    using State = gympp::sar::State;
    using Action = gympp::sar::Action;
    using Reward = gympp::sar::Reward;
    using Observation = gympp::sar::Observation;

    using ActionSpace = gympp::spaces::Space;
    using ObservationSpace = gympp::spaces::Space;
    using ActionSpacePtr = std::shared_ptr<ActionSpace>;
    using ObservationSpacePtr = std::shared_ptr<ObservationSpace>;

    ActionSpacePtr action_space;
    ObservationSpacePtr observation_space;
    gympp::sar::RewardRange reward_range;

    Environment() = delete;
    Environment(const ActionSpacePtr aSpace, const ObservationSpacePtr oSpace)
        : action_space(aSpace)
        , observation_space(oSpace)
    {}

    virtual ~Environment() = default;

    enum class RenderMode
    {
        HUMAN,
        RGB_ARRAY,
        ANSI,
    };

    // The registration should have the "make" method
    // https://github.com/openai/gym/blob/10e53654cd5b9e49a5f689ada339f1cd32843e43/gym/envs/registration.py
    // virtual std::shared_ptr<gympp::Environment> make(const gympp::EnvironmentName& name) = 0;

    virtual Environment* env() = 0;
    virtual std::optional<State> step(const Action& action) = 0;
    virtual std::optional<Observation> reset() = 0;
    virtual bool render(RenderMode mode) = 0;
    // TODO: close()
    // TODO: seed()
};

#endif // GYMPP_GYMPP
