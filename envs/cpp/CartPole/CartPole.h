#ifndef GYMPP_ENVS_CARTPOLE
#define GYMPP_ENVS_CARTPOLE

#include "gympp/Gympp.h"
#include "gympp/gyms/Ignition.h"

#include <memory>

namespace gympp {
    namespace env {
        class CartPole;
    }
} // namespace gympp

// This class is associated with the InvertedPendulum.sdf file

// TODO: how can I have a datatype double and a discrete space? it should not work!

using AType = size_t;
using OType = double;

class gympp::env::CartPole : public gympp::gyms::IgnitionGazebo<AType, OType>
{
private:
    class Impl;
    std::unique_ptr<Impl, void (*)(Impl*)> pImpl;

public:
    CartPole() = delete;
    CartPole(const std::string& sdfFile, double updateRate, uint64_t iterations = 1);
    ~CartPole() override = default;

    bool isDone() override;
    bool setAction(const TypedEnvironmentBehavior::Action& action) override;
    std::optional<TypedEnvironmentBehavior::Reward> computeReward() override;
    std::optional<TypedEnvironmentBehavior::Observation> getObservation() override;
};

#endif // GYMPP_ENVS_CARTPOLE
