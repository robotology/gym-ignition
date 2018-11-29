#include "CartPole.h"

#include "gympp/spaces/Space.h"

using namespace gympp::env;

class CartPole::Impl
{
public:
};

using OSpace = gympp::spaces::Box;
using ASpace = gympp::spaces::Discrete;

CartPole::CartPole(const std::string& sdfFile, double updateRate, uint64_t iterations)
    //    : gyms::IgnitionGazebo(
    : gyms::IgnitionGazebo<AType, OType>(
          std::make_shared<ASpace>(2),
          //                                         std::make_shared<ASpace>(2),
          std::make_shared<OSpace>(OSpace::Limit{-90, 0}, OSpace::Limit{90, 10}), // TODO
          sdfFile,
          updateRate,
          iterations)
    , pImpl{new Impl(), [](Impl* impl) { delete impl; }}
{
    //    auto a = OSpace::Limit{-90, 90};
    //    observation_space = {};
}

// CartPole::~CartPole() = default;

bool CartPole::isDone()
{
    //    auto z = std::make_shared<spaces::Discrete>(2);
    //    auto y = z;
    //    auto y = std::static_pointer_cast<gympp::spaces::Space>(z);
    //    auto a = std::static_pointer_cast<gympp::spaces::Space>();
    return false;
}

bool CartPole::setAction(const TypedEnvironmentBehavior::Action& action)
{
    return true;
}

// std::optional<EnvironmentBehavior<AType, OType>::Reward> CartPole::computeReward()
std::optional<CartPole::TypedEnvironmentBehavior::Reward> CartPole::computeReward()
{
    return CartPole::TypedEnvironmentBehavior::Reward{1};
}

std::optional<CartPole::TypedEnvironmentBehavior::Observation> CartPole::getObservation()
{
    auto obs = CartPole::TypedEnvironmentBehavior::Observation{20, 9};
    return obs;
}
