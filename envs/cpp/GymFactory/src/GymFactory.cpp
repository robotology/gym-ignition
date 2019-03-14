#include "gympp/GymFactory.h"
#include "gympp/gyms/Ignition.h"

gympp::EnvironmentPtr gympp::GymFactory::make(const std::__cxx11::string& envName)
{
    if (envName == "CartPole") {
        std::string sdfFile = "CartPoleWorld.sdf";

        using OSpace = gympp::spaces::Box;
        using ASpace = gympp::spaces::Discrete;

        // Create the gym environment
        auto ignGym = std::make_shared<gympp::gyms::IgnitionGazebo>(
            std::make_shared<ASpace>(3),
            std::make_shared<OSpace>(OSpace::Limit{-90, -1}, OSpace::Limit{90, 1}),
            sdfFile,
            /*updateRate=*/1000,
            /*iterations=*/1);

        // Setup the CartPolePlugin
        ignGym->setupIgnitionPlugin("CartPolePlugin", "gympp::plugins::CartPole");

        return ignGym->env();
    }

    return nullptr;
}
