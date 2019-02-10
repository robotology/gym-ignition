#include "gympp/GymFactory.h"
#include "gympp/gyms/Ignition.h"

gympp::EnvironmentPtr gympp::GymFactory::make(const std::__cxx11::string& envName)
{
    if (envName == "CartPole") {
        // TODO: find file in the fs
        std::string sdfFile = "/home/dferigo/git/gym-ignition/models/CartPole/CartPoleWorld.sdf";

        using OSpace = gympp::spaces::Box;
        using ASpace = gympp::spaces::Discrete;

        auto ignGym = std::make_shared<gympp::gyms::IgnitionGazebo>(
            "libCartPolePlugin.so",
            "gympp::plugins::CartPole",
            std::make_shared<ASpace>(3),
            std::make_shared<OSpace>(OSpace::Limit{-90, -1}, OSpace::Limit{90, 1}),
            sdfFile,
            /*updateRate=*/1000,
            /*iterations=*/1);

        return ignGym->env();
    }

    return nullptr;
}
