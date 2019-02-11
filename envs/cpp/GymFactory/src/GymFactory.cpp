#include "gympp/GymFactory.h"
#include "gympp/gyms/Ignition.h"

#include <ignition/common/Filesystem.hh>
#include <ignition/gazebo/SystemLoader.hh>

gympp::EnvironmentPtr gympp::GymFactory::make(const std::__cxx11::string& envName)
{
    ignition::gazebo::SystemLoader sl;

    if (envName == "CartPole") {
        // Add the plugin folder to the system search path
        sl.AddSystemPluginPath(CARTPOLE_PLUGIN_PATH);

        // TODO: find file in the fs
        std::string sdfFile = "/home/dferigo/git/gym-ignition/models/CartPole/CartPoleWorld.sdf";

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
