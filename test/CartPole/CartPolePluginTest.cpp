#include "gympp/Gympp.h"
#include "gympp/Log.h"
#include "gympp/common.h"
#include "gympp/gyms/Ignition.h"
#include "gympp/spaces/Space.h"

#include <iostream>

class Gym
{
public:
    static gympp::EnvironmentPtr make(const std::string& envName)
    {
        if (envName == "CartPole") {
            // TODO: find file in the fs
            std::string sdfFile =
                "/home/dferigo/git/gym-ignition/models/CartPole/CartPoleWorld.sdf";

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
};

using namespace gympp;

int main(int /*argc*/, char* /*argv*/[])
{
    auto env = Gym::make("CartPole");

    auto observation = env->reset();
    auto reward = Environment::Reward(0);

    Environment::State oldState;

    // TODO: command line option for setting the render mode
    if (!env->render(Environment::RenderMode::HUMAN)) {
        gymppError << "Failed to render the environment" << std::endl;
        return EXIT_FAILURE;
    }

    // Initialize the seed
    // TODO: command line option for reproducible simulation
    env->seed();
    // env->seed(42);

    size_t epoch = 0;
    size_t iteration = 1;

    while (true) {
        std::cout << "#" << epoch << "." << iteration++ << " " << std::flush;

        // Process oldState to obtain the action.
        // Here we use a random action to bypass it.
        auto actionSample = env->action_space->sample();

        // Simulate the system with the given action
        auto state = env->step(actionSample);

        if (!state.has_value()) {
            gymppError << "The environment didn't return the state" << std::endl;
            return EXIT_FAILURE;
        }

        // Print the observation
        if (auto* o = state->observation.get<double>(); o) {
            for (const auto el : *o) {
                std::cout << el << " ";
            }
            std::cout << std::endl << std::flush;
        }
        else {
            gymppError << "The environment didn't return the observation" << std::endl;
            return EXIT_FAILURE;
        }

        // Cumulate the reward
        reward += state->reward;

        // Save the old state
        oldState = std::move(state.value());

        // Handle termination
        if (state->done) {
            gymppDebug << "The environment reached the terminal state" << std::endl;
            //            break;
            auto newObservation = env->reset();
            if (!newObservation) {
                gymppError << "Failed to reset the environment" << std::endl;
                return EXIT_FAILURE;
            }
            else {
                epoch++;
                iteration = 0;
                oldState.observation = newObservation.value();
                oldState.reward = 0;
                oldState.done = false;
                gymppMessage << "Resetting the environment" << std::endl;
                continue;
            }
        }
    }

    return EXIT_SUCCESS;
}
