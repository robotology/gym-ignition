#include "gympp/Common.h"
#include "gympp/Environment.h"
#include "gympp/GymFactory.h"
#include "gympp/Log.h"
#include "gympp/Space.h"

#include <ignition/common/SignalHandler.hh>

#include <cassert>
#include <iostream>
#include <thread>

using namespace gympp;

int main(int /*argc*/, char* /*argv*/[])
{
    auto env = GymFactory::make("CartPole");

    if (!env) {
        gymppError << "Failed to load the CartPole environment" << std::endl;
        return EXIT_FAILURE;
    }

    ignition::common::SignalHandler sigHandler;
    assert(sigHandler.Initialized());
    sigHandler.AddCallback([&](const int /*_sig*/) {
        gymppDebug << "Shutting down gracefully" << std::endl;
        env.reset();
        exit(EXIT_FAILURE);
    });

    // Initialize the seed
    // TODO: command line option for reproducible simulation
    //    env->seed();
    env->seed(42);

    auto reward = Environment::Reward(0);
    auto observation = env->reset();

    if (!observation) {
        gymppError << "Failed to retrieve the initial observation" << std::endl;
        return EXIT_FAILURE;
    }

    Environment::State oldState;

    // TODO: command line option for setting the render mode
    if (!env->render(Environment::RenderMode::HUMAN)) {
        gymppError << "Failed to render the environment" << std::endl;
        return EXIT_FAILURE;
    }

    size_t epoch = 1;
    size_t iteration = 0;

    while (true) {
        iteration++;

        // Process oldState to obtain the action.
        // Here we use a random action to bypass it.
        auto actionSample = env->action_space->sample();

        // Simulate the system with the given action
        auto state = env->step(actionSample);

        if (!state) {
            gymppError << "The environment didn't return the state" << std::endl;
            return EXIT_FAILURE;
        }

        // Print the observation
        if (auto* o = state->observation.getBuffer<double>(); o) {
            std::cout << "#" << epoch << "." << iteration << "\t";
            for (const auto el : *o) {
                std::cout << el << "\t";
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
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // Reset the environment
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
