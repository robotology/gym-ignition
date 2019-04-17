/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "gympp/Common.h"
#include "gympp/Environment.h"
#include "gympp/GymFactory.h"
#include "gympp/Log.h"
#include "gympp/PluginDatabase.h"
#include "gympp/Space.h"

#include "clara.hpp"

#include <ignition/common/SignalHandler.hh>

#include <cassert>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace gympp;
using namespace clara;

struct Config
{
    bool help = false;
    bool gui = false;
    std::optional<size_t> seed;
};

int main(int argc, char* argv[])
{
    // ==================
    // PARSE COMMAND LINE
    // ==================

    Config config;

    // Create the command line parser
    auto cli = Help(config.help) | Opt(config.gui)["-g"]["--gui"]("render the environment")
               | Opt([&](unsigned value) { config.seed = value; },
                     "seed")["-s"]["--seed"]("use a specific seed for randomness");

    // Parse the command line
    if (auto result = cli.parse(Args(argc, argv)); !result) {
        gymppError << "Error in command line: " << result.errorMessage() << std::endl;
        exit(EXIT_FAILURE);
    }

    if (config.help) {
        std::cout << cli;
        exit(EXIT_SUCCESS);
    }

    // ==========================
    // INITIALIZE THE ENVIRONMENT
    // ==========================

    // Create the environment
    auto env = GymFactory::Instance()->make("CartPole");

    if (!env) {
        gymppError << "Failed to load the CartPole environment" << std::endl;
        return EXIT_FAILURE;
    }

    // Initialize the signal handler
    ignition::common::SignalHandler sigHandler;
    assert(sigHandler.Initialized());
    sigHandler.AddCallback([&](const int /*_sig*/) {
        gymppDebug << "Shutting down gracefully" << std::endl;
        env.reset();
        exit(EXIT_FAILURE);
    });

    // Initialize the seed
    if (config.seed) {
        env->seed(config.seed.value());
    }

    // Reset the environment
    auto reward = Environment::Reward(0);
    auto observation = env->reset();

    if (!observation) {
        gymppError << "Failed to retrieve the initial observation" << std::endl;
        return EXIT_FAILURE;
    }

    // Create the initial state object
    Environment::State oldState;
    oldState.done = false;
    oldState.observation = observation.value();

    // Render the environment
    if (config.gui && !env->render(Environment::RenderMode::HUMAN)) {
        gymppError << "Failed to render the environment" << std::endl;
        return EXIT_FAILURE;
    }

    // ===============
    // SIMULATION LOOP
    // ===============

    size_t epoch = 1;
    size_t iteration = 0;

    while (epoch <= 100) {
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
            std::cout << "[" << *actionSample.get<int>(0) << "]\t";
            for (const auto el : *o) {
                std::cout.setf(std::ios::fixed);
                std::cout.precision(6);
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
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

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
                gymppDebug << "Resetting the environment" << std::endl;
                continue;
            }
        }
    }

    return EXIT_SUCCESS;
}
