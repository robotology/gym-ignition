#include <iostream>

#include "CartPole.h"
#include "gympp/Gympp.h"
#include "gympp/Log.h"
#include "gympp/gyms/Ignition.h"

#include <chrono>
#include <thread>
#include <typeindex>
#include <typeinfo>
#include <variant>

int main(int /*argc*/, char* /*argv*/ [])
{

    std::string sdfFile = "/home/dferigo/git/gym-ignition/models/CartPole/CartPoleWorld.sdf";

    auto cartPoleGym = gympp::env::CartPole(sdfFile, /*updateRate=*/10, /*iterations=*/10); // TODO
    auto env = cartPoleGym.env();

    auto observation = env->reset();
    auto reward = gympp::env::CartPole::Reward(0);

    gympp::env::CartPole::State oldState;

    //    if (!env->render(gympp::Environment::RenderMode::HUMAN)) {
    //        gymppError << "Failed to render the environment" << std::endl;
    //        return EXIT_FAILURE;
    //    }

    //    std::chrono::milliseconds period{100};
    //    std::cout << "Period " << period.count() << "ms" << std::endl;

    size_t epoch = 0;

    while (true) {
        std::cout << "#" << epoch++ << " " << std::flush;

        //        auto tick = std::chrono::high_resolution_clock::now();

        // TODO: use oldState to obtain the action

        // Here we use a random action to bypass it
        auto actionSample = env->action_space->sample();
        auto* action = actionSample.get<AType>();
        auto state = env->step(actionSample);

        if (!state.has_value()) {
            gymppError << "The environment didn't return the state" << std::endl;
            return EXIT_FAILURE;
        }

        if (state->done) {
            break;
        }

        // Cumulate the reward
        reward += state->reward;

        // Save the old state
        oldState = std::move(state.value());

        //        auto tock = std::chrono::high_resolution_clock::now();
        //        auto computed = std::chrono::duration_cast<std::chrono::milliseconds>(tock -
        //        tick); auto sleep = period - computed;

        //        std::cout << "[Loop " << computed.count() << "ms]";

        //        if (sleep.count() > 0) {
        //            //            std::cout << sleep.count() << std::endl;
        //            std::this_thread::sleep_for(sleep);
        //        }

        std::cout << std::endl;
    }

    return EXIT_SUCCESS;

    //    auto sample = gympp::data::Sample{action};
    //    auto state = env->step(sample);
}
