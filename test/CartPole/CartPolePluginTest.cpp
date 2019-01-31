#include <iostream>

//#include "CartPole.h"
#include "gympp/Gympp.h"
#include "gympp/Log.h"
#include "gympp/common.h"
#include "gympp/gyms/Ignition.h"

#include <chrono>
#include <thread>
#include <typeindex>
#include <typeinfo>
#include <variant>

class Gym
{
public:
    //    struct CartPole
    //    {
    //        using ObservationBuffer = gympp::BufferDouble;
    //    };

    static gympp::Environment* make(const std::string& envName)
    {
        if (envName == "CartPole") {
            std::string sdfFile =
                "/home/dferigo/git/gym-ignition/models/CartPole/CartPoleWorld.sdf";

            using OSpace = gympp::spaces::Box;
            using ASpace = gympp::spaces::Discrete;

            using ActionDataType = size_t;
            using ObservationDataType = double;

            auto* ignGym = new gympp::gyms::IgnitionGazebo<ActionDataType, ObservationDataType>(
                "libCartPolePlugin.so",
                "gympp::plugins::CartPole",
                std::make_shared<ASpace>(2),
                std::make_shared<OSpace>(OSpace::Limit{-90, -20}, OSpace::Limit{90, 20}),
                sdfFile,
                /*updateRate=*/10,
                /*iterations=*/100);

            // TODO: how to handle the memory deallocation?
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

    if (!env->render(Environment::RenderMode::HUMAN)) {
        gymppError << "Failed to render the environment" << std::endl;
        return EXIT_FAILURE;
    }

    //    std::chrono::milliseconds period{100};
    //    std::cout << "Period " << period.count() << "ms" << std::endl;

    size_t epoch = 0;

    while (true) {
        std::cout << "#" << epoch++ << " " << std::flush;

        //        auto tick = std::chrono::high_resolution_clock::now();

        // TODO: use oldState to obtain the action

        // Here we use a random action to bypass it
        //        auto action = Environment::Action(std::valarray<size_t>{1});
        //        auto* action = actionSample.get<ActionDataType>();
        auto actionSample = env->action_space->sample();
        auto state = env->step(actionSample);

        if (!state.has_value()) {
            gymppError << "The environment didn't return the state" << std::endl;
            return EXIT_FAILURE;
        }

        if (state->done) {
            gymppDebug << "The environment reached the terminal state" << std::endl;
            break;
        }

        if (auto* o = state->observation.get<double>(); o) {
            for (const auto el : *o) {
                std::cout << el << " ";
            }
            std::cout << std::endl;
        }
        else {
            gymppError << "The environment didn't return the observation" << std::endl;
            return EXIT_FAILURE;
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
