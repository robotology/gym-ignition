/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "gympp/Environment.h"
#include "gympp/GymFactory.h"
#include "gympp/Log.h"
#include "gympp/PluginDatabase.h"
#include "gympp/Random.h"
#include "gympp/Space.h"

#include "clara.hpp"

#include <ignition/common/SignalHandler.hh>

#include <atomic>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace gympp;
using namespace clara;

class Worker
{
private:
    size_t m_id;
    size_t m_maxEpisodes;
    std::string m_environmentName;
    EnvironmentPtr m_env;
    static std::mutex m_mutex;

public:
    // Global variable shared by worker threads
    static std::atomic<size_t> BestScore;
    static std::atomic<size_t> InstanceCounter;
    static std::atomic<size_t> NumberOfEpisodes;
    static std::atomic<double> MovingAverageReward;

    using EnvironmentName = std::string;

    Worker(const EnvironmentName& name, size_t maxEpisodes)
        : m_id(Worker::InstanceCounter++)
        , m_maxEpisodes(maxEpisodes)
        , m_environmentName(name)
    {
        gymppDebug << "Constructing worker #" << m_id;
    }

    bool initialize()
    {
        m_env = GymFactory::Instance()->make(m_environmentName);

        if (!m_env) {
            gymppError << "Failed to create environment '" << m_environmentName << "'" << std::endl;
            return false;
        }

        return true;
    }

    void run()
    {
        // This check the number of episodes of all workers
        while (Worker::NumberOfEpisodes <= (m_maxEpisodes - std::thread::hardware_concurrency())) {
            // Reset the environment
            assert(m_env);
            auto reward = Environment::Reward(0);
            auto observation = m_env->reset();
            assert(observation);

            // Create the initial state object
            Environment::State oldState;
            oldState.done = false;
            oldState.observation = observation.value();

            size_t episodeStep = 0;

            while (!oldState.done) {
                // Process oldState to obtain the action.
                // Here we use a random action to bypass it.
                auto actionSample = m_env->action_space->sample();

                // Simulate the system with the given action
                auto state = m_env->step(actionSample);
                assert(state && state->observation.getBuffer<double>());

                // Cumulate the reward
                reward += state->reward;

                // Save the old state
                oldState = std::move(state.value());

                // Increase the episode steps counter
                episodeStep++;

                // Handle termination
                if (state->done) {
                    std::unique_lock lock(Worker::m_mutex);

                    // Increase the episodes counter
                    Worker::NumberOfEpisodes++;
                    record(NumberOfEpisodes, reward, m_id, episodeStep);

                    // Handle this episode if it is the best achieved until now
                    if (reward > Worker::BestScore) {
                        // CartPole rewards are integers
                        Worker::BestScore = static_cast<size_t>(reward);
                        gymppMessage << "New best score: " << reward << std::endl;
                    }
                }
            }
        }
    }

    static void record(const size_t episode,
                       const Reward reward,
                       const size_t worker_id,
                       const size_t numOfSteps)
    {
        if (Worker::MovingAverageReward == 0.0) {
            Worker::MovingAverageReward = reward;
        }
        else {
            Worker::MovingAverageReward = Worker::MovingAverageReward * 0.99 + reward * 0.01;
        }

        std::cout << "Episode: " << episode
                  << " | Moving Average Reward: " << Worker::MovingAverageReward
                  << " | Episode reward: " << reward << " | Steps: " << numOfSteps
                  << " | Worker: " << worker_id << std::endl;
    }
};

// Define static attributes
std::mutex Worker::m_mutex;
std::atomic<size_t> Worker::BestScore = 0;
std::atomic<size_t> Worker::InstanceCounter = 0;
std::atomic<size_t> Worker::NumberOfEpisodes = 0;
std::atomic<double> Worker::MovingAverageReward = 0;

class MasterAgent
{
private:
    size_t m_maxEpisodes;
    std::vector<std::unique_ptr<Worker>> m_workers;
    std::vector<std::thread> m_pool;
    const gympp::EnvironmentName EnvName = "CartPole";

public:
    MasterAgent(const size_t maxEpisodes)
        : m_maxEpisodes(maxEpisodes)
    {}

    bool train()
    {
        unsigned threadAffinity = std::thread::hardware_concurrency();
        gymppMessage << "Machine supports " << threadAffinity << " concurrent threads" << std::endl;

        for (size_t i = 0; i < threadAffinity; ++i) {
            // Create the worker
            m_workers.emplace_back(std::make_unique<Worker>(EnvName, m_maxEpisodes));

            auto& worker = m_workers.back();

            if (!worker->initialize()) {
                return false;
            }

            // Add its run method in the thread pool
            m_pool.emplace_back(&Worker::run, &*m_workers.back());
        }

        for (auto& thread : m_pool) {
            thread.join();
        }

        return true;
    }

    void reset()
    {
        for (auto& worker : m_workers) {
            worker.reset();
        }
    }
};

struct Config
{
    bool help = false;
    bool train = false;
    size_t maxEpisodes = 100;
    std::optional<size_t> seed;
};

int main(int argc, char* argv[])
{
    // ==================
    // PARSE COMMAND LINE
    // ==================

    Config config;

    // Create the command line parser
    auto cli =
        Help(config.help) //
        | Opt(config.maxEpisodes, "n")["-m"]["--max-episodes"]("Maximum number of episodes to run")
        | Opt(config.train)["-t"]["--train"]("Train the model")
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

    // ====================
    // INITIALIZE THE AGENT
    // ====================

    if (config.seed) {
        Random::setSeed(config.seed.value());
    }

    // Create the master agent
    MasterAgent agent(config.maxEpisodes);

    // Terminate the agent gracefully
    ignition::common::SignalHandler sigHandler;
    assert(sigHandler.Initialized());
    sigHandler.AddCallback([&](const int /*_sig*/) {
        gymppDebug << "Shutting down gracefully" << std::endl;
        agent.reset();
        exit(EXIT_FAILURE);
    });

    // =====
    // TRAIN
    // =====

    bool ok = agent.train();

    if (!ok) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
