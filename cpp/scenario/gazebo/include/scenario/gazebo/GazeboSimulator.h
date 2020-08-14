/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This project is dual licensed under LGPL v2.1+ or Apache License.
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SCENARIO_GAZEBO_GAZEBOSIMULATOR_H
#define SCENARIO_GAZEBO_GAZEBOSIMULATOR_H

#include "scenario/core/World.h"

#include <memory>
#include <string>
#include <vector>

namespace scenario::gazebo {
    class World;
    class GazeboSimulator;
} // namespace scenario::gazebo

class scenario::gazebo::GazeboSimulator
{
public:
    /**
     * Class wrapping the Ignition Gazebo simulator.
     *
     * The simulator can execute either unpaused or paused runs. Paused runs
     * update the internal state of the simulator without stepping the physics.
     * This is useful to process e.g. model insertions or removals.
     * Every simulator run can execute one or more physics steps, depending on
     * how it is configured when constructed.
     *
     * @param stepSize The size of the physics step.
     * @param rtf The desired real-time factor.
     * @param stepsPerRun Number of steps to execute at each simulator run.
     */
    GazeboSimulator(const double stepSize = 0.001,
                    const double rtf = 1.0,
                    const size_t stepsPerRun = 1);
    virtual ~GazeboSimulator();

    /**
     * Get the size of a simulator step.
     *
     * @return The simulator step size in seconds.
     */
    double stepSize() const;

    /**
     * Get the desired real-time factor of the simulator.
     *
     * @return The desired real-time factor.
     */
    double realTimeFactor() const;

    /**
     * Get the number or steps to execute every simulator run.
     *
     * @return The configured number of steps per run.
     */
    size_t stepsPerRun() const;

    /**
     * Initialize the simulator.
     *
     * @return True for success, false otherwise.
     */
    bool initialize();

    /**
     * Check if the simulator has been initialized.
     *
     * @return True if the simulator was initialized, false otherwise.
     */
    bool initialized() const;

    /**
     * Run the simulator.
     *
     * @param paused True to perform paused steps that do not affect the
     * physics, false for normal steps. The number of steps configured during
     * construction are executed.
     * @return True for success, false otherwise.
     */
    bool run(const bool paused = false);

    /**
     * Open the Ignition Gazebo GUI.
     *
     * @param verbosity Configure the verbosity of the GUI (0-4)
     * @return True for success, false otherwise.
     */
    bool gui(const int verbosity = -1);

    /**
     * Close the simulator and the GUI.
     *
     * @return True for success, false otherwise.
     */
    bool close();

    /**
     * Pause the simulator.
     *
     * @note This method is useful in non-deterministic mode, which is not
     * currently supported.
     *
     * @return True for success, false otherwise.
     */
    bool pause();

    /**
     * Check if the simulator is running.
     *
     * @note This method is useful in non-deterministic mode, which is not
     * currently supported.
     *
     * @return True for success, false otherwise.
     */
    bool running() const;

    /**
     * Load a SDF world file.
     *
     * @note If the world file is not passed, the default empty world is
     * inserted. The default empty world does not have the ground plane nor
     * any physics. Both can be added by operating on the ``World`` object.
     *
     * @param worldFile The path to the SDF world file.
     * @param worldName Optionally override the name of the world defined in the
     * SDF world file.
     * @return True for success, false otherwise.
     */
    bool insertWorldFromSDF(const std::string& worldFile = "",
                            const std::string& worldName = "");

    /**
     * Load a SDF world file containing multiple worlds.
     *
     * @param worldFile The path to the SDF world file.
     * @param worldNames Optionally override the names of the worlds defined in
     * the SDF world file.
     * @return True for success, false otherwise.
     *
     * @warning This is an experimental feature. Multiworld simulations are only
     * partially supported by Ignition Gazebo.
     */
    bool insertWorldsFromSDF(const std::string& worldFile,
                             const std::vector<std::string>& worldNames = {});

    /**
     * Get the list if the world names part of the simulation.
     *
     * @return The world names.
     */
    std::vector<std::string> worldNames() const;

    /**
     * Get a simulated world.
     *
     * @param worldName The name of the desired world.
     * @return The world object.
     */
    std::shared_ptr<scenario::gazebo::World>
    getWorld(const std::string& worldName = "") const;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

#endif // SCENARIO_GAZEBO_GAZEBOSIMULATOR_H
