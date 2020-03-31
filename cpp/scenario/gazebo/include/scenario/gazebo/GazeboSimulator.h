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

#include <memory>
#include <string>
#include <vector>

namespace scenario {
    namespace gazebo {
        class World;
        class GazeboSimulator;
    } // namespace gazebo
} // namespace scenario

class scenario::gazebo::GazeboSimulator
{
public:
    GazeboSimulator(const double stepSize = 0.001,
                    const double rtf = 1.0,
                    const size_t stepsPerRun = 1);

    virtual ~GazeboSimulator();

    double stepSize() const;
    double realTimeFactor() const;
    size_t stepsPerRun() const;

    bool initialize();
    bool initialized() const;

    bool run(const bool paused = false);
    bool gui(const int verbosity = -1);
    bool close();

    bool pause();
    bool running() const;

    bool loadSdfWorld(const std::string& worldFile,
                      const std::string& worldName = "");

    std::vector<std::string> worldNames() const;
    std::shared_ptr<scenario::gazebo::World>
    getWorld(const std::string& worldName = "") const;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

#endif // SCENARIO_GAZEBO_GAZEBOSIMULATOR_H
