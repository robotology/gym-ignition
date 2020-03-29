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

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#ifdef NDEBUG
#define DEFAULT_VERBOSITY 2
#else
#define DEFAULT_VERBOSITY 4
#endif

namespace scenario {
    namespace gazebo {
        struct PluginData;
        struct PhysicsData;
        struct ModelInitData;
        class GazeboSimulator;
    } // namespace gazebo
} // namespace scenario

namespace sdf {
    inline namespace v9 {
        class Root;
    } // namespace v9
} // namespace sdf

struct scenario::gazebo::ModelInitData
{
    std::string sdfString;
    bool fixedPose = false;
    std::string baseLink = "";
    std::string modelName = "";
    std::array<double, 3> position = {0, 0, 0};
    std::array<double, 4> orientation = {1, 0, 0, 0};
};

struct scenario::gazebo::PluginData
{
    std::string libName;
    std::string className;
};

class scenario::gazebo::GazeboSimulator
{
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

protected:
    bool findAndLoadSdf(const std::string& sdfFileName, sdf::Root& root);

public:
    GazeboSimulator(const size_t numOfIterations = 1,
                    const double desiredRTF = std::numeric_limits<double>::max(),
                    const double physicsUpdateRate = 1000);
    virtual ~GazeboSimulator();

    bool initialize();
    bool run();
    bool gui();
    bool close();

    bool initialized();

    double getSimulatedTime() const;
    PhysicsData getPhysicsData() const;
    static void setVerbosity(int level = DEFAULT_VERBOSITY);

    bool insertModel(const scenario::gazebo::ModelInitData& modelData,
                     const scenario::gazebo::PluginData& pluginData = {});
    bool removeModel(const std::string& modelName);
    static std::string getModelNameFromSDF(const std::string& sdfString);

    std::string getWorldName() const;
    bool setupGazeboWorld(const std::string& worldFile);
};

struct scenario::gazebo::PhysicsData
{
    double rtf;
    double maxStepSize;
    const double realTimeUpdateRate = -1;

    PhysicsData(double _rtf = 1, double _maxStepSize = 0.001)
        : rtf(_rtf)
        , maxStepSize(_maxStepSize)
    {}

    PhysicsData(const PhysicsData& other)
        : rtf(other.rtf)
        , maxStepSize(other.maxStepSize)
        , realTimeUpdateRate(other.realTimeUpdateRate)
    {}

    PhysicsData& operator=(const PhysicsData& other)
    {
        rtf = other.rtf;
        maxStepSize = other.maxStepSize;
        return *this;
    }

    bool operator==(const PhysicsData& other)
    {
        return other.rtf == rtf && other.maxStepSize == maxStepSize
               && other.realTimeUpdateRate == realTimeUpdateRate;
    }
};

#endif // SCENARIO_GAZEBO_GAZEBOSIMULATOR_H
