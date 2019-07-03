/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef GYMPP_GAZEBO_GAZEBOWRAPPER
#define GYMPP_GAZEBO_GAZEBOWRAPPER

#include <array>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#ifdef NDEBUG
#define DEFAULT_VERBOSITY 2
#else
#define DEFAULT_VERBOSITY 4
#endif

namespace gympp {
    namespace gazebo {
        struct PhysicsData;
        class GazeboWrapper;
    } // namespace gazebo
} // namespace gympp

class gympp::gazebo::GazeboWrapper
{
private:
    class Impl;
    std::unique_ptr<Impl, std::function<void(Impl*)>> pImpl;

public:
    // TODO
    using SdfModelName = std::string;

    GazeboWrapper(const size_t numOfIterations = 1,
                  const double desiredRTF = std::numeric_limits<double>::max(),
                  const double physicsUpdateRate = 1000);
    virtual ~GazeboWrapper();

    bool initialize();
    bool run();
    bool gui();
    bool close();

    PhysicsData getPhysicsData() const;

    static void setVerbosity(int level = DEFAULT_VERBOSITY);
    std::vector<SdfModelName> getModelNames() const;
    bool setupGazeboModel(const std::string& modelFile,
                          const std::array<double, 6>& pose = {0, 0, 0, 0, 0, 0});
    bool setupGazeboWorld(const std::string& worldFile);
    bool setupIgnitionPlugin(const std::string& libName, const std::string& className);
};

struct gympp::gazebo::PhysicsData
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

#endif // GYMPP_GAZEBO_GAZEBOWRAPPER
