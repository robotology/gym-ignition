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
        struct PluginData;
        struct PhysicsData;
        struct ModelInitData;
        class GazeboWrapper;
    } // namespace gazebo
} // namespace gympp

namespace sdf {
    inline namespace v8 {
        class Root;
    } // namespace v8
} // namespace sdf

struct gympp::gazebo::ModelInitData
{
    std::string sdfString;
    std::string modelName = "";
    std::array<double, 3> position = {0, 0, 0};
    std::array<double, 4> orientation = {1, 0, 0, 0};

    inline void setSdfString(const std::string& s) { sdfString = s; }
    inline void setModelName(const std::string& m) { modelName = m; }
    inline void setPosition(const std::array<double, 3> p) { position = p; }
    inline void setOrientation(const std::array<double, 4> o) { orientation = o; }
};

struct gympp::gazebo::PluginData
{
    std::string libName;
    std::string className;

    inline void setLibName(const std::string& l) { libName = l; }
    inline void setClassName(const std::string& c) { className = c; }
};

class gympp::gazebo::GazeboWrapper
{
private:
    class Impl;
    std::unique_ptr<Impl, std::function<void(Impl*)>> pImpl;

protected:
    bool findAndLoadSdf(const std::string& sdfFileName, sdf::Root& root);

public:
    GazeboWrapper(const size_t numOfIterations = 1,
                  const double desiredRTF = std::numeric_limits<double>::max(),
                  const double physicsUpdateRate = 1000);
    virtual ~GazeboWrapper();

    bool initialize();
    bool run();
    bool gui();
    bool close();

    bool initialized();

    PhysicsData getPhysicsData() const;
    static void setVerbosity(int level = DEFAULT_VERBOSITY);

    bool insertModel(const gympp::gazebo::ModelInitData& modelData,
                     const gympp::gazebo::PluginData& pluginData = {});
    bool removeModel(const std::string& modelName);
    static std::string getModelNameFromSDF(const std::string& sdfString);

    std::string getWorldName() const;
    bool setupGazeboWorld(const std::string& worldFile);
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
