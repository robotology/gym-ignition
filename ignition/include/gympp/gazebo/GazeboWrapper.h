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
        class GazeboWrapper;
    } // namespace gazebo
} // namespace gympp

class gympp::gazebo::GazeboWrapper
{
private:
    class Impl;
    std::unique_ptr<Impl, std::function<void(Impl*)>> pImpl;

public:
    using SdfModelName = std::string;

    GazeboWrapper() = delete;
    GazeboWrapper(double updateRate);
    virtual ~GazeboWrapper();

    bool initialize();
    bool run();
    bool gui();
    bool close();

    double getUpdateRate() const;
    uint64_t getNumberOfIterations() const;

    static void setVerbosity(int level = DEFAULT_VERBOSITY);
    std::vector<SdfModelName> getModelNames() const;
    bool setupGazeboModel(const std::string& modelFile,
                          std::array<double, 6> pose = {0, 0, 0, 0, 0, 0});
    bool setupGazeboWorld(const std::string& worldFile);
    bool setupIgnitionPlugin(const std::string& libName,
                             const std::string& className,
                             double agentUpdateRate = 0);
};

#endif // GYMPP_GAZEBO_GAZEBOWRAPPER
