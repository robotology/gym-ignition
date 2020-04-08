/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef GYMPP_GAZEBO_GYMFACTORY
#define GYMPP_GAZEBO_GYMFACTORY

#include "gympp/base/Environment.h"
#include <ignition/common/SingletonT.hh>

#include <memory>
#include <string>

namespace gympp {
    namespace gazebo {
        class GymFactory;
        class PluginMetadata;
    } // namespace gazebo
} // namespace gympp

class gympp::gazebo::GymFactory
    : public ignition::common::SingletonT<gympp::gazebo::GymFactory>
{
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

public:
    GymFactory();
    ~GymFactory();

    gympp::base::EnvironmentPtr make(const std::string& envName);
    bool registerPlugin(const PluginMetadata& md);
};

#endif // GYMPP_GAZEBO_GYMFACTORY
