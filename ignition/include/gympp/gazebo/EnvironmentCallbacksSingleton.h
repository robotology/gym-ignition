/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef GYMPP_ROBOT_ENVIRONMENTCALLBACKSSINGLETON_H
#define GYMPP_ROBOT_ENVIRONMENTCALLBACKSSINGLETON_H

#include <ignition/common/SingletonT.hh>

#include <functional>
#include <memory>
#include <string>

#include "gympp/gazebo/EnvironmentCallbacks.h"
namespace gympp {
    namespace gazebo {
        class EnvironmentCallbacksSingleton;
    } // namespace gazebo
} // namespace gympp

class gympp::gazebo::EnvironmentCallbacksSingleton
    : public ignition::common::SingletonT<EnvironmentCallbacksSingleton>
{
private:
    class Impl;
    std::unique_ptr<Impl, std::function<void(Impl*)>> pImpl;

public:
    EnvironmentCallbacksSingleton();

    gympp::gazebo::EnvironmentCallbacks* get(const std::string& label);
    bool storeEnvironmentCallback(const std::string& label,
                                  gympp::gazebo::EnvironmentCallbacks* cb);
};

#endif // GYMPP_ROBOT_ENVIRONMENTCALLBACKSSINGLETON_H
