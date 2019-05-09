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

#include <string>
#include <unordered_map>

namespace gympp {
    namespace gazebo {
        class EnvironmentCallbacks;
        class EnvironmentCallbacksSingleton;
    } // namespace gazebo
} // namespace gympp

class gympp::gazebo::EnvironmentCallbacksSingleton
    : public ignition::common::SingletonT<EnvironmentCallbacksSingleton>
{
private:
    static std::unordered_map<std::string, EnvironmentCallbacks*> m_callbacks;

protected:
public:
    EnvironmentCallbacksSingleton() = default;
    ~EnvironmentCallbacksSingleton() override = default;

    gympp::gazebo::EnvironmentCallbacks* get(const std::string& label);
    bool storeEnvironmentCallback(const std::string& label,
                                  gympp::gazebo::EnvironmentCallbacks* cb);
};

#endif // GYMPP_ROBOT_ENVIRONMENTCALLBACKSSINGLETON_H
