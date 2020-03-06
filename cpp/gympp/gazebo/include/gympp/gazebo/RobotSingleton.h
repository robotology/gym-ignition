/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef GYMPP_GAZEBO_ROBOTSINGLETON_H
#define GYMPP_GAZEBO_ROBOTSINGLETON_H

#include "gympp/base/Robot.h"

#include <memory>
#include <string>

namespace gympp {
    namespace gazebo {
        class RobotSingleton;
    } // namespace gazebo
} // namespace gympp

class gympp::gazebo::RobotSingleton
{
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

public:
    RobotSingleton();
    ~RobotSingleton();
    RobotSingleton(RobotSingleton&) = delete;
    void operator=(const RobotSingleton&) = delete;

    static RobotSingleton& get();

    bool exists(const std::string& robotName) const;

    std::weak_ptr<gympp::base::Robot> getRobot(const std::string& robotName) const;

    bool storeRobot(base::RobotPtr robot);
    bool deleteRobot(const std::string& robotName);
};

#endif // GYMPP_GAZEBO_ROBOTSINGLETON_H
