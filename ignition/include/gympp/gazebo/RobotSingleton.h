/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef GYMPP_ROBOT_ROBOTSINGLETON_H
#define GYMPP_ROBOT_ROBOTSINGLETON_H

#include "gympp/Robot.h"

#include <memory>
#include <string>

namespace gympp {
    class Robot;
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
    ~RobotSingleton() = default;
    RobotSingleton(RobotSingleton&) = delete;
    void operator=(const RobotSingleton&) = delete;

    static RobotSingleton& get();

    RobotPtr getRobot(const std::string& robotName) const;
    bool storeRobot(RobotPtr robotInterface);
};

#endif // GYMPP_ROBOT_ROBOTSINGLETON_H
