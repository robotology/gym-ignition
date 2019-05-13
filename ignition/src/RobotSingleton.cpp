/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "gympp/gazebo/RobotSingleton.h"
#include "gympp/Log.h"

#include <ostream>
#include <unordered_map>

using namespace gympp::gazebo;
using RobotName = std::string;

class RobotSingleton::Impl
{
public:
    std::unordered_map<RobotName, RobotPtr> robots;
};

RobotSingleton::RobotSingleton()
    : pImpl{new Impl(), [](Impl* impl) { delete impl; }}
{}

RobotSingleton& RobotSingleton::get()
{
    static RobotSingleton instance;
    return instance;
}

gympp::RobotPtr RobotSingleton::getRobot(const std::string& robotName) const
{
    if (robotName.empty()) {
        gymppError << "The robot name to register is empty" << std::endl;
        return nullptr;
    }

    if (pImpl->robots.find(robotName) == pImpl->robots.end()) {
        gymppError << "Failed to find robot '" << robotName << "'" << std::endl;
        return nullptr;
    }

    assert(pImpl->robots.at(robotName));
    return pImpl->robots.at(robotName);
}

bool RobotSingleton::storeRobot(RobotPtr robot)
{
    if (!(robot && robot->valid())) {
        gymppError << "Trying to store an Robot pointer not valid" << std::endl;
        return false;
    }

    if (pImpl->robots.find(robot->name()) != pImpl->robots.end()) {
        gymppError << "The '" << robot->name()
                   << "' robot seems duplicated. It has been already added." << std::endl;
        return false;
    }

    gymppDebug << "Registering robot '" << robot->name() << "'" << std::endl;
    pImpl->robots[robot->name()] = robot;
    return true;
}

bool RobotSingleton::deleteRobot(const std::string& robotName)
{
    if (robotName.empty()) {
        gymppError << "The robot name to unregister is empty" << std::endl;
        return false;
    }

    if (pImpl->robots.find(robotName) == pImpl->robots.end()) {
        gymppError << "The robot '" << robotName << "' has never been stored" << std::endl;
        return false;
    }

    pImpl->robots.erase(robotName);
    return true;
}
