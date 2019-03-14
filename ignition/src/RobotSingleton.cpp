#include "gympp/gazebo/RobotSingleton.h"
#include "gympp/Log.h"

#include <unordered_map>

using namespace gympp::gazebo;
using RobotName = std::string;

class RobotSingleton::Impl
{
public:
    std::unordered_map<RobotName, RobotPtr> robots;
};

RobotSingleton::RobotSingleton()
    : pImpl(std::make_unique<Impl>())
{}

RobotSingleton& RobotSingleton::get()
{
    static RobotSingleton instance;
    return instance;
}

gympp::RobotPtr RobotSingleton::getRobot(const std::string& robotName) const
{
    if (pImpl->robots.find(robotName) == pImpl->robots.end()) {
        gymppError << "Failed to find robot '" << robotName << "'" << std::endl;
        return nullptr;
    }

    return pImpl->robots.at(robotName);
}

bool RobotSingleton::storeRobot(RobotPtr robotInterface)
{
    if (!(robotInterface && robotInterface->valid())) {
        gymppError << "Trying to store an Robot pointer not valid" << std::endl;
        return false;
    }

    if (pImpl->robots.find(robotInterface->name()) != pImpl->robots.end()) {
        gymppError << "The '" << robotInterface->name()
                   << "' robot seems duplicated. It has been already added." << std::endl;
        return false;
    }

    gymppDebug << "Registering robot '" << robotInterface->name() << "'" << std::endl;
    pImpl->robots[robotInterface->name()] = robotInterface;
    return true;
}
