#ifndef GYMPP_ROBOT_ROBOTSINGLETON_H
#define GYMPP_ROBOT_ROBOTSINGLETON_H

#include "gympp/Robot.h"

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Model.hh>
#include <sdf/Model.hh>

#include <memory>

namespace gympp {
    class Robot;
    namespace robot {
        class RobotSingleton;
    } // namespace robot
} // namespace gympp

class gympp::robot::RobotSingleton
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
