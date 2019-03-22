#ifndef GYMPP_ROBOT_IGNITIONROBOT_H
#define GYMPP_ROBOT_IGNITIONROBOT_H

#include "gympp/Robot.h"

#include <ignition/gazebo/System.hh>

#include <memory>

namespace gympp {
    namespace gazebo {
        class IgnitionRobot;
    } // namespace gazebo
} // namespace gympp

class gympp::gazebo::IgnitionRobot final
    : public gympp::Robot
    , public ignition::gazebo::System
    , public ignition::gazebo::ISystemConfigure
{
private:
    class Impl;
    std::unique_ptr<Impl, std::function<void(Impl*)>> pImpl;

public:
    IgnitionRobot();
    ~IgnitionRobot() override;

    void Configure(const ignition::gazebo::Entity& entity,
                   const std::shared_ptr<const sdf::Element>& sdf,
                   ignition::gazebo::EntityComponentManager& ecm,
                   ignition::gazebo::EventManager& eventMgr) override;

    bool valid() const override;

    // ===========
    // GET METHODS
    // ===========

    RobotName name() const override;
    JointNames jointNames() const override;

    double jointPosition(const JointName& jointName) const override;
    double jointVelocity(const JointName& jointName) const override;

    JointPositions jointPositions() const override;
    JointVelocities jointVelocities() const override;

    StepSize dt() const override;
    PID jointPID(const JointName& jointName) const override;

    // ===========
    // SET METHODS
    // ===========

    bool setdt(const StepSize& stepSize) override;

    bool setJointForce(const JointName& jointName, const double jointForce) override;
    bool setJointPosition(const JointName& jointName, const double jointPositionReference) override;
    bool setJointVelocity(const JointName& jointName, const double jointVelocityReference) override;

    bool setJointPID(const JointName& jointName, const PID& pid) override;

    bool resetJoint(const JointName& jointName, const double jointPosition) override;
};

#endif // GYMPP_ROBOT_IGNITIONROBOT_H
