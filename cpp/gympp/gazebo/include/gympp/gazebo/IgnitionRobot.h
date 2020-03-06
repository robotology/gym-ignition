/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef GYMPP_GAZEBO_IGNITIONROBOT_H
#define GYMPP_GAZEBO_IGNITIONROBOT_H

#include "gympp/base/Robot.h"

#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>
#include <sdf/Element.hh>

#include <memory>

namespace gympp {
    namespace gazebo {
        class IgnitionRobot;
    } // namespace gazebo
} // namespace gympp

class gympp::gazebo::IgnitionRobot : public gympp::base::Robot
{
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

public:
    IgnitionRobot();
    ~IgnitionRobot() override;

    bool configureECM(const ignition::gazebo::Entity& entity,
                      ignition::gazebo::EntityComponentManager* ecm,
                      ignition::gazebo::EventManager* eventManager);
    bool valid() const override;

    // ===========
    // GET METHODS
    // ===========

    size_t dofs() const override;

    RobotName name() const override;
    JointNames jointNames() const override;

    base::JointType jointType(const JointName& jointName) const override;
    double jointForce(const JointName& jointName) const override;
    double jointPosition(const JointName& jointName) const override;
    double jointVelocity(const JointName& jointName) const override;
    base::JointControlMode jointControlMode(const JointName& jointName) const override;

    JointPositions jointPositions() const override;
    JointVelocities jointVelocities() const override;

    JointPositions initialJointPositions() const override;

    double jointEffortLimit(const JointName& jointName) const override;
    base::Limit jointPositionLimits(const JointName& jointName) const override;

    StepSize dt() const override;
    PID jointPID(const JointName& jointName) const override;

    LinkNames linksInContact() const override;
    std::vector<base::ContactData> contactData(const LinkName& linkName) const override;

    LinkNames linkNames() const override;
    base::Pose linkPose(const LinkName& linkName) const override;
    base::Velocity6D linkVelocity(const LinkName& linkName) const override;
    base::Acceleration6D linkAcceleration(const LinkName& linkName) const override;
    base::Velocity6D linkBodyFixedVelocity(const LinkName& linkName) const override;
    base::Acceleration6D linkBodyFixedAcceleration(const LinkName& linkName) const override;

    // ===========
    // SET METHODS
    // ===========

    bool setdt(const StepSize& stepSize) override;

    bool setJointForce(const JointName& jointName, const double jointForce) override;
    bool setJointEffortLimit(const JointName& jointName, const double effortLimit) override;

    bool setJointPositionTarget(const JointName& jointName,
                                const double jointPositionReference) override;
    bool setJointVelocityTarget(const JointName& jointName,
                                const double jointVelocityReference) override;

    bool setJointPosition(const JointName& jointName, const double jointPosition) override;
    bool setJointVelocity(const JointName& jointName, const double jointVelocity) override;
    bool setJointControlMode(const JointName& jointName,
                             const base::JointControlMode controlMode) override;

    bool setJointPID(const JointName& jointName, const PID& pid) override;

    bool resetJoint(const JointName& jointName,
                    const double jointPosition = 0,
                    const double jointVelocity = 0) override;

    bool addExternalWrench(const LinkName& linkName,
                           const std::array<double, 3>& force,
                           const std::array<double, 3>& torque) override;
    bool update(const std::chrono::duration<double>& simTime) override;

    // ==============
    // RobotBaseFrame
    // ==============

    LinkName baseFrame() override;
    bool setBaseFrame(const LinkName& baseLink) override;

    base::Pose basePose() override;
    base::Velocity6D baseVelocity() override;
    bool setAsFloatingBase(bool isFloating) override;
    bool resetBasePose(const std::array<double, 3>& position,
                       const std::array<double, 4>& orientation) override;
    bool resetBaseVelocity(const std::array<double, 3>& linear,
                           const std::array<double, 3>& angular) override;
    std::array<double, 6> baseWrench() override;
};

#endif // GYMPP_GAZEBO_IGNITIONROBOT_H
