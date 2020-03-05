/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef GYMPP_ROBOT_H
#define GYMPP_ROBOT_H

#include <array>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace gympp {
    class Robot;
    using RobotPtr = std::shared_ptr<gympp::Robot>;

    struct PID;
    struct Limit;
    struct Pose;
    struct Velocity6D;
    struct ContactData;
    struct Acceleration6D;
    enum class JointControlMode
    {
        Position,
        PositionInterpolated,
        Velocity,
        Torque,
    };
    enum class JointType
    {
        Invalid,
        Fixed,
        Revolute,
        Prismatic,
    };
} // namespace gympp

struct gympp::PID
{
    PID() = default;
    PID(double _p, double _i, double _d)
        : p(_p)
        , i(_i)
        , d(_d)
    {}

    double p;
    double i;
    double d;
};

struct gympp::Pose
{
    std::array<double, 3> position;
    std::array<double, 4> orientation;
};

struct gympp::Velocity6D
{
    std::array<double, 3> linear;
    std::array<double, 3> angular;
};

struct gympp::Acceleration6D
{
    std::array<double, 3> linear;
    std::array<double, 3> angular;
};

struct gympp::Limit
{
    double min = std::numeric_limits<double>::lowest();
    double max = std::numeric_limits<double>::max();
};

struct gympp::ContactData
{
    std::string bodyA;
    std::string bodyB;
    std::array<double, 3> depth;
    std::array<double, 6> wrench;
    std::array<double, 3> normal;
    std::array<double, 3> position;
};

class gympp::Robot
{
public:
    using VectorContainer = std::vector<double>;

    using LinkName = std::string;
    using RobotName = std::string;
    using JointName = std::string;
    using LinkNames = std::vector<LinkName>;
    using JointNames = std::vector<JointName>;

    using JointPositions = VectorContainer;
    using JointVelocities = VectorContainer;

    using PID = gympp::PID;
    using StepSize = std::chrono::duration<double>;

    Robot() = default;
    virtual ~Robot() = default;

    virtual bool valid() const = 0;

    // ===========
    // GET METHODS
    // ===========

    virtual size_t dofs() const = 0;
    virtual RobotName name() const = 0;
    virtual JointNames jointNames() const = 0;

    virtual JointType jointType(const JointName& jointName) const = 0;
    virtual double jointForce(const JointName& jointName) const = 0;
    virtual double jointPosition(const JointName& jointName) const = 0;
    virtual double jointVelocity(const JointName& jointName) const = 0;
    virtual JointControlMode jointControlMode(const JointName& jointName) const = 0;

    virtual JointPositions jointPositions() const = 0;
    virtual JointVelocities jointVelocities() const = 0;

    virtual JointPositions initialJointPositions() const = 0;

    virtual double jointEffortLimit(const JointName& jointName) const = 0;
    virtual Limit jointPositionLimits(const JointName& jointName) const = 0;

    virtual StepSize dt() const = 0;
    virtual PID jointPID(const JointName& jointName) const = 0;

    virtual LinkNames linksInContact() const = 0;
    virtual std::vector<ContactData> contactData(const LinkName& linkName) const = 0;

    virtual LinkNames linkNames() const = 0;
    virtual Pose linkPose(const LinkName& linkName) const = 0;
    virtual Velocity6D linkVelocity(const LinkName& linkName) const = 0;
    virtual Acceleration6D linkAcceleration(const LinkName& linkName) const = 0;
    virtual Velocity6D linkBodyFixedVelocity(const LinkName& linkName) const = 0;
    virtual Acceleration6D linkBodyFixedAcceleration(const LinkName& linkName) const = 0;

    // ===========
    // SET METHODS
    // ===========

    virtual bool setdt(const StepSize& stepSize) = 0;

    virtual bool setJointForce(const JointName& jointName, const double jointForce) = 0;
    virtual bool setJointEffortLimit(const JointName& jointName, const double effortLimit) = 0;

    virtual bool setJointPositionTarget(const JointName& jointName,
                                        const double jointPositionReference) = 0;
    virtual bool setJointVelocityTarget(const JointName& jointName,
                                        const double jointVelocityReference) = 0;

    virtual bool setJointPosition(const JointName& jointName, const double jointPosition) = 0;
    virtual bool setJointVelocity(const JointName& jointName, const double jointVelocity) = 0;
    virtual bool setJointControlMode(const JointName& jointName,
                                     const JointControlMode controlMode) = 0;

    virtual bool setJointPID(const JointName& jointName, const PID& pid) = 0;

    virtual bool resetJoint(const JointName& jointName,
                            const double jointPosition = 0,
                            const double jointVelocity = 0) = 0;

    virtual bool addExternalWrench(const LinkName& linkName,
                                   const std::array<double, 3>& force,
                                   const std::array<double, 3>& torque) = 0;
    virtual bool update(const std::chrono::duration<double>& simTime) = 0;

    // ==============
    // RobotBaseFrame
    // ==============

    virtual LinkName baseFrame() = 0;
    virtual bool setBaseFrame(const LinkName& baseLink) = 0;

    virtual Pose basePose() = 0;
    virtual Velocity6D baseVelocity() = 0;
    virtual bool setAsFloatingBase(bool isFloating) = 0;
    virtual bool resetBasePose(const std::array<double, 3>& position,
                               const std::array<double, 4>& orientation) = 0;
    virtual bool resetBaseVelocity(const std::array<double, 3>& linear,
                                   const std::array<double, 3>& angular) = 0;
    virtual std::array<double, 6> baseWrench() = 0;
};

#endif // GYMPP_ROBOT_H
