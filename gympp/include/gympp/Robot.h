#ifndef GYMPP_ROBOT_H
#define GYMPP_ROBOT_H

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace gympp {
    class Robot;
    //    using RobotPtr = std::shared_ptr<gympp::Robot>;
    using RobotPtr = gympp::Robot*;
    struct PID;
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

class gympp::Robot
{
public:
    using VectorContainer = std::vector<double>;

    using RobotName = std::string;
    using JointName = std::string;
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

    virtual RobotName name() const = 0;
    virtual JointNames jointNames() const = 0;

    virtual double jointPosition(const JointName& jointName) const = 0;
    virtual double jointVelocity(const JointName& jointName) const = 0;

    virtual JointPositions jointPositions() const = 0;
    virtual JointVelocities jointVelocities() const = 0;

    virtual StepSize dt() const = 0;
    virtual PID jointPID(const JointName& jointName) const = 0;

    // ===========
    // SET METHODS
    // ===========

    virtual bool setdt(const StepSize& stepSize) = 0;

    virtual bool setJointForce(const JointName& jointName, const double jointForce) = 0;
    virtual bool setJointPosition(const JointName& jointName,
                                  const double jointPositionReference) = 0;
    virtual bool setJointVelocity(const JointName& jointName,
                                  const double jointVelocityReference) = 0;

    virtual bool setJointPID(const JointName& jointName, const PID& pid) = 0;

    virtual bool resetJoint(const JointName& jointName, const double jointPosition) = 0;
};

#endif // GYMPP_ROBOT_H
