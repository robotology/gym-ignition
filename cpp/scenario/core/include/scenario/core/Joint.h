/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef SCENARIO_CORE_JOINT_H
#define SCENARIO_CORE_JOINT_H

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace scenario::core {
    struct PID;
    struct Limit;
    struct JointLimit;

    /**
     * Supported joint types.
     */
    enum class JointType
    {
        Invalid,
        Fixed,
        Revolute,
        Prismatic,
        Ball,
    };

    /**
     * Supported joint control modes.
     */
    enum class JointControlMode
    {
        /// Marks the joint to have an invalid control mode.
        Invalid,

        /// Marks the joint to be IDLE. An IDLE joint is equivalent to a joint
        /// controlled in Force with zero references. The joint shows only
        /// passive behaviour.
        Idle,

        /// Marks the joint to be controlled in force. A Force joint receives
        /// generalized force references that are actuated by a force actuator.
        /// Depending on the active backend, the presence of friction and other
        /// loss components could be compensated.
        Force,

        /// Marks the joint to be controlled in velocity. A Velocity joint
        /// receives velocity references that are actuated using a PID
        /// controller.
        Velocity,

        /// Marks the joint to follow precisely a velocity trajectory. A
        /// VelocityFollowerDart joint receives velocity references that
        /// are processed by the physics engine, which computes instantaneously
        /// the right force to apply to follow the desired trajectory.
        /// It works only with the DART physics engine.
        VelocityFollowerDart,

        /// Marks the joint to be controlled in position. A Position joint
        /// receives position references that are actuated using a PID
        /// controller.
        Position,

        /// Marks the joint to be controlled in position with trajectory
        /// smoothing. A PositionInterpolated joint receives position references
        /// that are filtered to get a smooth trajectory. The resulting
        /// trajectory is then actuated using a position PID controller.
        PositionInterpolated,
    };
    class Joint;
} // namespace scenario::core

class scenario::core::Joint
{
public:
    Joint() = default;
    virtual ~Joint() = default;

    /**
     * Check if the joint is valid.
     *
     * @return True if the joint is valid, false otherwise.
     */
    virtual bool valid() const = 0;

    /**
     * Get the number of degrees of freedom of the joint.
     *
     * @return The number of DOFs of the joint.
     */
    virtual size_t dofs() const = 0;

    /**
     * Get the name of the joint.
     *
     * @param scoped If true, the scoped name of the joint is returned.
     * @return The name of the joint.
     */
    virtual std::string name(const bool scoped = false) const = 0;

    /**
     * Get the type of the joint.
     *
     * @return The type of the joint.
     */
    virtual JointType type() const = 0;

    /**
     * Get the active joint control mode.
     *
     * @return The active joint control mode.
     */
    virtual JointControlMode controlMode() const = 0;

    /**
     * Set the joint control mode.
     *
     * @param mode The desired control mode.
     * @return True for success, false otherwise.
     */
    virtual bool setControlMode(const JointControlMode mode) = 0;

    /**
     * Get the period of the controller, if any.
     *
     * The controller period is a model quantity. If no controller
     * is active, infinity is returned.
     *
     * @return The the controller period.
     */
    virtual double controllerPeriod() const = 0;

    /**
     * Get the PID parameters of the joint.
     *
     * If no PID parameters have been set, the default parameters are
     * returned.
     *
     * @return The joint PID parameters.
     */
    virtual PID pid() const = 0;

    /**
     * Set the PID parameters of the joint.
     *
     * @param pid The desired PID parameters.
     * @return True for success, false otherwise.
     */
    virtual bool setPID(const PID& pid) = 0;

    /**
     * Check if the history of applied joint forces is enabled.
     *
     * @return True if the history is enabled, false otherwise.
     */
    virtual bool historyOfAppliedJointForcesEnabled() const = 0;

    /**
     * Enable the history of joint forces.
     *
     * @param enable True to enable, false to disable.
     * @param maxHistorySize The size of the history window.
     * @return True for success, false otherwise.
     */
    virtual bool enableHistoryOfAppliedJointForces( //
        const bool enable = true,
        const size_t maxHistorySize = 100) = 0;

    /**
     * Get the history of applied joint forces.
     *
     * The vector is populated with #DoFs values at each physics step.
     *
     * @return The vector containing the history of joint forces.
     */
    virtual std::vector<double> historyOfAppliedJointForces() const = 0;

    /**
     * Get the Coulomb friction of the joint.
     *
     * If \f$ K_c \f$ is the Coulomb friction parameter, and \f$ \dot{q} \f$
     * the joint velocity, the corresponding torque is often modelled as:
     *
     * \f$ \tau_{static} = sign(\dot{q}) K_c \f$
     *
     * @return The Coulomb friction parameter of the joint.
     */
    virtual double coulombFriction() const = 0;

    /**
     * Get the viscous friction of the joint.
     *
     * If \f$ K_v \f$ is the viscous friction parameter, and \f$ \dot{q} \f$
     * the joint velocity, the corresponding torque is often modelled as:
     *
     * \f$ \tau_{static} = K_v \dot{q} \f$
     *
     * @return The viscous friction parameter of the joint.
     */
    virtual double viscousFriction() const = 0;

    // ==================
    // Single DOF methods
    // ==================

    /**
     * Get the position limits of a joint DOF.
     *
     * @param dof The index of the DOF.
     * @throw std::runtime_error if the DOF is not valid.
     * @return The position limits of the joint DOF.
     */
    virtual Limit positionLimit(const size_t dof = 0) const = 0;

    /**
     * Get the maximum generalized force that could be applied to a joint
     * DOF.
     *
     * @param dof The index of the DOF.
     * @throw std::runtime_error if the DOF is not valid.
     * @return The maximum generalized force of the joint DOF.
     */
    virtual double maxGeneralizedForce(const size_t dof = 0) const = 0;

    /**
     * Set the maximum generalized force that can be applied to a joint DOF.
     *
     * This limit can be used to clip the force applied by joint
     * controllers.
     *
     * @param maxForce The maximum generalized force.
     * @param dof The index of the DOF.
     * @return True for success, false otherwise.
     */
    virtual bool setMaxGeneralizedForce(const double maxForce,
                                        const size_t dof = 0) = 0;

    /**
     * Get the position of a joint DOF.
     *
     * @param dof The index of the DOF.
     * @throw std::runtime_error if the DOF is not valid.
     * @return The position of the joint DOF.
     */
    virtual double position(const size_t dof = 0) const = 0;

    /**
     * Get the velocity of a joint DOF.
     *
     * @param dof The index of the DOF.
     * @throw std::runtime_error if the DOF is not valid.
     * @return The velocity of the joint DOF.
     */
    virtual double velocity(const size_t dof = 0) const = 0;

    /**
     * Get the acceleration of a joint DOF.
     *
     * @param dof The index of the DOF.
     * @throw std::runtime_error if the DOF is not valid.
     * @return The acceleration of the joint DOF.
     */
    virtual double acceleration(const size_t dof = 0) const = 0;

    /**
     * Get the generalized force of a joint DOF.
     *
     * @param dof The index of the DOF.
     * @throw std::runtime_error if the DOF is not valid.
     * @return The generalized force of the joint DOF.
     */
    virtual double generalizedForce(const size_t dof = 0) const = 0;

    /**
     * Set the position target of a joint DOF.
     *
     * The target is processed by a joint controller, if enabled.
     *
     * @param position The position target of the joint DOF.
     * @param dof The index of the DOF.
     * @throw std::runtime_error if the DOF is not valid.
     * @return True for success, false otherwise.
     */
    virtual bool setPositionTarget(const double position,
                                   const size_t dof = 0) = 0;

    /**
     * Set the velocity target of a joint DOF.
     *
     * The target is processed by a joint controller, if enabled.
     *
     * @param velocity The velocity target of the joint DOF.
     * @param dof The index of the DOF.
     * @return True for success, false otherwise.
     */
    virtual bool setVelocityTarget(const double velocity,
                                   const size_t dof = 0) = 0;

    /**
     * Set the acceleration target of a joint DOF.
     *
     * The target is processed by a joint controller, if enabled.
     *
     * @param acceleration The acceleration target of the joint DOF.
     * @param dof The index of the DOF.
     * @return True for success, false otherwise.
     */
    virtual bool setAccelerationTarget(const double acceleration,
                                       const size_t dof = 0) = 0;

    /**
     * Set the generalized force target of a joint DOF.
     *
     * The force is applied to the desired DOF. Note that if there's
     * friction or other loss components, the real joint force will differ.
     *
     * @param force The generalized force target of the joint DOF.
     * @param dof The index of the DOF.
     * @return True for success, false otherwise.
     */
    virtual bool setGeneralizedForceTarget(const double force,
                                           const size_t dof = 0) = 0;

    /**
     * Get the active position target of the joint DOF.
     *
     * @param dof The index of the DOF.
     * @throw std::runtime_error if the DOF is not valid or if no position
     * target was set.
     * @return The position target of the joint DOF.
     */
    virtual double positionTarget(const size_t dof = 0) const = 0;

    /**
     * Get the active velocity target of the joint DOF.
     *
     * @param dof The index of the DOF.
     * @throw std::runtime_error if the DOF is not valid or if no velocity
     * target was set.
     * @return The velocity target of the joint DOF.
     */
    virtual double velocityTarget(const size_t dof = 0) const = 0;

    /**
     * Get the active acceleration target of the joint DOF.
     *
     * @param dof The index of the DOF.
     * @throw std::runtime_error if the DOF is not valid or if no
     * acceleration target was set.
     * @return The acceleration target of the joint DOF.
     */
    virtual double accelerationTarget(const size_t dof = 0) const = 0;

    /**
     * Get the active generalized force target of the joint DOF.
     *
     * @param dof The index of the DOF.
     * @throw std::runtime_error if the DOF is not valid or if no
     * generalized force target was set.
     * @return The generalized force target of the joint DOF.
     */
    virtual double generalizedForceTarget(const size_t dof = 0) const = 0;

    // =================
    // Multi DOF methods
    // =================

    /**
     * Get the position limits of the joint.
     *
     * @return The position limits of the joint.
     */
    virtual JointLimit jointPositionLimit() const = 0;

    /**
     * Get the maximum generalized force that could be applied to the joint.
     *
     * @return The maximum generalized force of the joint.
     */
    virtual std::vector<double> jointMaxGeneralizedForce() const = 0;

    /**
     * Set the maximum generalized force that can be applied to the joint.
     *
     * This limit can be used to clip the force applied by joint
     * controllers.
     *
     * @param maxForce A vector with the maximum generalized forces of the
     * joint DOFs.
     * @return True for success, false otherwise.
     */
    virtual bool
    setJointMaxGeneralizedForce(const std::vector<double>& maxForce) = 0;

    /**
     * Get the position of the joint.
     *
     * @return The position of the joint.
     */
    virtual std::vector<double> jointPosition() const = 0;

    /**
     * Get the velocity of the joint.
     *
     * @return The velocity of the joint.
     */
    virtual std::vector<double> jointVelocity() const = 0;

    /**
     * Get the acceleration of the joint.
     *
     * @return The acceleration of the joint.
     */
    virtual std::vector<double> jointAcceleration() const = 0;

    /**
     * Get the generalized force of the joint.
     *
     * @return The generalized force of the joint.
     */
    virtual std::vector<double> jointGeneralizedForce() const = 0;

    /**
     * Set the position target of the joint.
     *
     * The target is processed by a joint controller, if enabled.
     *
     * @param position A vector with the position targets of the joint DOFs.
     * @return True for success, false otherwise.
     */
    virtual bool
    setJointPositionTarget(const std::vector<double>& position) = 0;

    /**
     * Set the velocity target of the joint.
     *
     * The target is processed by a joint controller, if enabled.
     *
     * @param velocity A vector with the velocity targets of the joint DOFs.
     * @return True for success, false otherwise.
     */
    virtual bool
    setJointVelocityTarget(const std::vector<double>& velocity) = 0;

    /**
     * Set the acceleration target of the joint.
     *
     * The target is processed by a joint controller, if enabled.
     *
     * @param acceleration A vector with the acceleration targets of the
     * joint DOFs.
     * @return True for success, false otherwise.
     */
    virtual bool
    setJointAccelerationTarget(const std::vector<double>& acceleration) = 0;

    /**
     * Set the generalized force target of the joint.
     *
     * Note that if there's friction or other loss components, the real
     * joint force will differ.
     *
     * @param force A vector with the generalized force targets of the joint
     * DOFs.
     * @return True for success, false otherwise.
     */
    virtual bool
    setJointGeneralizedForceTarget(const std::vector<double>& force) = 0;

    /**
     * Get the active position target.
     *
     * @return The position target of the joint.
     */
    virtual std::vector<double> jointPositionTarget() const = 0;

    /**
     * Get the active velocity target.
     *
     * @return The velocity target of the joint.
     */
    virtual std::vector<double> jointVelocityTarget() const = 0;

    /**
     * Get the active acceleration target.
     *
     * @return The acceleration target of the joint.
     */
    virtual std::vector<double> jointAccelerationTarget() const = 0;

    /**
     * Get the active generalized force target.
     *
     * @return The generalized force target of the joint.
     */
    virtual std::vector<double> jointGeneralizedForceTarget() const = 0;
};

struct scenario::core::PID
{
    PID() = default;

    PID(const double _p, const double _i, const double _d)
        : p(_p)
        , i(_i)
        , d(_d)
    {}

    double p = 0;
    double i = 0;
    double d = 0;
    double cmdMin = std::numeric_limits<double>::lowest();
    double cmdMax = std::numeric_limits<double>::max();
    double cmdOffset = 0;
    double iMin = std::numeric_limits<double>::lowest();
    double iMax = std::numeric_limits<double>::max();
};

struct scenario::core::Limit
{
    Limit() = default;
    Limit(const double _min, const double _max)
        : min(_min)
        , max(_max)
    {}

    double min = std::numeric_limits<double>::lowest();
    double max = std::numeric_limits<double>::max();
};

struct scenario::core::JointLimit
{
    JointLimit(const size_t dofs = 0)
    {
        constexpr double m = std::numeric_limits<double>::lowest();
        constexpr double M = std::numeric_limits<double>::max();

        min = std::vector<double>(dofs, m);
        max = std::vector<double>(dofs, M);
    }

    JointLimit(const std::vector<double>& _min, const std::vector<double>& _max)
        : JointLimit(std::min(_min.size(), _max.size()))
    {
        if (_min.size() != _max.size()) {
            return;
        }

        min = _min;
        max = _max;
    }

    std::vector<double> min;
    std::vector<double> max;
};

#endif // SCENARIO_CORE_JOINT_H
