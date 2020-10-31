/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef SCENARIO_CORE_MODEL_H
#define SCENARIO_CORE_MODEL_H

#include "scenario/core/Joint.h"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace scenario::core {
    struct Contact;
    class Link;
    class Model;
    using LinkPtr = std::shared_ptr<Link>;
    using JointPtr = std::shared_ptr<Joint>;
    using ModelPtr = std::shared_ptr<Model>;
} // namespace scenario::core

class scenario::core::Model
{
public:
    Model() = default;
    virtual ~Model() = default;

    /**
     * Check if the model is valid.
     *
     * @return True if the model is valid, false otherwise.
     */
    virtual bool valid() const = 0;

    /**
     * Get the degrees of freedom of the model.
     *
     * @param jointNames Optionally restrict the count to a subset of
     * joints.
     * @return The number of degrees of freedom of the model.
     */
    virtual size_t
    dofs(const std::vector<std::string>& jointNames = {}) const = 0;

    /**
     * Get the name of the model.
     *
     * @return The name of the model.
     */
    virtual std::string name() const = 0;

    /**
     * Get the number of links of the model.
     *
     * @return The number of links.
     */
    virtual size_t nrOfLinks() const = 0;

    /**
     * Get the number of joints of the model.
     *
     * @return The number of joints.
     */
    virtual size_t nrOfJoints() const = 0;

    /**
     * Get the total mass of the model.
     *
     * @param linkNames Optionally restrict the count to a subset of links.
     * @return The total mass of the model.
     */
    virtual double
    totalMass(const std::vector<std::string>& linkNames = {}) const = 0;

    /**
     * Get a link belonging to the model.
     *
     * @param linkName The name of the link.
     * @throw std::runtime_error if the link does not exist.
     * @return The desired link.
     */
    virtual LinkPtr getLink(const std::string& linkName) const = 0;

    /**
     * Get a joint belonging to the model.
     *
     * @param jointName The name of the joint.
     * @throw std::runtime_error if the joint does not exist.
     * @return The desired joint.
     */
    virtual JointPtr getJoint(const std::string& jointName) const = 0;

    /**
     * Get the name of all the model's links.
     *
     * @param scoped Scope the link names with the model name
     * (e.g. ``mymodel::link1``).
     * @return The list of link names.
     */
    virtual std::vector<std::string>
    linkNames(const bool scoped = false) const = 0;

    /**
     * Get the name of all the model's joints.
     *
     * @param scoped Scope the joint names with the model name,
     * (e.g. ``mymodel::joint1``).
     * @return The list of joint names.
     */
    virtual std::vector<std::string>
    jointNames(const bool scoped = false) const = 0;

    /**
     * Get the controller period of the model.
     *
     * If no controller has been enabled, infinite is returned.
     *
     * @return The controller period of the model.
     */
    virtual double controllerPeriod() const = 0;

    /**
     * Set the controller period of the model.
     *
     * This controller period is used by PIDs and custom controller.
     * If it is smaller than the physics step, it is treated as 0.
     *
     * @param period The desired controller period.
     * @return True for success, false otherwise.
     */
    virtual bool setControllerPeriod(const double period) = 0;

    /**
     * Enable logging the applied joint forces.
     *
     * The output of joint controllers is often a torque. This method allows to
     * log the force references that the controller sent to the joints. It is
     * useful when the controller runs in its own thread at its own rate and the
     * caller wants to extract the forces at a lower frequency.
     *
     * @param enable True to enable logging, false to disable.
     * @param maxHistorySizePerJoint Size of the logging window of each joint.
     * @param jointNames Optional vector of considered joints that also
     * defines the joint serialization. By default, ``Model::jointNames`` is
     * used.
     * @return True for success, false otherwise.
     */
    virtual bool enableHistoryOfAppliedJointForces( //
        const bool enable = true,
        const size_t maxHistorySizePerJoint = 100,
        const std::vector<std::string>& jointNames = {}) = 0;

    /**
     * Check if logging the applied joint force is enabled.
     *
     * @param jointNames Optional vector of considered joints that also
     * defines the joint serialization. By default, ``Model::jointNames`` is
     * used.
     * @return True if the log is enabled, false otherwise.
     */
    virtual bool historyOfAppliedJointForcesEnabled(
        const std::vector<std::string>& jointNames) const = 0;

    /**
     * Get the log of applied joint forces.
     *
     * @param jointNames Optional vector of considered joints that also
     * defines the joint serialization. By default, ``Model::jointNames`` is
     * used.
     * @return The entire window of applied joint forces.
     *
     * @note Given a serialization, the window has ``DoFs * JointWindowSize``
     * elements. The elements are ordered per time steps, i.e. the first
     * ``#DoFs`` elements refer to the oldest forces of the windows ordered with
     * the active joint serialization.
     *
     * @note If a joint has multiple DoFs, they are serialized contiguously.
     */
    virtual std::vector<double> historyOfAppliedJointForces(
        const std::vector<std::string>& jointNames = {}) const = 0;

    // ========
    // Contacts
    // ========

    /**
     * Check if the contact detection is enabled model-wise.
     *
     * @return True if the contact detection is enabled model-wise, false
     * otherwise.
     */
    virtual bool contactsEnabled() const = 0;

    /**
     * Enable the contact detection model-wise.
     *
     * @param enable True to enable the contact detection model-wise, false
     * to disable.
     * @return True for success, false otherwise.
     */
    virtual bool enableContacts(const bool enable = true) = 0;

    /**
     * Check if the detection of self-collisions is enabled.
     *
     * @return True if self-collisions detection is enabled, false
     * otherwise.
     */
    virtual bool selfCollisionsEnabled() const = 0;

    /**
     * Enable the detection of self-collisions.
     *
     * It will enable contact detection if it was disabled.
     *
     * @param enable True to enable the self-collision detection, false to
     * disable.
     * @return True for success, false otherwise.
     */
    virtual bool enableSelfCollisions(const bool enable = true) = 0;

    /**
     * Get the vector of links with active contacts with other bodies.
     *
     * @return The vector of links in contact.
     */
    virtual std::vector<std::string> linksInContact() const = 0;

    /**
     * Get the active contacts of the model.
     *
     * @param linkNames Optionally restrict the considered links.
     * @return A vector of contacts.
     */
    virtual std::vector<Contact>
    contacts(const std::vector<std::string>& linkNames = {}) const = 0;

    // ==================
    // Vectorized Methods
    // ==================

    /**
     * Get the joint positions.
     *
     * @param jointNames Optional vector of considered joints that also
     * defines the joint serialization. By default, ``Model::jointNames`` is
     * used.
     * @return The serialization of joint positions. The vector has as many
     * elements as DoFs of the considered joints.
     */
    virtual std::vector<double> jointPositions( //
        const std::vector<std::string>& jointNames = {}) const = 0;

    /**
     * Get the joint velocities.
     *
     * @param jointNames Optional vector of considered joints that also
     * defines the joint serialization. By default, ``Model::jointNames`` is
     * used.
     * @return The serialization of joint velocities. The vector has as many
     * elements as DoFs of the considered joints.
     */
    virtual std::vector<double> jointVelocities( //
        const std::vector<std::string>& jointNames = {}) const = 0;

    /**
     * Get the joint accelerations.
     *
     * @param jointNames Optional vector of considered joints that also
     * defines the joint serialization. By default, ``Model::jointNames`` is
     * used.
     * @return The serialization of joint accelerations. The vector has as many
     * elements as DoFs of the considered joints.
     */
    virtual std::vector<double> jointAccelerations( //
        const std::vector<std::string>& jointNames = {}) const = 0;

    /**
     * Get the joint generalized forces.
     *
     * @param jointNames Optional vector of considered joints that also
     * defines the joint serialization. By default, ``Model::jointNames`` is
     * used.
     * @return The serialization of joint forces. The vector has as many
     * elements as DoFs of the considered joints.
     */
    virtual std::vector<double> jointGeneralizedForces( //
        const std::vector<std::string>& jointNames = {}) const = 0;

    /**
     * Get the joint limits of the model.
     *
     * @param jointNames Optional vector of considered joints that also
     * defines the joint serialization. By default, ``Model::jointNames`` is
     * used.
     * @return The joint limits of the model. The vectors of the limit
     * object have as many elements as DoFs of the considered joints.
     */
    virtual JointLimit jointLimits( //
        const std::vector<std::string>& jointNames = {}) const = 0;

    /**
     * Set the control mode of model joints.
     *
     * @param mode The desired joint control mode.
     * @param jointNames Optional vector of considered joints that also
     * defines the joint serialization. By default, ``Model::jointNames`` is
     * used.
     * @return True for success, false otherwise.
     */
    virtual bool
    setJointControlMode(const JointControlMode mode,
                        const std::vector<std::string>& jointNames = {}) = 0;

    /**
     * Get the links of the model.
     *
     * @param linkNames Optional vector of considered links. By default,
     * ``Model::linkNames`` is used.
     * @return A vector of pointers to the link objects.
     */
    virtual std::vector<LinkPtr> links( //
        const std::vector<std::string>& linkNames = {}) const = 0;

    /**
     * Get the joints of the model.
     *
     * @param jointNames Optional vector of considered joints. By default,
     * ``Model::jointNames`` is used.
     * @return A vector of pointers to the joint objects.
     */
    virtual std::vector<JointPtr> joints( //
        const std::vector<std::string>& jointNames = {}) const = 0;

    // =========================
    // Vectorized Target Methods
    // =========================

    /**
     * Set the position targets of the joints.
     *
     * @param positions The vector with the joint position targets. It must
     * have as many elements as the considered joint DoFs.
     * @param jointNames Optional vector of considered joints. By default,
     * ``Model::jointNames`` is used.
     * @return True for success, false otherwise.
     */
    virtual bool setJointPositionTargets( //
        const std::vector<double>& positions,
        const std::vector<std::string>& jointNames = {}) = 0;

    /**
     * Set the velocity targets of the joints.
     *
     * @param velocities The vector with the joint velocity targets. It must
     * have as many elements as the considered joint DoFs.
     * @param jointNames Optional vector of considered joints. By default,
     * ``Model::jointNames`` is used.
     * @return True for success, false otherwise.
     */
    virtual bool setJointVelocityTargets( //
        const std::vector<double>& velocities,
        const std::vector<std::string>& jointNames = {}) = 0;

    /**
     * Set the acceleration targets of the joints.
     *
     * @param accelerations The vector with the joint acceleration targets.
     * It must have as many elements as the considered joint DoFs.
     * @param jointNames Optional vector of considered joints. By default,
     * ``Model::jointNames`` is used.
     * @return True for success, false otherwise.
     */
    virtual bool setJointAccelerationTargets( //
        const std::vector<double>& accelerations,
        const std::vector<std::string>& jointNames = {}) = 0;

    /**
     * Set the generalized force targets of the joints.
     *
     * @param forces The vector with the joint generalized force targets. It
     * must have as many elements as the considered joint DoFs.
     * @param jointNames Optional vector of considered joints. By default,
     * ``Model::jointNames`` is used.
     * @return True for success, false otherwise.
     */
    virtual bool setJointGeneralizedForceTargets( //
        const std::vector<double>& forces,
        const std::vector<std::string>& jointNames = {}) = 0;

    /**
     * Get the position targets of the joints.
     *
     * @param jointNames Optional vector of considered joints. By default,
     * ``Model::jointNames`` is used.
     * @return The position targets of the joints.
     */
    virtual std::vector<double> jointPositionTargets( //
        const std::vector<std::string>& jointNames = {}) const = 0;

    /**
     * Get the velocity targets of the joints.
     *
     * @param jointNames Optional vector of considered joints. By default,
     * ``Model::jointNames`` is used.
     * @return The velocity targets of the joints.
     */
    virtual std::vector<double> jointVelocityTargets( //
        const std::vector<std::string>& jointNames = {}) const = 0;

    /**
     * Get the acceleration targets of the joints.
     *
     * @param jointNames Optional vector of considered joints. By default,
     * ``Model::jointNames`` is used.
     * @return The acceleration targets of the joints.
     */
    virtual std::vector<double> jointAccelerationTargets( //
        const std::vector<std::string>& jointNames = {}) const = 0;

    /**
     * Get the generalized force targets of the joints.
     *
     * @param jointNames Optional vector of considered joints. By default,
     * ``Model::jointNames`` is used.
     * @return The generalized force targets of the joints.
     */
    virtual std::vector<double> jointGeneralizedForceTargets( //
        const std::vector<std::string>& jointNames = {}) const = 0;

    // =========
    // Base Link
    // =========

    /**
     * Get the name of the model's base frame.
     *
     * By default, the base frame is typically the root of the kinematic tree of
     * the model.
     *
     * @return The name of the model's base frame.
     */
    virtual std::string baseFrame() const = 0;

    /**
     * Get the position of the base link.
     *
     * @return The position of the base link in world coordinates.
     */
    virtual std::array<double, 3> basePosition() const = 0;

    /**
     * Get the orientation of the base link.
     *
     * @return The wxyz quaternion defining the orientation of the base link wrt
     * the world frame.
     */
    virtual std::array<double, 4> baseOrientation() const = 0;

    /**
     * Get the linear body velocity of the base link.
     *
     * @todo Add link to the velocity representation documentation page.
     *
     * @return The linear body velocity of the base link.
     */
    virtual std::array<double, 3> baseBodyLinearVelocity() const = 0;

    /**
     * Get the angular body velocity of the base link.
     *
     * @todo Add link to the velocity representation documentation page.
     *
     * @return The angular body velocity of the base link.
     */
    virtual std::array<double, 3> baseBodyAngularVelocity() const = 0;

    /**
     * Get the linear mixed velocity of the base link.
     *
     * @todo Add link to the velocity representation documentation page.
     *
     * @return The linear mixed velocity of the base link.
     */
    virtual std::array<double, 3> baseWorldLinearVelocity() const = 0;

    /**
     * Get the angular mixed velocity of the base link.
     *
     * @todo Add link to the velocity representation documentation page.
     *
     * @return The angular mixed velocity of the base link.
     */
    virtual std::array<double, 3> baseWorldAngularVelocity() const = 0;

    // =================
    // Base Link Targets
    // =================

    /**
     * Set the pose target of the base link.
     *
     * @param position The position target of the base link in world
     * coordinates.
     * @param orientation The wxyz quaternion defining the orientation target of
     * the base link wrt the world frame.
     * @return True for success, false otherwise.
     */
    virtual bool
    setBasePoseTarget(const std::array<double, 3>& position,
                      const std::array<double, 4>& orientation) = 0;

    /**
     * Set the position target of the base link.
     *
     * @param position The position target of the base link in world
     * coordinates.
     * @return True for success, false otherwise.
     */
    virtual bool
    setBasePositionTarget(const std::array<double, 3>& position) = 0;

    /**
     * Set the orientation target of the base link.
     *
     * @param orientation The wxyz quaternion defining the orientation target of
     * the base link wrt the world frame.
     * @return True for success, false otherwise.
     */
    virtual bool
    setBaseOrientationTarget(const std::array<double, 4>& orientation) = 0;

    /**
     * Set the mixed velocity target of the base link.
     *
     * @param linear The mixed linear velocity target of the base link.
     * @param angular The mixed angular velocity target of the base link.
     * @return True for success, false otherwise.
     */
    virtual bool
    setBaseWorldVelocityTarget(const std::array<double, 3>& linear,
                               const std::array<double, 3>& angular) = 0;

    /**
     * Set the mixed linear velocity target of the base link.
     *
     * @param linear The mixed linear velocity target of the base link.
     * @return True for success, false otherwise.
     */
    virtual bool
    setBaseWorldLinearVelocityTarget(const std::array<double, 3>& linear) = 0;

    /**
     * Set the mixed angular velocity target of the base link.
     *
     * @param angular The mixed angular velocity target of the base link.
     * @return True for success, false otherwise.
     */
    virtual bool setBaseWorldAngularVelocityTarget( //
        const std::array<double, 3>& angular) = 0;

    /**
     * Set the mixed linear acceleration target of the base link.
     *
     * @param linear The mixed linear acceleration target of the base link.
     * @return True for success, false otherwise.
     */
    virtual bool setBaseWorldLinearAccelerationTarget( //
        const std::array<double, 3>& linear) = 0;

    /**
     * Set the mixed angular acceleration target of the base link.
     *
     * @param angular The mixed angular acceleration target of the base link.
     * @return True for success, false otherwise.
     */
    virtual bool setBaseWorldAngularAccelerationTarget( //
        const std::array<double, 3>& angular) = 0;

    /**
     * Get the position target of the base link.
     *
     * @return The position target of the base link.
     */
    virtual std::array<double, 3> basePositionTarget() const = 0;

    /**
     * Get the orientation target of the base link.
     *
     * @return The quaternion defining the orientation target of the base link.
     */
    virtual std::array<double, 4> baseOrientationTarget() const = 0;

    /**
     * Get the mixed linear velocity target of the base link.
     *
     * @return The mixed linear velocity target of the base link.
     */
    virtual std::array<double, 3> baseWorldLinearVelocityTarget() const = 0;

    /**
     * Get the mixed angular velocity target of the base link.
     *
     * @return The mixed angular velocity target of the base link.
     */
    virtual std::array<double, 3> baseWorldAngularVelocityTarget() const = 0;

    /**
     * Get the mixed linear acceleration target of the base link.
     *
     * @return The mixed linear acceleration target of the base link.
     */
    virtual std::array<double, 3> baseWorldLinearAccelerationTarget() const = 0;

    /**
     * Get the mixed angular acceleration target of the base link.
     *
     * @return The mixed angular acceleration target of the base link.
     */
    virtual std::array<double, 3>
    baseWorldAngularAccelerationTarget() const = 0;
};

#endif // SCENARIO_CORE_MODEL_H
