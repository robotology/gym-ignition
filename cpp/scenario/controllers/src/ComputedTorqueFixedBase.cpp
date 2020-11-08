/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "scenario/controllers/ComputedTorqueFixedBase.h"
#include "scenario/controllers/References.h"
#include "scenario/core/Joint.h"
#include "scenario/core/Model.h"
#include "scenario/core/utils/Log.h"

#include <Eigen/Dense>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/ModelIO/ModelLoader.h>

#include <cassert>
#include <unordered_map>

using namespace scenario::controllers;

class ComputedTorqueFixedBase::Impl
{
public:
    class Buffers;

    std::string urdfFile;

    struct
    {
        std::vector<double> kp;
        std::vector<double> kd;
        std::array<double, 3> gravity;
        std::unordered_map<std::string, core::JointControlMode> controlMode;
    } initialValues;

    JointReferences jointReferences;
    std::unique_ptr<Buffers> buffers;
    std::unique_ptr<iDynTree::KinDynComputations> kinDyn;

    static Eigen::Map<Eigen::VectorXd> toEigen(std::vector<double>& vector)
    {
        return {vector.data(), Eigen::Index(vector.size())};
    }
};

class ComputedTorqueFixedBase::Impl::Buffers
{
public:
    Buffers(const unsigned controlledDofs = 0)
    {
        jointPositions.resize(controlledDofs);
        jointVelocities.resize(controlledDofs);
        massMatrix.resize(controlledDofs + 6, controlledDofs + 6);

        kp = Eigen::ArrayXd(controlledDofs);
        kd = Eigen::ArrayXd(controlledDofs);

        torques = Eigen::VectorXd(controlledDofs);
        dds_star = Eigen::VectorXd(controlledDofs);
        positionError = Eigen::VectorXd(controlledDofs);
        velocityError = Eigen::VectorXd(controlledDofs);

        torquesVector.reserve(controlledDofs);
    }

    iDynTree::Vector3 gravity = {g.data(), 3};
    iDynTree::MatrixDynSize massMatrix;
    iDynTree::VectorDynSize jointPositions;
    iDynTree::VectorDynSize jointVelocities;
    iDynTree::FreeFloatingGeneralizedTorques biasForces;

    Eigen::ArrayXd kp;
    Eigen::ArrayXd kd;

    Eigen::VectorXd torques;
    Eigen::VectorXd dds_star;
    Eigen::VectorXd positionError;
    Eigen::VectorXd velocityError;

    std::vector<double> torquesVector;
};

ComputedTorqueFixedBase::ComputedTorqueFixedBase(
    const std::string& urdfFile,
    std::shared_ptr<core::Model> model,
    const std::vector<double>& kp,
    const std::vector<double>& kd,
    const std::vector<std::string>& controlledJoints,
    const std::array<double, 3> gravity)
    : Controller()
    , UseScenarioModel()
    , SetJointReferences()
    , pImpl{std::make_unique<Impl>()}
{
    m_model = model;
    pImpl->urdfFile = urdfFile;
    m_controlledJoints = controlledJoints;

    if (m_controlledJoints.size() == 0) {
        sDebug << "No list of controlled joints found. "
               << "Controlling all the robots joints." << std::endl;
        // Note: the joint serialization is now given by the default
        //       list of joint names provided by the model
        m_controlledJoints = m_model->jointNames();
    }

    pImpl->initialValues.gravity = gravity;

    pImpl->initialValues.kp = kp;
    pImpl->initialValues.kd = kd;
    assert(m_controlledJoints.size() == kp.size());
    assert(kp.size() == kd.size());
}

ComputedTorqueFixedBase::~ComputedTorqueFixedBase() = default;

bool ComputedTorqueFixedBase::initialize()
{
    sDebug << "Initializing ComputedTorqueFixedBaseCpp" << std::endl;

    if (pImpl->kinDyn) {
        sWarning << "The KinDynComputations object has been already initialized"
                 << std::endl;
        return true;
    }

    if (!(m_model && m_model->valid())) {
        sError << "Couldn't initialize controller. Model not valid."
               << std::endl;
        return false;
    }

    if (m_controlledJoints.empty()) {
        sError << "The list of controlled joints is not valid" << std::endl;
        return false;
    }

    if (m_controlledJoints.size() != m_model->dofs()) {
        sError << "Controlling only a subset of joints is not yet supported"
               << std::endl;
        return false;
    }

    for (auto& joint : m_model->joints(m_controlledJoints)) {
        if (joint->dofs() != 1) {
            sError << "Joint '" << joint->name()
                   << "' does not have 1 DoF and is not supported" << std::endl;
            return false;
        }
    }

    iDynTree::ModelLoader loader;
    if (!loader.loadReducedModelFromFile(pImpl->urdfFile, m_controlledJoints)) {
        sError << "Failed to load reduced model from the urdf file"
               << std::endl;
        return false;
    }

    pImpl->kinDyn = std::make_unique<iDynTree::KinDynComputations>();
    pImpl->kinDyn->setFrameVelocityRepresentation(
        iDynTree::MIXED_REPRESENTATION);

    if (!pImpl->kinDyn->loadRobotModel(loader.model())) {
        sError << "Failed to insert model in the KinDynComputations object"
               << std::endl;
        return false;
    }

    // Set controlled joints in torque control mode
    for (auto& joint : m_model->joints(m_controlledJoints)) {
        pImpl->initialValues.controlMode[joint->name()] = joint->controlMode();

        if (!joint->setControlMode(core::JointControlMode::Force)) {
            sError << "Failed to control joint '" << joint->name()
                   << "' in Force" << std::endl;
            return false;
        }
    }

    // Initialize buffers
    sDebug << "Controlling " << m_controlledJoints.size() << " DoFs"
           << std::endl;
    pImpl->buffers = std::make_unique<Impl::Buffers>(m_controlledJoints.size());

    pImpl->buffers->kp = Impl::toEigen(pImpl->initialValues.kp);
    pImpl->buffers->kd = Impl::toEigen(pImpl->initialValues.kd);
    pImpl->buffers->biasForces.resize(loader.model());

    // Set the gravity
    pImpl->buffers->gravity[0] = pImpl->initialValues.gravity[0];
    pImpl->buffers->gravity[1] = pImpl->initialValues.gravity[1];
    pImpl->buffers->gravity[2] = pImpl->initialValues.gravity[2];

    return true;
}

bool ComputedTorqueFixedBase::step(const Controller::StepSize& /*dt*/)
{
    // ===================
    // Intermediate Values
    // ===================

    const auto nrControlledDofs = pImpl->buffers->jointPositions.size();

    auto Mfloating = iDynTree::toEigen(pImpl->buffers->massMatrix);

    auto h = iDynTree::toEigen(pImpl->buffers->biasForces.jointTorques());
    auto M = Mfloating.bottomRightCorner(nrControlledDofs, nrControlledDofs);
    assert(h.size() == nrControlledDofs);
    assert(M.size() == nrControlledDofs * nrControlledDofs);

    auto s = iDynTree::toEigen(pImpl->buffers->jointPositions);
    auto ds = iDynTree::toEigen(pImpl->buffers->jointVelocities);
    assert(s.size() == nrControlledDofs);
    assert(ds.size() == nrControlledDofs);

    auto s_ref = Impl::toEigen(pImpl->jointReferences.position);
    auto ds_ref = Impl::toEigen(pImpl->jointReferences.velocity);
    auto dds_ref = Impl::toEigen(pImpl->jointReferences.acceleration);
    assert(s_ref.size() == nrControlledDofs);
    assert(ds_ref.size() == nrControlledDofs);
    assert(dds_ref.size() == nrControlledDofs);

    auto& kp = pImpl->buffers->kp;
    auto& kd = pImpl->buffers->kd;
    auto& s_tilde = pImpl->buffers->positionError;
    auto& ds_tilde = pImpl->buffers->velocityError;
    assert(kp.size() == nrControlledDofs);
    assert(kd.size() == nrControlledDofs);
    assert(s_tilde.size() == nrControlledDofs);
    assert(ds_tilde.size() == nrControlledDofs);

    auto& tau = pImpl->buffers->torques;
    auto& dds_star = pImpl->buffers->dds_star;
    assert(tau.size() == nrControlledDofs);
    assert(dds_star.size() == nrControlledDofs);

    // ===========
    // Control Law
    // ===========

    // Compute errors
    s_tilde = s - s_ref;
    ds_tilde = ds - ds_ref;

    // Compute the acceleration
    dds_star = dds_ref.array() - kp * s_tilde.array() - kd * ds_tilde.array();

    // Compute the torque
    tau = M * dds_star + h;

    pImpl->buffers->torquesVector = std::vector<double>(
        pImpl->buffers->torques.data(),
        pImpl->buffers->torques.data() + pImpl->buffers->torques.size());

    if (!m_model->setJointGeneralizedForceTargets(pImpl->buffers->torquesVector,
                                                  m_controlledJoints)) {
        sError << "Failed to set joint forces" << std::endl;
        return false;
    }

    return true;
}

bool ComputedTorqueFixedBase::terminate()
{
    bool ok = true;

    for (const auto& [jointName, controlMode] :
         pImpl->initialValues.controlMode) {

        auto joint = m_model->getJoint(jointName);

        if (!joint->setControlMode(controlMode)) {
            sError << "Failed to restore original control mode of joint '"
                   << jointName << "'" << std::endl;
            ok = ok && false;
        }
    }

    pImpl->kinDyn.reset();
    pImpl->buffers.reset();
    return ok;
}

bool ComputedTorqueFixedBase::updateStateFromModel()
{
    assert(m_model->jointPositions(m_controlledJoints).size()
           == pImpl->buffers->jointPositions.size());
    assert(m_model->jointVelocities(m_controlledJoints).size()
           == pImpl->buffers->jointVelocities.size());

    for (unsigned i = 0; i < m_controlledJoints.size(); ++i) {
        // Get the joint
        const auto& jointName = m_controlledJoints[i];
        auto joint = m_model->getJoint(jointName);
        assert(joint->dofs() == 1);

        // Update the buffers
        pImpl->buffers->jointPositions.setVal(i, joint->position());
        pImpl->buffers->jointVelocities.setVal(i, joint->velocity());
    }

    if (!pImpl->kinDyn->setRobotState(pImpl->buffers->jointPositions,
                                      pImpl->buffers->jointVelocities,
                                      pImpl->buffers->gravity)) {
        sError << "Failed to set the robot state" << std::endl;
        return false;
    }

    if (!pImpl->kinDyn->getFreeFloatingMassMatrix(pImpl->buffers->massMatrix)) {
        sError << "Failed to get the mass matrix" << std::endl;
        return false;
    }

    if (!pImpl->kinDyn->generalizedBiasForces(pImpl->buffers->biasForces)) {
        sError << "Failed to get the bias forces " << std::endl;
        return false;
    }

    return true;
}

const std::vector<std::string>& ComputedTorqueFixedBase::controlledJoints()
{
    return m_controlledJoints;
}

bool ComputedTorqueFixedBase::setJointReferences(
    const JointReferences& jointReferences)
{
    pImpl->jointReferences = jointReferences;
    return pImpl->jointReferences.valid();
};
