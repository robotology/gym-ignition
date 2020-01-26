/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "gympp/controllers/ComputedTorqueFixedBase.h"
#include "gympp/Log.h"
#include "gympp/controllers/PositionControllerReferences.h"

#include <Eigen/Dense>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/ModelIO/ModelLoader.h>

using namespace gympp::controllers;

class ComputedTorqueFixedBase::Impl
{
public:
    class Buffers;

    std::string urdfFile;
    gympp::RobotPtr robot;
    std::vector<std::string> controlledJoints;

    std::vector<double> kpInitial;
    std::vector<double> kdInitial;
    std::unordered_map<std::string, JointControlMode> initialControlMode;

    PositionControllerReferences references;
    std::unique_ptr<Buffers> buffers;
    std::unique_ptr<iDynTree::KinDynComputations> kinDyn;

    static Eigen::Map<Eigen::VectorXd> toEigen(const std::vector<double>& vector)
    {
        return {const_cast<double*>(vector.data()), Eigen::Index(vector.size())};
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
        s_ddot_star = Eigen::VectorXd(controlledDofs);
        positionError = Eigen::VectorXd(controlledDofs);
        velocityError = Eigen::VectorXd(controlledDofs);
    }

    iDynTree::Vector3 gravity;
    iDynTree::MatrixDynSize massMatrix;
    iDynTree::VectorDynSize jointPositions;
    iDynTree::VectorDynSize jointVelocities;
    iDynTree::FreeFloatingGeneralizedTorques biasForces;

    Eigen::ArrayXd kp;
    Eigen::ArrayXd kd;

    Eigen::VectorXd torques;
    Eigen::VectorXd s_ddot_star;
    Eigen::VectorXd positionError;
    Eigen::VectorXd velocityError;
};

ComputedTorqueFixedBase::ComputedTorqueFixedBase(const std::string& urdfFile,
                                                 gympp::RobotPtr robot,
                                                 const std::vector<double>& kp,
                                                 const std::vector<double>& kd,
                                                 const std::vector<std::string>& controlledJoints)
    : pImpl{std::make_unique<Impl>()}
{
    pImpl->robot = robot;
    pImpl->urdfFile = urdfFile;
    pImpl->controlledJoints = controlledJoints;
    assert(robot->valid());
    assert(!pImpl->urdfFile.empty());
    assert(robot->dofs() == controlledJoints.size() || controlledJoints.empty());

    pImpl->kpInitial = kp;
    pImpl->kdInitial = kd;
    assert(robot->dofs() == kp.size());
    assert(kp.size() == kd.size());
}

ComputedTorqueFixedBase::~ComputedTorqueFixedBase() = default;

bool ComputedTorqueFixedBase::initialize()
{
    gymppDebug << "Initializing ComputedTorqueFixedBaseCpp" << std::endl;

    if (pImpl->kinDyn) {
        gymppWarning << "The KinDynComputations object has been already initialized" << std::endl;
        return true;
    }

    if (!(pImpl->robot && pImpl->robot->valid())) {
        gymppError << "Couldn't initialize controller. Robot not valid." << std::endl;
        return false;
    }

    if (pImpl->controlledJoints.empty()) {
        gymppDebug << "No list of controlled joints. Controlling all the robots joints."
                   << std::endl;

        // Read the joint names from the robot object.
        // This is useful to use the same joint serialization in the vectorized methods.
        pImpl->controlledJoints = pImpl->robot->jointNames();
    }

    iDynTree::ModelLoader loader;
    if (!loader.loadReducedModelFromFile(pImpl->urdfFile, pImpl->controlledJoints)) {
        gymppError << "Failed to load reduced model from the urdf file" << std::endl;
        return false;
    }

    pImpl->kinDyn = std::make_unique<iDynTree::KinDynComputations>();
    pImpl->kinDyn->setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION);

    if (!pImpl->kinDyn->loadRobotModel(loader.model())) {
        gymppError << "Failed to insert model in the KinDynComputations object" << std::endl;
        return false;
    }

    // Set controlled joints in torque control mode
    for (const auto& jointName : pImpl->controlledJoints) {
        pImpl->initialControlMode[jointName] = pImpl->robot->jointControlMode(jointName);

        if (!pImpl->robot->setJointControlMode(jointName, JointControlMode::Torque)) {
            gymppError << "Failed to control joint '" << jointName << "' in Torque" << std::endl;
            return false;
        }
    }

    // Initialize buffers
    gymppDebug << "Controlling " << pImpl->controlledJoints.size() << " DoFs" << std::endl;
    pImpl->buffers = std::make_unique<Impl::Buffers>(pImpl->controlledJoints.size());

    pImpl->buffers->kp = Impl::toEigen(pImpl->kpInitial);
    pImpl->buffers->kd = Impl::toEigen(pImpl->kdInitial);
    pImpl->buffers->gravity.zero(); // TODO: if we randomize gravity, this has to be exposed
    pImpl->buffers->gravity(2) = -9.8182;
    pImpl->buffers->biasForces.resize(loader.model());

    return true;
}

std::optional<std::vector<double>> ComputedTorqueFixedBase::step()
{
    // ==================
    // KINDYNCOMPUTATIONS
    // ==================

    for (unsigned i = 0; i < pImpl->controlledJoints.size(); ++i) {
        const auto& jointName = pImpl->controlledJoints[i];
        pImpl->buffers->jointPositions.setVal(i, pImpl->robot->jointPosition(jointName));
        pImpl->buffers->jointVelocities.setVal(i, pImpl->robot->jointVelocity(jointName));
    }

    if (!pImpl->kinDyn->setRobotState(pImpl->buffers->jointPositions,
                                      pImpl->buffers->jointVelocities,
                                      pImpl->buffers->gravity)) {
        gymppError << "Failed to set the robot state" << std::endl;
        return {};
    }

    if (!pImpl->kinDyn->getFreeFloatingMassMatrix(pImpl->buffers->massMatrix)) {
        gymppError << "Failed to get the mass matrix" << std::endl;
        return {};
    }

    if (!pImpl->kinDyn->generalizedBiasForces(pImpl->buffers->biasForces)) {
        gymppError << "Failed to get the bias forces " << std::endl;
        return {};
    }

    // ===================
    // INTERMEDIATE VALUES
    // ===================

    const auto nrControlledDofs = pImpl->buffers->jointPositions.size();

    auto Mfloating = iDynTree::toEigen(pImpl->buffers->massMatrix);

    auto h = iDynTree::toEigen(pImpl->buffers->biasForces.jointTorques());
    auto M = Mfloating.bottomRightCorner(nrControlledDofs, nrControlledDofs);
    assert(h.size() == nrControlledDofs);
    assert(M.size() == nrControlledDofs * nrControlledDofs);

    auto s = iDynTree::toEigen(pImpl->buffers->jointPositions);
    auto s_dot = iDynTree::toEigen(pImpl->buffers->jointVelocities);
    assert(s.size() == nrControlledDofs);
    assert(s_dot.size() == nrControlledDofs);

    auto s_ref = Impl::toEigen(pImpl->references.joints.position);
    auto s_ref_dot = Impl::toEigen(pImpl->references.joints.velocity);
    auto s_ref_ddot = Impl::toEigen(pImpl->references.joints.acceleration);
    assert(s_ref.size() == nrControlledDofs);
    assert(s_ref_dot.size() == nrControlledDofs);
    assert(s_ref_ddot.size() == nrControlledDofs);

    auto& kp = pImpl->buffers->kp;
    auto& kd = pImpl->buffers->kd;
    auto& s_tilde = pImpl->buffers->positionError;
    auto& s_dot_tilde = pImpl->buffers->velocityError;
    assert(kp.size() == nrControlledDofs);
    assert(kd.size() == nrControlledDofs);
    assert(s_tilde.size() == nrControlledDofs);
    assert(s_dot_tilde.size() == nrControlledDofs);

    auto& tau = pImpl->buffers->torques;
    auto& s_ddot_star = pImpl->buffers->s_ddot_star;
    assert(tau.size() == nrControlledDofs);
    assert(s_ddot_star.size() == nrControlledDofs);

    // ===========
    // CONTROL LAW
    // ===========

    s_tilde = s - s_ref;
    s_dot_tilde = s_dot - s_ref_dot;

    s_ddot_star = s_ref_ddot.array() - kp * s_tilde.array() - kd * s_dot_tilde.array();

    tau = M * pImpl->buffers->s_ddot_star + h;

    return std::vector<double>(pImpl->buffers->torques.data(),
                               pImpl->buffers->torques.data() + pImpl->buffers->torques.size());
}

bool ComputedTorqueFixedBase::terminate()
{
    bool ok = true;

    for (const auto& [jointName, controlMode] : pImpl->initialControlMode) {
        if (!pImpl->robot->setJointControlMode(jointName, controlMode)) {
            gymppError << "Failed to restore original control mode of joint '" << jointName << "'"
                       << std::endl;
            ok = false;
        }
    }

    pImpl->kinDyn.reset();
    pImpl->buffers.reset();
    return ok;
}

bool ComputedTorqueFixedBase::setReferences(const PositionControllerReferences& references)
{
    if (!references.valid()) {
        gymppError << "References are not valid" << std::endl;
        return false;
    }

    pImpl->references = references;
    return true;
}
