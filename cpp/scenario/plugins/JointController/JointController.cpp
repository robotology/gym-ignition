/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This project is dual licensed under LGPL v2.1+ or Apache License.
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "JointController.h"
#include "scenario/gazebo/Joint.h"
#include "scenario/gazebo/Log.h"
#include "scenario/gazebo/Model.h"
#include "scenario/gazebo/components/JointControlMode.h"
#include "scenario/gazebo/components/JointController.h"
#include "scenario/gazebo/components/JointPID.h"
#include "scenario/gazebo/components/JointPositionTarget.h"
#include "scenario/gazebo/components/JointVelocityTarget.h"
#include "scenario/gazebo/helpers.h"

#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/math/PID.hh>
#include <ignition/plugin/Register.hh>

#include <cassert>
#include <chrono>
#include <limits>
#include <ratio>
#include <string>
#include <vector>

using namespace scenario::gazebo;
using namespace scenario::plugins::gazebo;

class JointController::Impl
{
public:
    ignition::gazebo::Entity modelEntity;
    std::shared_ptr<scenario::gazebo::Model> model;
    std::chrono::steady_clock::duration prevUpdateTime{0};

    static bool runPIDController(scenario::gazebo::Joint& joint,
                                 const bool computeNewForce,
                                 ignition::math::PID& pid,
                                 const std::chrono::steady_clock::duration& dt,
                                 const std::vector<double>& reference,
                                 const std::vector<double>& current);
};

JointController::JointController()
    : System()
    , pImpl{std::make_unique<Impl>()}
{}

JointController::~JointController() = default;

void JointController::Configure(
    const ignition::gazebo::Entity& entity,
    const std::shared_ptr<const sdf::Element>& /*sdf*/,
    ignition::gazebo::EntityComponentManager& ecm,
    ignition::gazebo::EventManager& eventMgr)
{
    // Check if the model already has a JointController plugin
    if (ecm.EntityHasComponentType(
            entity, ignition::gazebo::components::JointController::typeId)) {
        sError << "The model already has a JointController plugin" << std::endl;
        return;
    }

    // Store the model entity
    pImpl->modelEntity = entity;

    // Create a model that will be given to the controller
    pImpl->model = std::make_shared<Model>();

    // Create a model and check its validity
    if (!pImpl->model->initialize(entity, &ecm, &eventMgr)) {
        sError << "Failed to initialize model for controller" << std::endl;
        return;
    }

    if (!pImpl->model->valid()) {
        sError << "Failed to create a model from Entity [" << entity << "]"
               << std::endl;
        return;
    }

    // Add the JointController component to the model
    utils::setComponentData<ignition::gazebo::components::JointController>(
        &ecm, entity, true);
}

void JointController::PreUpdate(const ignition::gazebo::UpdateInfo& info,
                                ignition::gazebo::EntityComponentManager& ecm)
{
    if (info.paused) {
        return;
    }

    if (!pImpl->model) {
        return;
    }

    // This plugin keep being called also after the model was removed
    try {
        pImpl->model->controllerPeriod();
    }
    catch (exceptions::ComponentNotFound) {
        return;
    }

    using namespace std::chrono;

    // Update the controller only if enough time is passed
    duration<double> elapsedFromLastUpdate =
        info.simTime - pImpl->prevUpdateTime;
    assert(elapsedFromLastUpdate.count() > 0);

    // Handle first iteration
    if (pImpl->prevUpdateTime.count() == 0) {
        elapsedFromLastUpdate =
            duration<double>(pImpl->model->controllerPeriod());
    }

    // If enough time has passed, store the time of this actuation step. In this
    // case the state of the robot is read and new force references are computed
    // and actuated. Otherwise, the same force of the last step is actuated.
    bool computeNewForce;

    // Due to numerical floating point approximations, sometimes a comparison of
    // chrono durations has an error in the 1e-18 order
    auto greaterThan = [](const duration<double>& a,
                          const duration<double>& b) -> bool {
        return a.count() >= b.count() - std::numeric_limits<double>::epsilon();
    };

    if (greaterThan(elapsedFromLastUpdate,
                    duration<double>(pImpl->model->controllerPeriod()))) {
        // Store the current update time
        pImpl->prevUpdateTime = info.simTime;

        // Enable using the PID to compute the new force
        computeNewForce = true;
    }
    else {
        // Disable the PID and send the same force reference as last update
        computeNewForce = false;
    }

    const auto positionControlledJoints = ecm.EntitiesByComponents(
        ignition::gazebo::components::Joint(),
        ignition::gazebo::components::ParentEntity(pImpl->modelEntity),
        ignition::gazebo::components::JointControlMode(
            core::JointControlMode::Position));

    const auto positionInterpolatedControlledJoints = ecm.EntitiesByComponents(
        ignition::gazebo::components::Joint(),
        ignition::gazebo::components::ParentEntity(pImpl->modelEntity),
        ignition::gazebo::components::JointControlMode(
            core::JointControlMode::PositionInterpolated));

    const auto velocityControlledJoints = ecm.EntitiesByComponents(
        ignition::gazebo::components::Joint(),
        ignition::gazebo::components::ParentEntity(pImpl->modelEntity),
        ignition::gazebo::components::JointControlMode(
            core::JointControlMode::Velocity));

    const auto velocityFollowerDartControlledJoints = ecm.EntitiesByComponents(
        ignition::gazebo::components::Joint(),
        ignition::gazebo::components::ParentEntity(pImpl->modelEntity),
        ignition::gazebo::components::JointControlMode(
            core::JointControlMode::VelocityFollowerDart));

    // Update PIDs for Revolute and Prismatic joints controlled in Position
    for (const auto jointEntity : positionControlledJoints) {

        ignition::math::PID& pid = utils::getExistingComponentData< //
            ignition::gazebo::components::JointPID>(&ecm, jointEntity);

        std::string& jointName = utils::getExistingComponentData< //
            ignition::gazebo::components::Name>(&ecm, jointEntity);

        auto joint = pImpl->model->getJoint(jointName);
        auto jointGazebo = std::static_pointer_cast<Joint>(joint);

        const std::vector<double>& position = joint->jointPosition();

        const std::vector<double>& positionTarget =
            utils::getExistingComponentData<
                ignition::gazebo::components::JointPositionTarget>(&ecm,
                                                                   jointEntity);

        if (!Impl::runPIDController(*jointGazebo,
                                    computeNewForce,
                                    pid,
                                    info.dt,
                                    positionTarget,
                                    position)) {
            sError << "Failed to run PID controller of joint " << joint->name()
                   << " [" << jointEntity << "]" << std::endl;
        }
    }

    // TODO: Update PIDs for Revolute and Prismatic joints controlled in
    //       PositionInterpolated

    // Update PIDs for Revolute and Prismatic joints controlled in Velocity
    for (const auto jointEntity : velocityControlledJoints) {

        ignition::math::PID& pid = utils::getExistingComponentData<
            ignition::gazebo::components::JointPID>(&ecm, jointEntity);

        std::string& jointName =
            utils::getExistingComponentData<ignition::gazebo::components::Name>(
                &ecm, jointEntity);

        auto joint = pImpl->model->getJoint(jointName);
        auto jointGazebo = std::static_pointer_cast<Joint>(joint);

        const std::vector<double>& velocity = joint->jointVelocity();

        const std::vector<double>& velocityTarget =
            utils::getExistingComponentData<
                ignition::gazebo::components::JointVelocityTarget>(&ecm,
                                                                   jointEntity);

        if (!Impl::runPIDController(*jointGazebo,
                                    computeNewForce,
                                    pid,
                                    info.dt,
                                    velocityTarget,
                                    velocity)) {
            sError << "Failed to run PID controller of joint " << joint->name()
                   << " [" << jointEntity << "]" << std::endl;
        }
    }

    // Set the velocity command for Revolute and Prismatic joints controlled in
    // VelocityDirect. This control mode computes and applies the right force
    // to get the desired velocity at the next step. It can be thought as an
    // ideal velocity PID.
    for (const auto jointEntity : velocityFollowerDartControlledJoints) {

        const std::string& jointName =
            utils::getExistingComponentData<ignition::gazebo::components::Name>(
                &ecm, jointEntity);

        const auto joint = pImpl->model->getJoint(jointName);

        const std::vector<double>& velocityTarget =
            utils::getExistingComponentData<
                ignition::gazebo::components::JointVelocityTarget>(&ecm,
                                                                   jointEntity);

        auto& jointVelocityCmd = utils::getComponentData< //
            ignition::gazebo::components::JointVelocityCmd>(&ecm, jointEntity);

        if (jointVelocityCmd.size() != joint->dofs()) {
            assert(jointVelocityCmd.size() == 0);
            jointVelocityCmd = std::vector<double>(joint->dofs(), 0.0);
        }

        // Set the target
        jointVelocityCmd = velocityTarget;
    }
}

bool JointController::Impl::runPIDController(
    scenario::gazebo::Joint& joint,
    const bool computeNewForce,
    ignition::math::PID& pid,
    const std::chrono::steady_clock::duration& dt,
    const std::vector<double>& reference,
    const std::vector<double>& current)
{
    switch (joint.type()) {

        case core::JointType::Revolute:
        case core::JointType::Prismatic: {

            double force;

            if (computeNewForce) {
                assert(current.size() == 1);
                assert(reference.size() == 1);

                double error = current[0] - reference[0];
                force = pid.Update(error, dt);
            }
            else {
                force = pid.Cmd();
            }

            if (!joint.setGeneralizedForceTarget(force)) {
                sError << "Failed to set force of joint " << joint.name()
                       << std::endl;
                return false;
            }
            return true;
        }
        case core::JointType::Fixed:
        case core::JointType::Ball:
        case core::JointType::Invalid:
            sWarning << "Type of joint '" << joint.name() << " not supported"
                     << std::endl;
            return true;
    }

    return false;
};

IGNITION_ADD_PLUGIN(
    scenario::plugins::gazebo::JointController,
    scenario::plugins::gazebo::JointController::System,
    scenario::plugins::gazebo::JointController::ISystemConfigure,
    scenario::plugins::gazebo::JointController::ISystemPreUpdate)
