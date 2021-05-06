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

#include "ControllerRunner.h"
#include "ControllersFactory.h"
#include "scenario/controllers/Controller.h"
#include "scenario/controllers/References.h"
#include "scenario/gazebo/Link.h"
#include "scenario/gazebo/Log.h"
#include "scenario/gazebo/Model.h"
#include "scenario/gazebo/components/BasePoseTarget.h"
#include "scenario/gazebo/components/BaseWorldAccelerationTarget.h"
#include "scenario/gazebo/components/BaseWorldVelocityTarget.h"
#include "scenario/gazebo/exceptions.h"
#include "scenario/gazebo/helpers.h"

#include <ignition/math/Helpers.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/plugin/Register.hh>
#include <sdf/Element.hh>

#include <array>
#include <cassert>
#include <chrono>
#include <limits>
#include <ratio>
#include <string>
#include <vector>

using namespace scenario::gazebo;
using namespace scenario::plugins::gazebo;

class ControllerRunner::Impl
{
public:
    bool referencesHaveBeenSet = false;

    std::shared_ptr<Model> model;
    ignition::gazebo::Entity modelEntity;
    std::chrono::steady_clock::duration prevUpdateTime{0};

    std::shared_ptr<controllers::Controller> controller;

    controllers::BaseReferences baseReferences;
    controllers::JointReferences jointReferences;

    struct
    {
        controllers::SetBaseReferences* base = nullptr;
        controllers::UseScenarioModel* useModel = nullptr;
        controllers::SetJointReferences* joints = nullptr;
    } controllerInterfaces;

    bool
    updateAllSupportedReferences(ignition::gazebo::EntityComponentManager& ecm);

    bool
    updateBaseReferencesfromECM(ignition::gazebo::EntityComponentManager& ecm);

    bool
    updateJointReferencesfromECM(ignition::gazebo::EntityComponentManager& ecm);

    void printControllerContext(
        const std::shared_ptr<const sdf::Element> context) const;
};

ControllerRunner::ControllerRunner()
    : System()
    , pImpl{std::make_unique<Impl>()}
{}

// NOTE: we should terminate the controller here, but plugins are not
//       unloaded when the model is removed.
//       All model plugins are deleted when the simulator is destroyed,
//       and there's no more ECM -> we would get segfault.
ControllerRunner::~ControllerRunner() = default;

void ControllerRunner::Configure(const ignition::gazebo::Entity& entity,
                                 const std::shared_ptr<const sdf::Element>& sdf,
                                 ignition::gazebo::EntityComponentManager& ecm,
                                 ignition::gazebo::EventManager& eventMgr)
{

    // Store the model entity
    pImpl->modelEntity = entity;

    // Create a model that will be given to the controller
    pImpl->model = std::make_shared<Model>();

    if (!pImpl->model->initialize(entity, &ecm, &eventMgr)) {
        sError << "Failed to initialize model for controller" << std::endl;
        return;
    }

    if (!pImpl->model->valid()) {
        sError << "Failed to create a model from Entity [" << entity << "]"
               << std::endl;
        return;
    }

    if (sdf->GetName() != "plugin") {
        sError << "Received context does not contain the <plugin> element"
               << std::endl;
        return;
    }

    // This is the <plugin> element (with extra options stored in its children)
    sdf::ElementPtr pluginElement = sdf->Clone();

    // Initialize the controller context
    sdf::ElementPtr controllerContext;

    // Check if it contains extra options stored in a <controller> child
    if (pluginElement->HasElement("controller")) {
        // Store the <controller> element
        controllerContext = pluginElement->GetElement("controller");

        if (utils::verboseFromEnvironment()) {
            pImpl->printControllerContext(pluginElement);
        }
    }

    pImpl->controller =
        ControllersFactory::Instance().get(controllerContext, pImpl->model);

    if (!pImpl->controller) {
        sError << "Failed to find controller in the factory" << std::endl;
        return;
    }

    if (!pImpl->controller->initialize()) {
        sError << "Failed to initialize the controller" << std::endl;
        pImpl->controller = nullptr;
        return;
    }

    pImpl->controllerInterfaces.useModel = dynamic_cast< //
        controllers::UseScenarioModel*>(pImpl->controller.get());

    pImpl->controllerInterfaces.base = dynamic_cast< //
        controllers::SetBaseReferences*>(pImpl->controller.get());

    pImpl->controllerInterfaces.joints = dynamic_cast< //
        controllers::SetJointReferences*>(pImpl->controller.get());

    // Controller classes could inherit from various interfaces that specify the
    // accepted references. This design allows developing generic controllers.
    // Here we check if the controller inherits from the supported interfaces.
    if (!(pImpl->controllerInterfaces.base
          || pImpl->controllerInterfaces.joints)) {
        sWarning << "Failed to find any of the supported interfaces to set "
                 << "controller references" << std::endl;
        return;
    }

    sDebug << "Controller successfully initialized" << std::endl;
}

void ControllerRunner::PreUpdate(const ignition::gazebo::UpdateInfo& info,
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

    if (!pImpl->controller) {
        sError << "The controller was not initialized successfully"
               << std::endl;
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

        // Enable using the controller to compute the new force
        computeNewForce = true;
    }
    else {
        // Disable the controller and send the same force reference as last
        // update
        computeNewForce = false;
    }

    // Get and set the new references
    if (computeNewForce) {

        try {
            if (!pImpl->updateAllSupportedReferences(ecm)) {
                sError << "Failed to update supported references" << std::endl;
                return;
            }
        }
        catch (const exceptions::ComponentNotFound& e) {
            sDebug << "Controller references not yet available" << std::endl;
            sDebug << e.what();
            sWarning << "[t="
                     << utils::steadyClockDurationToDouble(info.simTime)
                     << "] The controller is not stepping" << std::endl;
            return;
        }

        if (pImpl->controllerInterfaces.useModel
            && !pImpl->controllerInterfaces.useModel->updateStateFromModel()) {
            sError << "Failed to update controller state from internal model"
                   << std::endl;
            return;
        }

        // The controller is stepped only when the references have been set
        // at least once
        pImpl->referencesHaveBeenSet = true;
    }

    // Step the controller
    if (pImpl->referencesHaveBeenSet && !pImpl->controller->step(info.dt)) {
        sError << "Failed to step the controller" << std::endl;
        return;
    }
}

bool ControllerRunner::Impl::updateAllSupportedReferences(
    ignition::gazebo::EntityComponentManager& ecm)
{
    bool ok = true;

    if (controllerInterfaces.base) {
        if (!updateBaseReferencesfromECM(ecm)) {
            sError << "Failed to update base references" << std::endl;
            ok = false;
        }
        else {
            if (!controllerInterfaces.base->setBaseReferences(baseReferences)) {
                sError << "Failed to set base references" << std::endl;
                ok = false;
            }
        }
    }

    if (controllerInterfaces.joints) {
        if (!updateJointReferencesfromECM(ecm)) {
            sError << "Failed to update joint references" << std::endl;
            ok = false;
        }
        else {
            if (!controllerInterfaces.joints->setJointReferences(
                    jointReferences)) {
                sError << "Failed to set joint references" << std::endl;
                ok = false;
            }
        }
    }

    return ok;
}

bool ControllerRunner::Impl::updateBaseReferencesfromECM(
    ignition::gazebo::EntityComponentManager& ecm)
{
    assert(controllerInterfaces.base);
    using namespace ignition::math;
    using namespace ignition::gazebo;

    // =========
    // Base Pose
    // =========

    Pose3d& basePoseTarget = utils::getExistingComponentData< //
        components::BasePoseTarget>(&ecm, modelEntity);

    core::Pose basePose = utils::fromIgnitionPose(basePoseTarget);
    baseReferences.position = basePose.position;
    baseReferences.orientation = basePose.orientation;

    // =============
    // Base Velocity
    // =============

    Vector3d baseLinearVelocityTarget = utils::getExistingComponentData< //
        components::BaseWorldLinearVelocityTarget>(&ecm, modelEntity);

    Vector3d baseAngularVelocityTarget = utils::getExistingComponentData< //

        components::BaseWorldAngularVelocityTarget>(&ecm, modelEntity);

    baseReferences.linearVelocity =
        utils::fromIgnitionVector(baseLinearVelocityTarget);
    baseReferences.angularVelocity =
        utils::fromIgnitionVector(baseAngularVelocityTarget);

    // =================
    // Base Acceleration
    // =================

    Vector3d baseLinearAccelerationTarget = utils::getExistingComponentData< //
        components::BaseWorldLinearAccelerationTarget>(&ecm, modelEntity);

    Vector3d baseAngularAccelerationTarget = utils::getExistingComponentData< //
        components::BaseWorldAngularAccelerationTarget>(&ecm, modelEntity);

    baseReferences.linearAcceleration =
        utils::fromIgnitionVector(baseLinearAccelerationTarget);
    baseReferences.angularAcceleration =
        utils::fromIgnitionVector(baseAngularAccelerationTarget);

    return true;
}

bool ControllerRunner::Impl::updateJointReferencesfromECM(
    ignition::gazebo::EntityComponentManager& /*ecm*/)
{
    assert(controllerInterfaces.joints);

    auto& controlledJoints = controllerInterfaces.joints->controlledJoints();

    jointReferences.position = model->jointPositionTargets(controlledJoints);
    jointReferences.velocity = model->jointVelocityTargets(controlledJoints);
    jointReferences.acceleration =
        model->jointAccelerationTargets(controlledJoints);

    return true;
}

void ControllerRunner::Impl::printControllerContext(
    const std::shared_ptr<const sdf::Element> context) const
{
    sDebug << "SDF elements received by the controller:" << std::endl;
    std::cout << context->ToString("") << std::endl;
}

IGNITION_ADD_PLUGIN(
    scenario::plugins::gazebo::ControllerRunner,
    scenario::plugins::gazebo::ControllerRunner::System,
    scenario::plugins::gazebo::ControllerRunner::ISystemConfigure,
    scenario::plugins::gazebo::ControllerRunner::ISystemPreUpdate)
