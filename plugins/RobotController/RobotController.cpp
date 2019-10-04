/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "RobotController.h"
#include "gympp/Log.h"
#include "gympp/Robot.h"
#include "gympp/gazebo/RobotSingleton.h"

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/plugin/Register.hh>

#include <cassert>
#include <chrono>
#include <ostream>
#include <string>

using namespace gympp::gazebo;
using namespace gympp::plugins;

// ====
// IMPL
// ====

class RobotController::Impl
{
public:
    std::string modelName;
    bool warningReported = false;
    std::shared_ptr<gympp::Robot> robot = nullptr;
    static gympp::RobotPtr getRobotPtr(const std::string& robotName);
};

gympp::RobotPtr RobotController::Impl::getRobotPtr(const std::string& robotName)
{
    // Get the robot interface
    auto robotPtr = RobotSingleton::get().getRobot(robotName).lock();

    // Check that is not a nullptr
    if (!robotPtr) {
        gymppWarning << "Failed to get the robot '" << robotName << "' from the singleton"
                     << std::endl;
        return {};
    }

    if (!robotPtr->valid()) {
        gymppError << "The robot interface is not valid" << std::endl;
        return {};
    }

    return robotPtr;
}

// ===============
// ROBOTCONTROLLER
// ===============

RobotController::RobotController()
    : System()
    , pImpl{new Impl(), [](Impl* impl) { delete impl; }}
{}

RobotController::~RobotController()
{
    gymppDebug << "Destroying the RobotController of the robot '" << pImpl->modelName << "'"
               << std::endl;
};

void RobotController::Configure(const ignition::gazebo::Entity& entity,
                                const std::shared_ptr<const sdf::Element>& /*sdf*/,
                                ignition::gazebo::EntityComponentManager& ecm,
                                ignition::gazebo::EventManager& /*eventMgr*/)
{
    // Create a model and check its validity
    auto model = ignition::gazebo::Model(entity);
    if (!model.Valid(ecm)) {
        gymppError << "The entity of the parent model of the gympp plugin is not valid"
                   << std::endl;
        assert(false);
        return;
    }

    // Get the model name and ask the robot interface
    pImpl->modelName = model.Name(ecm);
}

void RobotController::PreUpdate(const ignition::gazebo::UpdateInfo& info,
                                ignition::gazebo::EntityComponentManager& ecm)
{
    if (info.paused) {
        return;
    }

    // TODO: after model deletion, plugins are not unloaded from the simulation. This means that the
    //       RobotController keeps running also if its parent model does not exist anymore.
    //       We check here the existing of the model in the ECM before proceeding.
    auto modelEntity = ecm.EntityByComponents(ignition::gazebo::components::Model(),
                                              ignition::gazebo::components::Name(pImpl->modelName));
    if (modelEntity == ignition::gazebo::kNullEntity) {
        if (!pImpl->warningReported) {
            pImpl->warningReported = true;
            gymppWarning << "The model " << pImpl->modelName
                         << " does not exist anymore. Its RobotController will be no-op."
                         << std::endl;
        }
        return;
    }

    // TODO: The model entity removal is a -request- of removal. Depending how it is called, the
    //       simulator can still finish its simulation step with the entity in pending removal
    //       state. Currently the ECM has a method IsMarkedForRemoval but it is private.

    // During the process of model creation, this plugin is loaded right after the creation of
    // all the entities related to links, joints, etc, and before storing the robot interface in
    // the singleton. This means that asking for the robot interface during the Configure step
    // is still early. We do it lazily here at the first PreUpdate call.
    if (!pImpl->robot) {
        pImpl->robot = pImpl->getRobotPtr(pImpl->modelName);

        if (!pImpl->robot) {
            gymppWarning << "Failed to get the robot interface. Maybe the plugin is going to "
                            "be unloaded?"
                         << std::endl;
            return;
        }
        assert(pImpl->robot);
    }

    // Update the PID controllers of the robot
    if (!pImpl->robot->update(info.simTime)) {
        assert(false);
        gymppError << "Failed to update the robot controller" << std::endl;
        return;
    }
}

IGNITION_ADD_PLUGIN(gympp::plugins::RobotController,
                    gympp::plugins::RobotController::System,
                    gympp::plugins::RobotController::ISystemConfigure,
                    gympp::plugins::RobotController::ISystemPreUpdate)
