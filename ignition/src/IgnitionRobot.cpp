/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "gympp/gazebo/IgnitionRobot.h"
#include "gympp/Log.h"
#include "gympp/gazebo/RobotSingleton.h"

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointPositionCmd.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/gazebo/components/JointType.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/JointVelocityReset.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/math/PID.hh>
#include <sdf/Joint.hh>

#include <cassert>
#include <chrono>
#include <map>
#include <ostream>
#include <string>

using namespace gympp::gazebo;

using LinkName = std::string;
using JointName = std::string;
using LinkEntity = ignition::gazebo::Entity;
using JointEntity = ignition::gazebo::Entity;

const ignition::math::PID DefaultPID(1, 0.1, 0.01, 1, -1, 10000, -10000);

enum class ControlType
{
    Position,
    Velocity
};

struct PIDData
{
    ControlType type;
    ignition::math::PID pid;
};

struct Buffers
{
    struct
    {
        gympp::Robot::JointPositions positions;
        gympp::Robot::JointVelocities velocities;
        std::map<JointName, double> references;
        std::map<JointName, double> appliedForces;
        std::map<JointName, PIDData> pidData;
    } joints;
};

class IgnitionRobot::Impl
{
public:
    std::string name;

    ignition::gazebo::EntityComponentManager* ecm = nullptr;
    ignition::gazebo::Model model;

    std::chrono::duration<double> dt;
    std::chrono::duration<double> prevUpdateTime = std::chrono::duration<double>(0.0);

    Buffers buffers;

    std::map<LinkName, LinkEntity> links;
    std::map<JointName, JointEntity> joints;

    inline bool jointExists(const JointName& jointName) const
    {
        return joints.find(jointName) != joints.end();
    }

    inline bool pidExists(const JointName& jointName) const
    {
        return buffers.joints.pidData.find(jointName) != buffers.joints.pidData.end();
    }

    inline bool linkExists(const LinkName& linkName) const
    {
        return links.find(linkName) != links.end();
    }

    JointEntity getLinkEntity(const LinkName& linkName);
    JointEntity getJointEntity(const JointName& jointName);

    template <typename ComponentType>
    ComponentType& getOrCreateComponent(const ignition::gazebo::Entity entity);
};

LinkEntity IgnitionRobot::Impl::getLinkEntity(const LinkName& linkName)
{
    if (!ecm) {
        gymppError << "Failed to get the entity-component mananger" << std::endl;
        return ignition::gazebo::kNullEntity;
    }

    if (!linkExists(linkName)) {
        gymppError << "Link '" << linkName << "' not found" << std::endl;
        return ignition::gazebo::kNullEntity;
    }

    if (links[linkName] == ignition::gazebo::kNullEntity) {
        gymppError << "The entity associated to link '" << linkName
                   << "' has not been properly stored" << std::endl;
        return ignition::gazebo::kNullEntity;
    }

    // Return the link entity
    return links[linkName];
}

JointEntity IgnitionRobot::Impl::getJointEntity(const JointName& jointName)
{
    if (!ecm) {
        gymppError << "Failed to get the entity-component mananger" << std::endl;
        return ignition::gazebo::kNullEntity;
    }

    if (!jointExists(jointName)) {
        gymppError << "Joint '" << jointName << "' not found" << std::endl;
        return ignition::gazebo::kNullEntity;
    }

    if (joints[jointName] == ignition::gazebo::kNullEntity) {
        gymppError << "The entity associated to joint '" << jointName
                   << "' has not been properly stored" << std::endl;
        return ignition::gazebo::kNullEntity;
    }

    // Return the joint entity
    return joints[jointName];
}

template <typename ComponentTypeT>
ComponentTypeT& IgnitionRobot::Impl::getOrCreateComponent(const ignition::gazebo::Entity entity)
{
    auto* component = ecm->Component<ComponentTypeT>(entity);

    if (!component) {
        ecm->CreateComponent(entity, ComponentTypeT());
        component = ecm->Component<ComponentTypeT>(entity);
    }

    return *component;
}

// ==============
// IGNITION ROBOT
// ==============

IgnitionRobot::IgnitionRobot()
    : pImpl{new Impl(), [](Impl* impl) { delete impl; }}
{}

IgnitionRobot::~IgnitionRobot() = default;

bool IgnitionRobot::configureECM(const ignition::gazebo::Entity& entity,
                                 const std::shared_ptr<const sdf::Element>& sdf,
                                 ignition::gazebo::EntityComponentManager& ecm)
{
    // Store the address of the entity-component manager
    pImpl->ecm = &ecm;

    // Create the model
    pImpl->model = ignition::gazebo::Model(entity);

    // Check that the model is valid
    if (!pImpl->model.Valid(ecm)) {
        // Create a label to identify the sdf element
        std::string sdfElementString = "<" + sdf->GetName();
        for (size_t i = 0; i < sdf->GetAttributeCount(); ++i) {
            sdfElementString += " attr='" + sdf->GetAttribute(i)->GetAsString() + "'";
        }
        sdfElementString += ">";

        gymppError << "The model associated to sdf element '" << sdfElementString << "is not valid"
                   << std::endl;
        return false;
    }

    gymppDebug << "Processing model '" << pImpl->model.Name(ecm) << "'" << std::endl;

    // Get all the model joints
    ecm.Each<ignition::gazebo::components::Joint,
             ignition::gazebo::components::Name,
             ignition::gazebo::components::JointType,
             ignition::gazebo::components::ParentEntity>(
        [&](const ignition::gazebo::Entity& entity,
            ignition::gazebo::components::Joint* /*joint*/,
            ignition::gazebo::components::Name* name,
            ignition::gazebo::components::JointType* type,
            ignition::gazebo::components::ParentEntity* parentEntityComponent) -> bool {
            // Skip all the joints not belonging to this model
            if (parentEntityComponent->Data() != pImpl->model.Entity()) {
                return true;
            }

            gymppDebug << "  Found joint: " << pImpl->model.Name(ecm) << "::" << name->Data()
                       << " [" << entity << "]" << std::endl;

            // Find the entity of the joint in the ecm
            auto jointEntity = pImpl->model.JointByName(ecm, name->Data());
            if (jointEntity == ignition::gazebo::kNullEntity) {
                gymppError << "Failed to find entity for joint '" << pImpl->model.Name(ecm)
                           << "::" << name->Data() << "'" << std::endl;
                return false;
            }

            // Ignore fixed joints
            if (type->Data() == sdf::JointType::FIXED) {
                gymppDebug << "  Skipping fixed joint '" << pImpl->model.Name(ecm)
                           << "::" << name->Data() << "'" << std::endl;
                return true;
            }

            // Create the joint position and velocity components.
            // In this way this data is stored in these components after the physics step.
            ecm.CreateComponent(entity, ignition::gazebo::components::JointPosition());
            ecm.CreateComponent(entity, ignition::gazebo::components::JointVelocity());

            // Store the joint entity
            pImpl->joints[name->Data()] = jointEntity;

            return true;
        });

    // Get all the model links
    ecm.Each<ignition::gazebo::components::Link,
             ignition::gazebo::components::Name,
             ignition::gazebo::components::Pose,
             ignition::gazebo::components::ParentEntity>(
        [&](const ignition::gazebo::Entity& entity,
            ignition::gazebo::components::Link* /*link*/,
            ignition::gazebo::components::Name* name,
            ignition::gazebo::components::Pose* /*pose*/,
            ignition::gazebo::components::ParentEntity* parentEntityComponent) -> bool {
            // Skip all the joints not belonging to this model
            if (parentEntityComponent->Data() != pImpl->model.Entity()) {
                return true;
            }

            gymppDebug << "  Found link: " << pImpl->model.Name(ecm) << "::" << name->Data() << " ["
                       << entity << "]" << std::endl;

            // TODO: there is an extra link 'link', I suspect related to the <include><pose>
            if (name->Data() == "link") {
                gymppDebug << "  Skipping dummy link 'link'" << std::endl;
                return true;
            }

            // Find the entity of the link in the ecm
            auto linkEntity = pImpl->model.LinkByName(ecm, name->Data());
            if (linkEntity == ignition::gazebo::kNullEntity) {
                gymppError << "Failed to find entity for link '" << pImpl->model.Name(ecm)
                           << "::" << name->Data() << "'" << std::endl;
                return false;
            }

            // Store the link entity
            pImpl->links[name->Data()] = linkEntity;
            return true;
        });

    // Check that the created object is valid
    if (!valid()) {
        gymppError << "The IgnitionRobot object for model '" << pImpl->model.Name(ecm)
                   << "' is not valid" << std::endl;
        return false;
    }

    // Store the name of the robot
    pImpl->name = pImpl->model.Name(ecm);

    if (pImpl->name.empty()) {
        gymppError << "The model entity has an empty name component" << std::endl;
        return false;
    }

    // Initialize the buffers
    pImpl->buffers.joints.positions.resize(pImpl->joints.size());
    pImpl->buffers.joints.velocities.resize(pImpl->joints.size());

    return true;
}

bool IgnitionRobot::valid() const
{
    // TODO: find the proper logic to check if this object is valid

    if (!pImpl->ecm) {
        return false;
    }

    if (pImpl->joints.size() == 0) {
        return false;
    }

    if (pImpl->links.size() == 0) {
        return false;
    }

    return true;
}

// ===========
// GET METHODS
// ===========

gympp::Robot::RobotName IgnitionRobot::name() const
{
    return pImpl->name;
}

gympp::Robot::JointNames IgnitionRobot::jointNames() const
{
    JointNames names;
    names.reserve(pImpl->joints.size());

    for (const auto& [jointName, _] : pImpl->joints) {
        names.push_back(jointName);
    }

    return names;
}

double IgnitionRobot::jointPosition(const gympp::Robot::JointName& jointName) const
{
    JointEntity jointEntity = pImpl->getJointEntity(jointName);
    assert(jointEntity != ignition::gazebo::kNullEntity);

    // Get the joint position component
    auto jointPositionComponent =
        pImpl->ecm->Component<ignition::gazebo::components::JointPosition>(jointEntity);

    if (!jointPositionComponent) {
        gymppError << "Position for joint '" << jointName << "' not found in the ecm" << std::endl;
        return {};
    }

    if (jointPositionComponent->Data().size() <= 0) {
        gymppWarning << "The joint position component exists but it does not have yet any data"
                     << std::endl;
        return {};
    }

    assert(jointPositionComponent->Data().size() == 1);
    return jointPositionComponent->Data()[0];
}

double IgnitionRobot::jointVelocity(const gympp::Robot::JointName& jointName) const
{
    JointEntity jointEntity = pImpl->getJointEntity(jointName);
    assert(jointEntity != ignition::gazebo::kNullEntity);

    // Get the joint velocity component
    auto jointVelocityComponent =
        pImpl->ecm->Component<ignition::gazebo::components::JointVelocity>(jointEntity);

    if (!jointVelocityComponent) {
        gymppError << "Velocity for joint '" << jointName << "' not found in the ecm" << std::endl;
        return {};
    }

    if (jointVelocityComponent->Data().size() <= 0) {
        gymppWarning << "The joint velocity component exists but it does not have yet any data"
                     << std::endl;
        return {};
    }

    assert(jointVelocityComponent->Data().size() == 1);
    return jointVelocityComponent->Data()[0];
}

gympp::Robot::JointPositions IgnitionRobot::jointPositions() const
{
    size_t i = 0;
    for (const auto& [jointName, _] : pImpl->joints) {
        pImpl->buffers.joints.positions[i++] = jointPosition(jointName);
    }

    return pImpl->buffers.joints.positions;
}

gympp::Robot::JointVelocities IgnitionRobot::jointVelocities() const
{
    size_t i = 0;
    for (const auto& [jointName, _] : pImpl->joints) {
        pImpl->buffers.joints.velocities[i++] = jointVelocity(jointName);
    }

    return pImpl->buffers.joints.velocities;
}

gympp::Robot::StepSize IgnitionRobot::dt() const
{
    return pImpl->dt;
}

IgnitionRobot::PID IgnitionRobot::jointPID(const gympp::Robot::JointName& jointName) const
{
    assert(pImpl->jointExists(jointName));
    assert(pImpl->pidExists(jointName));

    auto& pid = pImpl->buffers.joints.pidData[jointName].pid;
    return PID(pid.PGain(), pid.IGain(), pid.DGain());
}

bool IgnitionRobot::setdt(const gympp::Robot::StepSize& stepSize)
{
    pImpl->dt = stepSize;
    return true;
}

// ===========
// SET METHODS
// ===========

bool IgnitionRobot::setJointForce(const gympp::Robot::JointName& jointName, const double jointForce)
{
    JointEntity jointEntity = pImpl->getJointEntity(jointName);
    if (jointEntity == ignition::gazebo::kNullEntity) {
        return false;
    }

    // Get the JointForce component
    auto& forceComponent =
        pImpl->getOrCreateComponent<ignition::gazebo::components::JointForceCmd>(jointEntity);

    // Set the joint force
    forceComponent = ignition::gazebo::components::JointForceCmd({jointForce});

    return true;
}

bool IgnitionRobot::setJointPositionTarget(const gympp::Robot::JointName& jointName,
                                           const double jointPositionReference)
{
    // The controller period must have been set in order to set references
    if (pImpl->dt == std::chrono::duration<double>(0.0)) {
        gymppError << "The update time of the controlled was not set" << std::endl;
        return false;
    }

    JointEntity jointEntity = pImpl->getJointEntity(jointName);
    if (jointEntity == ignition::gazebo::kNullEntity) {
        return false;
    }

    // Create the PID if it does not exist
    if (!pImpl->pidExists(jointName)) {
        pImpl->buffers.joints.pidData[jointName] = {ControlType::Position, DefaultPID};
    }

    // Check the control type and switch to position control if the joint was controlled differently
    if (pImpl->buffers.joints.pidData[jointName].type != ControlType::Position) {
        gymppDebug << "Switching joint '" << jointName << "' to Position control" << std::endl;
        pImpl->buffers.joints.pidData[jointName].type = ControlType::Position;

        // Reset the PID
        assert(pImpl->pidExists(jointName));
        pImpl->buffers.joints.pidData[jointName].pid.Reset();
    }

    // Update the joint reference
    pImpl->buffers.joints.references[jointName] = jointPositionReference;
    return true;
}

bool IgnitionRobot::setJointVelocityTarget(const gympp::Robot::JointName& jointName,
                                           const double jointVelocityReference)
{
    // The controller period must have been set in order to set references
    if (pImpl->dt == std::chrono::duration<double>(0.0)) {
        gymppError << "The update time of the controlled was not set" << std::endl;
        return false;
    }

    JointEntity jointEntity = pImpl->getJointEntity(jointName);
    if (jointEntity == ignition::gazebo::kNullEntity) {
        return false;
    }

    // Create the PID if it does not exist
    if (!pImpl->pidExists(jointName)) {
        pImpl->buffers.joints.pidData[jointName] = {ControlType::Velocity, DefaultPID};
    }

    // Check the control type and switch to velocity control if the joint was controlled differently
    if (pImpl->buffers.joints.pidData[jointName].type != ControlType::Velocity) {
        gymppDebug << "Switching joint '" << jointName << "' to Velocity control" << std::endl;
        pImpl->buffers.joints.pidData[jointName].type = ControlType::Velocity;

        // Reset the PID
        assert(pImpl->pidExists(jointName));
        pImpl->buffers.joints.pidData[jointName].pid.Reset();
    }

    // Update the joint reference
    pImpl->buffers.joints.references[jointName] = jointVelocityReference;

    return true;
}

bool IgnitionRobot::setJointPosition(const gympp::Robot::JointName& jointName,
                                     const double jointPosition)
{
    JointEntity jointEntity = pImpl->getJointEntity(jointName);
    if (jointEntity == ignition::gazebo::kNullEntity) {
        return false;
    }

    // Reset the position
    auto& jointPosResetComponent =
        pImpl->getOrCreateComponent<ignition::gazebo::components::JointPositionReset>(jointEntity);

    jointPosResetComponent = ignition::gazebo::components::JointPositionReset({jointPosition});

    // Store the new position in the ECM
    auto& jointPosComponent =
        pImpl->getOrCreateComponent<ignition::gazebo::components::JointPosition>(jointEntity);

    jointPosComponent = ignition::gazebo::components::JointPosition({jointPosition});

    return true;
}

bool IgnitionRobot::setJointVelocity(const gympp::Robot::JointName& jointName,
                                     const double jointVelocity)
{
    JointEntity jointEntity = pImpl->getJointEntity(jointName);
    if (jointEntity == ignition::gazebo::kNullEntity) {
        return false;
    }

    // Reset the velocity
    auto& jointVelResetComponent =
        pImpl->getOrCreateComponent<ignition::gazebo::components::JointVelocityReset>(jointEntity);

    jointVelResetComponent = ignition::gazebo::components::JointVelocityReset({jointVelocity});

    // Store the new velocity in the ECM
    auto& jointVelComponent =
        pImpl->getOrCreateComponent<ignition::gazebo::components::JointVelocity>(jointEntity);

    jointVelComponent = ignition::gazebo::components::JointVelocity({jointVelocity});

    return true;
}

bool IgnitionRobot::setJointPID(const gympp::Robot::JointName& jointName, const PID& pid)
{
    JointEntity jointEntity = pImpl->getJointEntity(jointName);
    if (jointEntity == ignition::gazebo::kNullEntity) {
        return false;
    }

    if (!pImpl->pidExists(jointName)) {
        gymppDebug << "Creating new PID for joint " << jointName << std::endl;
        pImpl->buffers.joints.pidData[jointName] = {ControlType::Position, DefaultPID};
    }
    else {
        pImpl->buffers.joints.pidData[jointName].pid.Reset();
    }

    // Update the gains. The other PID parameters do not change.
    pImpl->buffers.joints.pidData[jointName].pid.SetPGain(pid.p);
    pImpl->buffers.joints.pidData[jointName].pid.SetIGain(pid.i);
    pImpl->buffers.joints.pidData[jointName].pid.SetDGain(pid.d);

    return true;
}

bool IgnitionRobot::resetJoint(const gympp::Robot::JointName& jointName,
                               const double jointPosition,
                               const double jointVelocity)
{
    JointEntity jointEntity = pImpl->getJointEntity(jointName);
    if (jointEntity == ignition::gazebo::kNullEntity) {
        return false;
    }

    // Reset the joint position
    if (!setJointPosition(jointName, jointPosition)) {
        gymppError << "Failed to reset the joint position of joint '" << jointName << "'"
                   << std::endl;
        return false;
    }

    // Reset the joint velocity
    if (!setJointVelocity(jointName, jointVelocity)) {
        gymppError << "Failed to reset the joint velocity of joint '" << jointName << "'"
                   << std::endl;
        return false;
    }

    // Reset the PID
    if (pImpl->pidExists(jointName)) {
        pImpl->buffers.joints.pidData[jointName].pid.Reset();
        pImpl->buffers.joints.pidData[jointName].type = ControlType::Position;
    }

    // Clean the joint controlling storage
    pImpl->buffers.joints.references.erase(jointName);
    return true;
}

bool IgnitionRobot::update(const std::chrono::duration<double> time)
{
    // Return if there are no references to actuate
    if (pImpl->buffers.joints.references.empty()) {
        return true;
    }

    // The controller period must have been set in order to use PIDs
    if (pImpl->dt == std::chrono::duration<double>(0.0)) {
        gymppError << "The update time of the controlled was not set" << std::endl;
        return false;
    }

    // Update the controller only if enough time is passed
    std::chrono::duration<double> stepTime = time - pImpl->prevUpdateTime;

    // Handle first iteration
    if (pImpl->prevUpdateTime == std::chrono::duration<double>(0.0)) {
        stepTime = pImpl->dt;
    }

    // If enough time is passed, store the time of this actuation step. In this case the state of
    // the robot is read and new force references are computed and actuated.
    // Otherwise, the same force of the last step is actuated.
    bool updateCurrentState;

    if (stepTime >= pImpl->dt) {
        // Store the current update time
        pImpl->prevUpdateTime = time;

        // Enable using the PID to compute the new force
        updateCurrentState = true;
    }
    else {
        // Disable the PID and send the same force reference as last update
        updateCurrentState = false;
    }

    // Actuate the references
    // The references can be either position or velocity references
    for (auto& [jointName, reference] : pImpl->buffers.joints.references) {
        assert(pImpl->pidExists(jointName));

        // Use the PID the compute the new force
        if (updateCurrentState) {
            double force = 0;

            // Get the PID
            auto& pid = pImpl->buffers.joints.pidData[jointName].pid;

            // Use the PID to get the reference
            switch (pImpl->buffers.joints.pidData[jointName].type) {
                case ControlType::Position:
                    force = pid.Update(jointPosition(jointName) - reference, stepTime);
                    break;
                case ControlType::Velocity:
                    force = pid.Update(jointVelocity(jointName) - reference, stepTime);
                    break;
            }

            // Store the force
            pImpl->buffers.joints.appliedForces[jointName] = force;
        }

        // Break if there is no force to actuate for this joint
        if (pImpl->buffers.joints.appliedForces.find(jointName)
            == pImpl->buffers.joints.appliedForces.end()) {
            break;
        }

        // Get the force
        auto force = pImpl->buffers.joints.appliedForces[jointName];

        // Actuate the force
        if (!setJointForce(jointName, force)) {
            gymppError << "Failed to set force to joint '" << jointName << "'" << std::endl;
            return false;
        }
    }

    return true;
}
