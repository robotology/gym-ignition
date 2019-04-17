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
#include <ignition/gazebo/components/JointForce.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/math/PID.hh>

#include <optional>
#include <unordered_map>

using namespace gympp::gazebo;

using LinkName = std::string;
using JointName = std::string;
using LinkEntity = ignition::gazebo::Entity;
using JointEntity = ignition::gazebo::Entity;

const ignition::math::PID DefaultPID(1, 0.1, 0.1, 1, -1, 1000, -1000);

class IgnitionRobot::Impl
{
public:
    ignition::gazebo::EntityComponentManager* ecm = nullptr;
    ignition::gazebo::Model model;
    std::chrono::duration<double> dt;

    std::unordered_map<LinkName, LinkEntity> links;
    std::unordered_map<JointName, JointEntity> joints;
    std::unordered_map<JointName, ignition::math::PID> pids;

    inline bool jointExists(const JointName& jointName) const
    {
        return joints.find(jointName) != joints.end();
    }

    inline bool pidExists(const JointName& jointName) const
    {
        return pids.find(jointName) != pids.end();
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

    // Return the link entity
    return joints[jointName];
}

template <typename ComponentTypeT>
ComponentTypeT& IgnitionRobot::Impl::getOrCreateComponent(const ignition::gazebo::Entity entity)
{
    auto component = ecm->Component<ComponentTypeT>(entity);

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
    ecm.Each<ignition::gazebo::components::Joint, ignition::gazebo::components::Name>(
        [&](const ignition::gazebo::Entity& /*_entity*/,
            ignition::gazebo::components::Joint* /*joint*/,
            ignition::gazebo::components::Name* name) -> bool {
            gymppDebug << "Found joint: " << name->Data() << std::endl;

            // Find the entity of the joint in the ecm
            auto jointEntity = pImpl->model.JointByName(ecm, name->Data());
            if (jointEntity == ignition::gazebo::kNullEntity) {
                gymppDebug << "Failed to find entity for joint '" << name->Data() << "'"
                           << std::endl;
                return false;
            }

            // Store the joint entity
            pImpl->joints[name->Data()] = jointEntity;

            // Create a PID for each model joint
            pImpl->pids[name->Data()] = DefaultPID;

            return true;
        });

    // Get all the model links
    ecm.Each<ignition::gazebo::components::Link,
             ignition::gazebo::components::Name,
             ignition::gazebo::components::Pose>([&](const ignition::gazebo::Entity& /*_entity*/,
                                                     ignition::gazebo::components::Link* /*link*/,
                                                     ignition::gazebo::components::Name* name,
                                                     ignition::gazebo::components::Pose
                                                         * /*pose*/) -> bool {
        gymppDebug << "Found link: " << name->Data() << std::endl;

        // TODO: there is an extra link 'link', I suspect related to the <include><pose>
        if (name->Data() == "link") {
            gymppDebug << "Skipping dummy link 'link'" << std::endl;
            return true;
        }

        // Find the entity of the joint in the ecm
        auto linkEntity = pImpl->model.LinkByName(ecm, name->Data());
        if (linkEntity == ignition::gazebo::kNullEntity) {
            gymppDebug << "Failed to find entity for link '" << name->Data() << "'" << std::endl;
            //            return false;
        }

        // Store the joint entity
        pImpl->links[name->Data()] = linkEntity;
        return true;
    });

    // Check that the created object is valid
    if (!valid()) {
        gymppError << "The IgnitionRobot object for model '" << pImpl->model.Name(ecm)
                   << "' is not valid" << std::endl;
        return false;
    }

    // Register the robot in the singleton
    if (!RobotSingleton::get().storeRobot(this)) {
        gymppError << "Failed to store the robot in the RobotSingleton" << std::endl;
        return false;
    }

    return true;
}

IgnitionRobot::~IgnitionRobot() = default;
bool IgnitionRobot::valid() const
{
    // TODO: find the proper logic to check if this object is valid

    if (!pImpl->ecm) {
        return false;
    }

    if (pImpl->joints.size() == 0) {
        return false;
    }

    if (pImpl->pids.size() == 0) {
        return false;
    }

    if (pImpl->joints.size() != pImpl->pids.size()) {
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
    assert(pImpl->ecm);
    return pImpl->model.Name(*pImpl->ecm);
}

gympp::Robot::JointNames IgnitionRobot::jointNames() const
{
    JointNames names;

    for (const auto& [jointName, jointEntity] : pImpl->joints) {
        names.push_back(jointName);
    }

    return names;
}

double IgnitionRobot::jointPosition(const gympp::Robot::JointName& jointName) const
{
    JointEntity jointEntity = pImpl->getJointEntity(jointName);
    if (jointEntity == ignition::gazebo::kNullEntity) {
        // TODO
        return {};
    }

    // Get the joint position component
    auto jointPositionComponent =
        pImpl->ecm->Component<ignition::gazebo::components::JointPosition>(jointEntity);

    if (!jointPositionComponent) {
        gymppError << "Position for joint '" << jointName << "' not found in the ecm" << std::endl;
        return {};
    }

    return jointPositionComponent->Data();
}

double IgnitionRobot::jointVelocity(const gympp::Robot::JointName& jointName) const
{
    JointEntity jointEntity = pImpl->getJointEntity(jointName);
    if (jointEntity == ignition::gazebo::kNullEntity) {
        // TODO
        return {};
    }

    // Get the joint velocity component
    auto jointVelocityComponent =
        pImpl->ecm->Component<ignition::gazebo::components::JointVelocity>(jointEntity);

    if (!jointVelocityComponent) {
        gymppError << "Velocity for joint '" << jointName << "' not found in the ecm" << std::endl;
        return {};
    }

    return jointVelocityComponent->Data();
}

gympp::Robot::JointPositions IgnitionRobot::jointPositions() const
{
    // TODO
    return {};
}

gympp::Robot::JointVelocities IgnitionRobot::jointVelocities() const
{
    // TODO
    return {};
}

gympp::Robot::StepSize IgnitionRobot::dt() const
{
    return pImpl->dt;
}

IgnitionRobot::PID IgnitionRobot::jointPID(const gympp::Robot::JointName& jointName) const
{
    if (!pImpl->jointExists(jointName)) {
        return {};
        // TODO
    }

    assert(pImpl->pidExists(jointName));
    auto& pid = pImpl->pids[jointName];
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
        pImpl->getOrCreateComponent<ignition::gazebo::components::JointForce>(jointEntity);

    // Set the joint force
    forceComponent = ignition::gazebo::components::JointForce(jointForce);

    return true;
}

bool IgnitionRobot::setJointPosition(const gympp::Robot::JointName& jointName,
                                     const double jointPositionReference)
{
    JointEntity jointEntity = pImpl->getJointEntity(jointName);
    if (jointEntity == ignition::gazebo::kNullEntity) {
        return false;
    }

    // Get the current joint position
    // TODO: what if it does not exist? This should return optional.
    double jointPositionCurrent = jointPosition(jointName);

    // Use the position PID to process the reference
    assert(pImpl->pidExists(jointName));
    auto& pid = pImpl->pids[jointName];
    double force = pid.Update(jointPositionCurrent - jointPositionReference, pImpl->dt);

    return setJointForce(jointName, force);
}

bool IgnitionRobot::setJointVelocity(const gympp::Robot::JointName& /*jointName*/,
                                     const double /*jointVelocityReference*/)
{
    // TODO
    return {};
}

bool IgnitionRobot::setJointPID(const gympp::Robot::JointName& jointName, const PID& pid)
{
    if (!pImpl->pidExists(jointName)) {
        gymppError << "Joint name '" << jointName << "' is not a controlled joint" << std::endl;
        return false;
    }

    // Update the gains. The other PID parameters do not change.
    pImpl->pids[jointName].SetPGain(pid.p);
    pImpl->pids[jointName].SetIGain(pid.i);
    pImpl->pids[jointName].SetDGain(pid.d);

    return true;
}

bool IgnitionRobot::resetJoint(const gympp::Robot::JointName& jointName,
                               const double jointPosition,
                               const double jointVelocity)
{
    // Reset the joint
    // ===============

    JointEntity jointEntity = pImpl->getJointEntity(jointName);
    if (jointEntity == ignition::gazebo::kNullEntity) {
        return false;
    }

    // Reset the position
    auto& jointPosComponent =
        pImpl->getOrCreateComponent<ignition::gazebo::components::JointPosition>(jointEntity);

    jointPosComponent = ignition::gazebo::components::JointPosition(jointPosition);

    // Reset the velocity
    auto& jointVelComponent =
        pImpl->getOrCreateComponent<ignition::gazebo::components::JointVelocity>(jointEntity);

    jointVelComponent = ignition::gazebo::components::JointVelocity(jointVelocity);

    // Reset the PID
    // =============

    assert(pImpl->pidExists(jointName));
    pImpl->pids[jointName] = DefaultPID;

    return true;
}
