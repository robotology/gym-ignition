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
#include "gympp/gazebo/components/JointPositionReset.h"
#include "gympp/gazebo/components/JointVelocityReset.h"
#include "gympp/gazebo/components/WorldVelocityCmd.h"

#include <eigen3/Eigen/Dense>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/SdfEntityCreator.hh>
#include <ignition/gazebo/components/AngularAcceleration.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/CanonicalLink.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/ContactSensorData.hh>
#include <ignition/gazebo/components/ExternalWorldWrenchCmd.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/components/JointForce.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointType.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/LinearAcceleration.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
#include <ignition/math/PID.hh>
#include <ignition/math/eigen3/Conversions.hh>
#include <ignition/msgs/contacts.pb.h>
#include <sdf/Joint.hh>

#include <cassert>
#include <chrono>
#include <map>
#include <ostream>
#include <string>

using namespace gympp::gazebo;
using namespace ignition::gazebo;

using LinkName = std::string;
using JointName = std::string;
using LinkEntity = ignition::gazebo::Entity;
using JointEntity = ignition::gazebo::Entity;

const std::string World2BaseJoint = "world2base";
const ignition::math::PID DefaultPID(1, 0.1, 0.01, 1, -1, 10000, -10000);
const gympp::JointControlMode DefaultJointControlMode = gympp::JointControlMode::Torque;

class IgnitionRobot::Impl
{
public:
    struct
    {
        struct
        {
            gympp::Robot::JointPositions positions;
            gympp::Robot::JointVelocities velocities;
            std::map<JointName, double> references;
            std::map<JointName, ignition::math::PID> pid;
            std::map<JointName, JointControlMode> controlMode;
            std::map<JointName, double> appliedForces;
            std::map<JointName, double> effortLimits;
        } joints;
        struct
        {
            std::unordered_map<LinkName, std::vector<gympp::ContactData>> contacts;
        } links;
    } buffers;

    std::string name;

    ignition::gazebo::EventManager* eventManager = nullptr;
    ignition::gazebo::EntityComponentManager* ecm = nullptr;

    ignition::gazebo::Model model;

    std::chrono::duration<double> dt;
    std::chrono::duration<double> prevUpdateTime = std::chrono::duration<double>(0.0);

    bool floating = true;
    std::map<LinkName, LinkEntity> links;
    std::map<JointName, JointEntity> joints;

    inline bool jointExists(const JointName& jointName) const
    {
        return joints.find(jointName) != joints.end();
    }

    inline bool pidExists(const JointName& jointName) const
    {
        return buffers.joints.pid.find(jointName) != buffers.joints.pid.end();
    }

    inline bool linkExists(const LinkName& linkName) const
    {
        return links.find(linkName) != links.end();
    }

    inline bool world2baseExists()
    {
        return model.JointByName(*ecm, World2BaseJoint) != ignition::gazebo::kNullEntity;
    }

    void gatherLinksContactData();
    JointEntity getLinkEntity(const LinkName& linkName);
    JointEntity getJointEntity(const JointName& jointName);

    template <typename ComponentType>
    ComponentType& getOrCreateComponent(const ignition::gazebo::Entity entity);

    static gympp::Pose fromIgnitionMath(const ignition::math::Pose3d& ignitionPose);
    static ignition::math::Pose3d toIgnitionMath(const gympp::Pose& pose);

    static inline std::array<double, 3>
    fromIgnitionMath(const ignition::math::Vector3d& ignitionVector)
    {
        return {ignitionVector.X(), ignitionVector.Y(), ignitionVector.Z()};
    }

    static inline ignition::math::Vector3d toIgnitionMath(const std::array<double, 3>& vector)
    {
        return {vector[0], vector[1], vector[2]};
    }

    static inline ignition::math::PID toIgnitionMath(const gympp::PID& pid)
    {
        return {pid.p, pid.i, pid.d};
    }
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

void IgnitionRobot::Impl::gatherLinksContactData()
{
    auto getEntityName = [&](const ignition::gazebo::Entity entity) -> std::string {
        auto nameComponent = ecm->Component<ignition::gazebo::components::Name>(entity);
        assert(nameComponent);
        return nameComponent->Data();
    };

    auto getParentModelNameFromCollisionEntity =
        [&](const ignition::gazebo::Entity collisionEntity) -> std::string {
        // The hierarchy must be: model -> link -> collision
        auto linkEntity = ecm->ParentEntity(collisionEntity);
        auto modelEntity = ecm->ParentEntity(linkEntity);
        return getEntityName(modelEntity);
    };

    ecm->Each<ignition::gazebo::components::Collision,
              ignition::gazebo::components::ContactSensorData,
              ignition::gazebo::components::ParentEntity>(
        [&](const ignition::gazebo::Entity& /*collisionEntity*/,
            ignition::gazebo::components::Collision* /*collisionComponent*/,
            ignition::gazebo::components::ContactSensorData* contactDataComponent,
            ignition::gazebo::components::ParentEntity* linkEntityComponent) -> bool {
            // Get the model entity of the link that contains the collision
            auto modelEntity = ecm->ParentEntity(linkEntityComponent->Data());

            // Keep only collisions of links that belong to the handled model
            if (modelEntity != model.Entity()) {
                return true;
            }

            // Get the name of the link associated to this collision entity
            std::string linkName =
                ecm->Component<ignition::gazebo::components::Name>(linkEntityComponent->Data())
                    ->Data();

            // Extract the contacts data structure
            ignition::msgs::Contacts contactsData = contactDataComponent->Data();

            assert(buffers.links.contacts.find(linkName) != buffers.links.contacts.end());
            buffers.links.contacts[linkName].clear();
            buffers.links.contacts[linkName].reserve(contactsData.contact_size());

            // Process each contact
            for (int i = 0; i < contactsData.contact_size(); ++i) {
                // Extract the contact object
                const ignition::msgs::Contact& contactData = contactsData.contact(i);

                std::string scopedBodyA =
                    getParentModelNameFromCollisionEntity(contactData.collision1().id())
                    + "::" + getEntityName(contactData.collision1().id());

                std::string scopedBodyB =
                    getParentModelNameFromCollisionEntity(contactData.collision2().id())
                    + "::" + getEntityName(contactData.collision2().id());

                // Extract the contact data
                ContactData contact;
                contact.bodyA = scopedBodyA;
                contact.bodyB = scopedBodyB;
                contact.position[0] = contactData.position(i).x();
                contact.position[1] = contactData.position(i).y();
                contact.position[2] = contactData.position(i).z();

                // TODO: fill the normal
                // TODO: fill the contacty depth
                // TODO: fill the wrench magnitude
                assert(contactData.depth_size() == 0);
                assert(contactData.normal_size() == 0);
                assert(contactData.wrench_size() == 0);

                // Add the new contact data to the link's buffer
                buffers.links.contacts[linkName].push_back(contact);
            }

            return true;
        });
}

gympp::Pose IgnitionRobot::Impl::fromIgnitionMath(const ignition::math::Pose3d& ignitionPose)
{
    gympp::Pose pose;
    pose.position[0] = ignitionPose.Pos().X();
    pose.position[1] = ignitionPose.Pos().Y();
    pose.position[2] = ignitionPose.Pos().Z();
    pose.orientation[0] = ignitionPose.Rot().W();
    pose.orientation[1] = ignitionPose.Rot().X();
    pose.orientation[2] = ignitionPose.Rot().Y();
    pose.orientation[3] = ignitionPose.Rot().Z();
    return pose;
}

ignition::math::Pose3d IgnitionRobot::Impl::toIgnitionMath(const gympp::Pose& pose)
{
    ignition::math::Pose3d ignitionPose;
    ignitionPose.Pos() =
        ignition::math::Vector3d(pose.position[0], pose.position[1], pose.position[2]);
    ignitionPose.Rot() = ignition::math::Quaterniond(
        pose.orientation[0], pose.orientation[1], pose.orientation[2], pose.orientation[3]);
    return ignitionPose;
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
                                 ignition::gazebo::EntityComponentManager* ecm,
                                 ignition::gazebo::EventManager* eventManager)
{
    if (!ecm || !eventManager) {
        gymppError << "Either the ECM or the event mananger are not valid" << std::endl;
        return false;
    }

    // Store the address of the entity-component manager
    pImpl->ecm = ecm;
    pImpl->eventManager = eventManager;

    // Create the model
    pImpl->model = ignition::gazebo::Model(entity);

    // Check that the model is valid
    if (!pImpl->model.Valid(*ecm)) {
        gymppError << "The SDF model '" << pImpl->model.Name(*ecm) << "' is not valid" << std::endl;
        return false;
    }

    gymppDebug << "Processing model '" << pImpl->model.Name(*ecm) << "'" << std::endl;

    ecm->Each<ignition::gazebo::components::Joint,
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

            gymppDebug << "  Found joint: " << pImpl->model.Name(*ecm) << "::" << name->Data()
                       << " [" << entity << "]" << std::endl;

            // Find the entity of the joint in the ecm
            auto jointEntity = pImpl->model.JointByName(*ecm, name->Data());
            if (jointEntity == ignition::gazebo::kNullEntity) {
                gymppError << "Failed to find entity of joint '" << pImpl->model.Name(*ecm)
                           << "::" << name->Data() << "'" << std::endl;
                assert(false);
                return true;
            }

            // Ignore fixed joints
            if (type->Data() == sdf::JointType::FIXED) {
                gymppDebug << "  Skipping fixed joint '" << pImpl->model.Name(*ecm)
                           << "::" << name->Data() << "'" << std::endl;
                return true;
            }

            // Store the joint entity
            pImpl->joints[name->Data()] = jointEntity;

            return true;
        });

    // Control all the joints in Torque by default
    for (const auto& jointName : jointNames()) {
        if (!setJointControlMode(jointName, DefaultJointControlMode)) {
            gymppError << "Failed to set the default control mode of joint '" << jointName << "'"
                       << std::endl;
            return false;
        }
    }

    // Create the joint position, velocity and force components.
    // In this way this data is stored in the components after the physics step.
    for (auto& [_, jointEntity] : pImpl->joints) {
        ecm->CreateComponent(jointEntity, ignition::gazebo::components::JointPosition());
        ecm->CreateComponent(jointEntity, ignition::gazebo::components::JointVelocity());
        ecm->CreateComponent(jointEntity, ignition::gazebo::components::JointForce({0.0}));
    }

    // Get all the model links
    ecm->Each<ignition::gazebo::components::Link,
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

            gymppDebug << "  Found link: " << pImpl->model.Name(*ecm) << "::" << name->Data()
                       << " [" << entity << "]" << std::endl;

            // TODO: there is an extra link 'link', I suspect related to the <include><pose>
            if (name->Data() == "link") {
                gymppDebug << "  Skipping dummy link 'link'" << std::endl;
                return true;
            }

            // Find the entity of the link in the ecm
            auto linkEntity = pImpl->model.LinkByName(*ecm, name->Data());
            if (linkEntity == ignition::gazebo::kNullEntity) {
                gymppError << "Failed to find entity for link '" << pImpl->model.Name(*ecm)
                           << "::" << name->Data() << "'" << std::endl;
                assert(false);
                return true;
            }

            // Store the link entity
            pImpl->links[name->Data()] = linkEntity;
            return true;
        });

    // Initialize the contacts buffers for each link
    for (const auto& [linkName, LinkEntity] : pImpl->links) {
        pImpl->buffers.links.contacts[linkName] = {};
    }

    // Create the components to store link quantities.
    // The Physics system fills the components during the simulation.
    for (auto& [_, linkEntity] : pImpl->links) {
        ecm->CreateComponent(linkEntity, ignition::gazebo::components::WorldPose());
        ecm->CreateComponent(linkEntity, ignition::gazebo::components::WorldLinearVelocity());
        ecm->CreateComponent(linkEntity, ignition::gazebo::components::WorldAngularVelocity());
        ecm->CreateComponent(linkEntity, ignition::gazebo::components::WorldLinearAcceleration());
        ecm->CreateComponent(linkEntity, ignition::gazebo::components::WorldAngularAcceleration());
        ecm->CreateComponent(linkEntity, ignition::gazebo::components::LinearVelocity());
        ecm->CreateComponent(linkEntity, ignition::gazebo::components::AngularVelocity());
        ecm->CreateComponent(linkEntity, ignition::gazebo::components::LinearAcceleration());
        ecm->CreateComponent(linkEntity, ignition::gazebo::components::AngularAcceleration());
    }

    // Check that the created object is valid
    if (!valid()) {
        gymppError << "The IgnitionRobot object for model '" << pImpl->model.Name(*ecm)
                   << "' is not valid" << std::endl;
        return false;
    }

    // Store the name of the robot
    pImpl->name = pImpl->model.Name(*ecm);

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

    if (pImpl->links.size() == 0) {
        return false;
    }

    return true;
}

size_t IgnitionRobot::dofs() const
{
    return jointNames().size();
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

gympp::JointType IgnitionRobot::jointType(const gympp::Robot::JointName& jointName) const
{
    JointEntity jointEntity = pImpl->getJointEntity(jointName);
    if (jointEntity == ignition::gazebo::kNullEntity) {
        return JointType::Invalid;
    }

    auto typeComponent =
        pImpl->ecm->Component<ignition::gazebo::components::JointType>(jointEntity);
    assert(typeComponent);

    gympp::JointType returnedType;
    sdf::JointType type = typeComponent->Data();

    switch (type) {
        case sdf::JointType::FIXED:
            returnedType = JointType::Fixed;
            break;
        case sdf::JointType::REVOLUTE:
            returnedType = JointType::Revolute;
            break;
        case sdf::JointType::PRISMATIC:
            returnedType = JointType::Prismatic;
            break;
        default:
            gymppError << "Joint type not recognized" << std::endl;
            returnedType = JointType::Invalid;
            break;
    }

    return returnedType;
}

double IgnitionRobot::jointForce(const gympp::Robot::JointName& jointName) const
{
    JointEntity jointEntity = pImpl->getJointEntity(jointName);
    if (jointEntity == ignition::gazebo::kNullEntity) {
        assert(false);
        return 0.0;
    }

    // Get the joint force component
    auto jointForceComponent =
        pImpl->ecm->Component<ignition::gazebo::components::JointForce>(jointEntity);

    // If the component does not exists, it means that no force command has been set
    // in the previous step.
    if (!(jointForceComponent && !jointForceComponent->Data().empty())) {
        return 0.0;
    }

    // Return the last applied force reference (first DoF of the joint)
    return jointForceComponent->Data().front();
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
        assert(false);
        return {};
    }

    if (jointPositionComponent->Data().size() <= 0) {
        gymppWarning << "The joint position component exists but it does not have yet any data"
                     << std::endl;
        assert(false);
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
        assert(false);
        return {};
    }

    if (jointVelocityComponent->Data().size() <= 0) {
        gymppWarning << "The joint velocity component exists but it does not have yet any data"
                     << std::endl;
        assert(false);
        return {};
    }

    assert(jointVelocityComponent->Data().size() == 1);
    return jointVelocityComponent->Data()[0];
}

gympp::JointControlMode
IgnitionRobot::jointControlMode(const gympp::Robot::JointName& jointName) const
{
    return pImpl->buffers.joints.controlMode[jointName];
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

gympp::Robot::JointPositions IgnitionRobot::initialJointPositions() const
{
    JointPositions initialJointPositions;
    initialJointPositions.reserve(dofs());

    for (const auto& name : jointNames()) {
        JointEntity jointEntity = pImpl->getJointEntity(name);
        assert(jointEntity != ignition::gazebo::kNullEntity);

        // Get the joint axis component
        auto jointAxisComponent =
            pImpl->ecm->Component<ignition::gazebo::components::JointAxis>(jointEntity);

        if (!jointAxisComponent) {
            gymppError << "JointAxis of joint '" << name << "' not found in the ecm" << std::endl;
            assert(false);
            return {};
        }

        initialJointPositions.push_back(jointAxisComponent->Data().InitialPosition());
    }

    return initialJointPositions;
}

double IgnitionRobot::jointEffortLimit(const gympp::Robot::JointName& jointName) const
{
    JointEntity jointEntity = pImpl->getJointEntity(jointName);
    if (jointEntity == ignition::gazebo::kNullEntity) {
        assert(false);
        return 0.0;
    }

    // The effort limit specified in the SDF is not currently exposed in the ECM.
    // We use our buffer to store the limit.
    auto& effortLimits = pImpl->buffers.joints.effortLimits;

    // Return the biggest double if no limit was set
    if (effortLimits.find(jointName) == effortLimits.end()) {
        return std::numeric_limits<double>::max();
    }

    assert(effortLimits[jointName] >= 0);
    return effortLimits[jointName];
}

gympp::Limit IgnitionRobot::jointPositionLimits(const gympp::Robot::JointName& jointName) const
{
    JointEntity jointEntity = pImpl->getJointEntity(jointName);
    assert(jointEntity != ignition::gazebo::kNullEntity);

    // Get the joint axis component
    auto jointAxisComponent =
        pImpl->ecm->Component<ignition::gazebo::components::JointAxis>(jointEntity);

    if (!jointAxisComponent) {
        gymppError << "JointAxis of joint '" << jointName << "' not found in the ecm" << std::endl;
        assert(false);
        return {};
    }

    gympp::Limit limit;
    limit.min = jointAxisComponent->Data().Lower();
    limit.max = jointAxisComponent->Data().Upper();

    return limit;
}

gympp::Robot::StepSize IgnitionRobot::dt() const
{
    return pImpl->dt;
}

gympp::Robot::PID IgnitionRobot::jointPID(const gympp::Robot::JointName& jointName) const
{
    assert(pImpl->jointExists(jointName));
    assert(pImpl->pidExists(jointName));

    auto& pid = pImpl->buffers.joints.pid[jointName];
    return PID(pid.PGain(), pid.IGain(), pid.DGain());
}

gympp::Robot::LinkNames IgnitionRobot::linksInContact() const
{
    // Acquire the data
    pImpl->gatherLinksContactData();

    // Return the list of links that have collisions with other bodies
    LinkNames linksInContact;

    for (const auto& [linkName, linkContacts] : pImpl->buffers.links.contacts) {
        if (linkContacts.size() > 0) {
            linksInContact.push_back(linkName);
        }
    }

    return linksInContact;
}

std::vector<gympp::ContactData>
IgnitionRobot::contactData(const gympp::Robot::LinkName& linkName) const
{
    pImpl->gatherLinksContactData();
    return pImpl->buffers.links.contacts.at(linkName);
}

gympp::Robot::LinkNames IgnitionRobot::linkNames() const
{
    LinkNames names;
    names.reserve(pImpl->links.size());

    for (const auto& [linkName, _] : pImpl->links) {
        names.push_back(linkName);
    }

    return names;
}

gympp::Pose IgnitionRobot::linkPose(const gympp::Robot::LinkName& linkName) const
{
    LinkEntity linkEntity = pImpl->getLinkEntity(linkName);
    assert(linkEntity != ignition::gazebo::kNullEntity);

    // Get the link world pose component expressed in the world frame
    auto linkWorldPoseComponent =
        pImpl->ecm->Component<ignition::gazebo::components::WorldPose>(linkEntity);
    assert(linkWorldPoseComponent);

    // Get the link pose
    const ignition::math::Pose3d& linkWorldPoseGazebo = linkWorldPoseComponent->Data();

    return Impl::fromIgnitionMath(linkWorldPoseGazebo);
}

gympp::Velocity6D IgnitionRobot::linkVelocity(const gympp::Robot::LinkName& linkName) const
{
    LinkEntity linkEntity = pImpl->getLinkEntity(linkName);
    assert(linkEntity != ignition::gazebo::kNullEntity);

    // Get the link linear velocity componenent expressed in the world frame.
    auto linkWorldLinVelComponent =
        pImpl->ecm->Component<ignition::gazebo::components::WorldLinearVelocity>(linkEntity);
    assert(linkWorldLinVelComponent);

    // Get the link angular velocity componenent expressed in the world frame.
    auto linkWorldAngVelComponent =
        pImpl->ecm->Component<ignition::gazebo::components::WorldAngularVelocity>(linkEntity);
    assert(linkWorldAngVelComponent);

    // Get the link velocities
    const ignition::math::Vector3d& linkWorldLinVel = linkWorldLinVelComponent->Data();
    const ignition::math::Vector3d& linkWorldAngVel = linkWorldAngVelComponent->Data();

    // Fill the output data structure
    gympp::Velocity6D linkWorldVelocity;
    linkWorldVelocity.linear = Impl::fromIgnitionMath(linkWorldLinVel);
    linkWorldVelocity.angular = Impl::fromIgnitionMath(linkWorldAngVel);

    return linkWorldVelocity;
}

gympp::Acceleration6D IgnitionRobot::linkAcceleration(const gympp::Robot::LinkName& linkName) const
{
    LinkEntity linkEntity = pImpl->getLinkEntity(linkName);
    assert(linkEntity != ignition::gazebo::kNullEntity);

    // Get the link linear acceleration componenent expressed in the world frame.
    auto linkWorldLinAccComponent =
        pImpl->ecm->Component<ignition::gazebo::components::WorldLinearAcceleration>(linkEntity);
    assert(linkWorldLinAccComponent);

    // Get the link angular acceleration componenent expressed in the world frame.
    auto linkWorldAngAccComponent =
        pImpl->ecm->Component<ignition::gazebo::components::WorldAngularAcceleration>(linkEntity);
    assert(linkWorldAngAccComponent);

    // Get the link accelerations
    const ignition::math::Vector3d& linkWorldLinAcc = linkWorldLinAccComponent->Data();
    const ignition::math::Vector3d& linkWorldAngAcc = linkWorldAngAccComponent->Data();

    // Fill the output data structure
    gympp::Acceleration6D linkWorldAcceleration;
    linkWorldAcceleration.linear = Impl::fromIgnitionMath(linkWorldLinAcc);
    linkWorldAcceleration.angular = Impl::fromIgnitionMath(linkWorldAngAcc);

    return linkWorldAcceleration;
}

gympp::Velocity6D IgnitionRobot::linkBodyFixedVelocity(const gympp::Robot::LinkName& linkName) const
{
    LinkEntity linkEntity = pImpl->getLinkEntity(linkName);
    assert(linkEntity != ignition::gazebo::kNullEntity);

    // Get the link linear velocity componenent expressed in the link frame.
    auto linkBodyLinVelComponent =
        pImpl->ecm->Component<ignition::gazebo::components::LinearVelocity>(linkEntity);
    assert(linkBodyLinVelComponent);

    // Get the link angular velocity componenent expressed in the link frame.
    auto linkBodyAngVelComponent =
        pImpl->ecm->Component<ignition::gazebo::components::AngularVelocity>(linkEntity);
    assert(linkBodyAngVelComponent);

    // Get the link velocities
    const ignition::math::Vector3d& linkBodyLinVel = linkBodyLinVelComponent->Data();
    const ignition::math::Vector3d& linkBodyAngVel = linkBodyAngVelComponent->Data();

    // Fill the output data structure
    gympp::Velocity6D linkBodyVelocity;
    linkBodyVelocity.linear = Impl::fromIgnitionMath(linkBodyLinVel);
    linkBodyVelocity.angular = Impl::fromIgnitionMath(linkBodyAngVel);

    return linkBodyVelocity;
}

gympp::Acceleration6D
IgnitionRobot::linkBodyFixedAcceleration(const gympp::Robot::LinkName& linkName) const
{
    LinkEntity linkEntity = pImpl->getLinkEntity(linkName);
    assert(linkEntity != ignition::gazebo::kNullEntity);

    // Get the link linear acceleration componenent expressed in the link frame.
    auto linkBodyLinAccComponent =
        pImpl->ecm->Component<ignition::gazebo::components::LinearAcceleration>(linkEntity);
    assert(linkBodyLinAccComponent);

    // Get the link angular acceleration componenent expressed in the link frame.
    auto linkBodyAngAccComponent =
        pImpl->ecm->Component<ignition::gazebo::components::AngularAcceleration>(linkEntity);
    assert(linkBodyAngAccComponent);

    // Get the link velocities
    const ignition::math::Vector3d& linkBodyLinAcc = linkBodyLinAccComponent->Data();
    const ignition::math::Vector3d& linkBodyAngAcc = linkBodyAngAccComponent->Data();

    // Fill the output data structure
    gympp::Acceleration6D linkBodyAcceleration;
    linkBodyAcceleration.linear = Impl::fromIgnitionMath(linkBodyLinAcc);
    linkBodyAcceleration.angular = Impl::fromIgnitionMath(linkBodyAngAcc);

    return linkBodyAcceleration;
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

bool IgnitionRobot::setJointEffortLimit(const gympp::Robot::JointName& jointName,
                                        const double effortLimit)
{
    JointEntity jointEntity = pImpl->getJointEntity(jointName);
    if (jointEntity == ignition::gazebo::kNullEntity) {
        return false;
    }

    if (effortLimit < 0) {
        gymppError << "The effort must be greater or equal than 0" << std::endl;
        return false;
    }

    pImpl->buffers.joints.effortLimits[jointName] = effortLimit;
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

    if (jointControlMode(jointName) != JointControlMode::Position) {
        gymppError << "Cannot set the position target of joint '" << jointName
                   << "' not controlled in Position" << std::endl;
        return false;
    }

    JointEntity jointEntity = pImpl->getJointEntity(jointName);
    if (jointEntity == ignition::gazebo::kNullEntity) {
        return false;
    }

    // Create the PID if it does not exist
    if (!pImpl->pidExists(jointName)) {
        pImpl->buffers.joints.pid[jointName] = DefaultPID;
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

    if (jointControlMode(jointName) != JointControlMode::Velocity) {
        gymppError << "Cannot set the velocity target of joint '" << jointName
                   << "' not controlled in Velocity" << std::endl;
        return false;
    }

    JointEntity jointEntity = pImpl->getJointEntity(jointName);
    if (jointEntity == ignition::gazebo::kNullEntity) {
        return false;
    }

    // Create the PID if it does not exist
    if (!pImpl->pidExists(jointName)) {
        pImpl->buffers.joints.pid[jointName] = DefaultPID;
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

    // Store the new position in the ECM.
    // This is necessary because before the physics engine executes the step and resets the
    // position, we might need to read the JointPosition component that might be uninitialized.
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

    // Store the new velocity in the ECM.
    // This is necessary because before the physics engine executes the step and resets the
    // velocity, we might need to read the JointVelocity component that might be uninitialized.
    auto& jointVelComponent =
        pImpl->getOrCreateComponent<ignition::gazebo::components::JointVelocity>(jointEntity);

    jointVelComponent = ignition::gazebo::components::JointVelocity({jointVelocity});

    return true;
}

bool IgnitionRobot::setJointControlMode(const gympp::Robot::JointName& jointName,
                                        const JointControlMode controlMode)
{
    // Clean up possible old references
    pImpl->buffers.joints.references.erase(jointName);

    // Clean the buffer that stores the force
    pImpl->buffers.joints.appliedForces.erase(jointName);

    // Update the control mode
    pImpl->buffers.joints.controlMode[jointName] = controlMode;
    return true;
}

bool IgnitionRobot::setJointPID(const gympp::Robot::JointName& jointName, const PID& pid)
{
    JointEntity jointEntity = pImpl->getJointEntity(jointName);
    if (jointEntity == ignition::gazebo::kNullEntity) {
        return false;
    }

    // Create a new PID
    pImpl->buffers.joints.pid[jointName] = Impl::toIgnitionMath(pid);
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
        pImpl->buffers.joints.pid[jointName].Reset();
    }

    // Clean the joint controlling storage
    pImpl->buffers.joints.references.erase(jointName);
    return true;
}

bool IgnitionRobot::addExternalWrench(const gympp::Robot::LinkName& linkName,
                                      const std::array<double, 3>& force,
                                      const std::array<double, 3>& torque)
{
    LinkEntity linkEntity = pImpl->getLinkEntity(linkName);
    if (linkEntity == ignition::gazebo::kNullEntity) {
        return false;
    }

    // Get the ExternalWorldWrenchCmd component
    auto& externalWrenchCmd =
        pImpl->getOrCreateComponent<ignition::gazebo::components::ExternalWorldWrenchCmd>(
            linkEntity);

    auto forceMsg = std::make_unique<ignition::msgs::Vector3d>();
    auto torqueMsg = std::make_unique<ignition::msgs::Vector3d>();
    ignition::msgs::Wrench& externalWrench = externalWrenchCmd.Data();

    forceMsg->set_x(force[0]);
    forceMsg->set_y(force[1]);
    forceMsg->set_z(force[2]);

    torqueMsg->set_x(torque[0]);
    torqueMsg->set_y(torque[1]);
    torqueMsg->set_z(torque[2]);

    externalWrench.set_allocated_force(forceMsg.release());
    externalWrench.set_allocated_torque(torqueMsg.release());
    return true;
}

bool IgnitionRobot::update(const std::chrono::duration<double>& simTime)
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
    std::chrono::duration<double> stepTime = simTime - pImpl->prevUpdateTime;

    // Handle first iteration
    if (pImpl->prevUpdateTime == std::chrono::duration<double>(0.0)) {
        stepTime = pImpl->dt;
    }

    // If enough time is passed, store the time of this actuation step. In this case the state
    // of the robot is read and new force references are computed and actuated. Otherwise, the
    // same force of the last step is actuated.
    bool computeNewForce;

    // Due to numerical floating point approximations, sometimes a comparison of chrono durations
    // has an error in the 1e-18 order
    auto greaterThen = [](const std::chrono::duration<double>& a,
                          const std::chrono::duration<double>& b) -> bool {
        return a.count() >= b.count() - std::numeric_limits<double>::epsilon();
    };

    if (greaterThen(stepTime, pImpl->dt)) {
        // Store the current update time
        pImpl->prevUpdateTime = simTime;

        // Enable using the PID to compute the new force
        computeNewForce = true;
    }
    else {
        // Disable the PID and send the same force reference as last update
        computeNewForce = false;
    }

    // Actuate the references.
    // The references can be either position or velocity references.
    for (const auto& [jointName, reference] : pImpl->buffers.joints.references) {
        assert(pImpl->pidExists(jointName));

        // Use the PID the compute the new force
        if (computeNewForce) {
            double force = 0;

            // Get the PID
            auto& pid = pImpl->buffers.joints.pid[jointName];

            // Use the PID to get the reference
            switch (pImpl->buffers.joints.controlMode[jointName]) {
                case JointControlMode::Position:
                    force = pid.Update(jointPosition(jointName) - reference, stepTime);
                    break;
                case JointControlMode::Velocity:
                    force = pid.Update(jointVelocity(jointName) - reference, stepTime);
                    break;
                default:
                    gymppError << "Joint control mode '"
                               << int(pImpl->buffers.joints.controlMode[jointName])
                               << "' not supported" << std::endl;
                    return false;
            }

            // Store the force
            pImpl->buffers.joints.appliedForces[jointName] = force;
        }

        assert(pImpl->buffers.joints.appliedForces.find(jointName)
               != pImpl->buffers.joints.appliedForces.end());

        // Get the force
        auto force = pImpl->buffers.joints.appliedForces[jointName];

        // Get the effort limits
        auto& effortLimits = pImpl->buffers.joints.effortLimits;

        // Clip the force if an effort limit was set
        if (effortLimits.find(jointName) != effortLimits.end()) {

            if (std::abs(force) > effortLimits[jointName]) {
                force = std::min(force, effortLimits[jointName]);
                force = std::max(force, -effortLimits[jointName]);
            }
        }

        // Actuate the force
        if (!setJointForce(jointName, force)) {
            gymppError << "Failed to set force to joint '" << jointName << "'" << std::endl;
            return false;
        }
    }

    return true;
}

// ==============
// RobotBaseFrame
// ==============

gympp::Robot::LinkName IgnitionRobot::baseFrame()
{
    // Get all the canonical links of the model
    auto candidateBaseLinks = pImpl->ecm->EntitiesByComponents(
        components::CanonicalLink(), components::ParentEntity(pImpl->model.Entity()));

    if (candidateBaseLinks.size() == 0) {
        gymppError << "Failed to find base link of model '" << pImpl->name << "'" << std::endl;
        assert(false);
        return {};
    }

    assert(candidateBaseLinks.size() == 1);

    // Return the base link name
    auto baseLinkName = pImpl->ecm->Component<components::Name>(candidateBaseLinks[0])->Data();
    return baseLinkName;
}

bool IgnitionRobot::setBaseFrame(const gympp::Robot::LinkName& baseLink)
{
    if (!pImpl->linkExists(baseLink)) {
        gymppError << "Failed to set base frame to not existing link '" << baseLink << "'"
                   << std::endl;
        return false;
    }

    if (baseLink == baseFrame()) {
        gymppDebug << "The base is already set on the '" << baseLink << "' link" << std::endl;
        return true;
    }

    gymppError << "Changing the base link is not yet supported" << std::endl;
    return false;
}

gympp::Pose IgnitionRobot::basePose()
{
    // Get the pose component
    auto* modelWorldPoseComponent =
        pImpl->ecm->Component<ignition::gazebo::components::Pose>(pImpl->model.Entity());
    assert(modelWorldPoseComponent);

    // Extract the transform
    const ignition::math::Pose3d& world_H_model = modelWorldPoseComponent->Data();

    return Impl::fromIgnitionMath(world_H_model);
}

gympp::Velocity6D IgnitionRobot::baseVelocity()
{
    // We can get the velocity of the base link. Since there's only a rigid
    // transformation between base and model frame, and the velocity is expressed
    // in the world frame, we do not need to perform any conversion.

    // Get the name of the base link
    const LinkName& baseLink = baseFrame();

    // Get the velocity of the base link
    gympp::Velocity6D baseVelocity = linkVelocity(baseLink);

    // Convert it to ignition math objects
    ignition::math::Vector3d baseLinVel = Impl::toIgnitionMath(baseVelocity.linear);
    ignition::math::Vector3d baseAngVel = Impl::toIgnitionMath(baseVelocity.angular);

    // Create the output data structure
    gympp::Velocity6D modelVelocity;
    modelVelocity.linear = {baseLinVel.X(), baseLinVel.Y(), baseLinVel.Z()};
    modelVelocity.angular = {baseAngVel.X(), baseAngVel.Y(), baseAngVel.Z()};

    // Return the velocity of the model frame
    return modelVelocity;
}

bool IgnitionRobot::setAsFloatingBase(bool isFloating)
{
    // Fixing the model to the world involves two steps:
    // 1) Creating a new joint to fix the robot base to the world
    // 2) Setting the desired pose of the fixed base
    //
    // At the moment these steps cannot be done in a single iteration because setting the pose
    // of an entity (the new joint) has to be done after the physics engine already processed
    // the new joint performing a step.
    // Since this behavior is confusing, we only support fixing the robot at its last processed
    // base pose. We do not expose yet both features together.

    if (!pImpl->floating && isFloating) {
        gymppError << "Converting a fixed-base robot to a floating-base robot is not yet supported"
                   << std::endl;
        return false;
    }

    if (pImpl->floating && !isFloating) {
        gymppDebug << "Fixing robot at the current pose" << std::endl;
        assert(!pImpl->world2baseExists());

        sdf::Joint joint;
        joint.SetName(World2BaseJoint);
        joint.SetParentLinkName("world");
        joint.SetChildLinkName(baseFrame());
        joint.SetType(sdf::JointType::FIXED);

        auto creator = SdfEntityCreator(*pImpl->ecm, *pImpl->eventManager);
        auto jointEntity = creator.CreateEntities(&joint);

        pImpl->ecm->CreateComponent(jointEntity, components::ParentEntity(pImpl->model.Entity()));
        pImpl->ecm->SetParentEntity(jointEntity, pImpl->model.Entity());
    }

    pImpl->floating = isFloating;
    return true;
}

bool IgnitionRobot::resetBasePose(const std::array<double, 3>& position,
                                  const std::array<double, 4>& orientation)
{
    if (!pImpl->floating) {
        gymppError << "The pose of fixed-base robots cannot be changed" << std::endl;
        return false;
    }

    // Construct the desired transform between world and base
    gympp::Pose pose;
    pose.position = position;
    pose.orientation = orientation;
    ignition::math::Pose3d world_H_base = Impl::toIgnitionMath(pose);

    // Get the name of the canonical link
    const LinkName& baseLink = baseFrame();

    // Get the Pose component of the canonical link.
    // This is the fixed transformation between the model and the base
    const auto* canonicalLinkPose =
        pImpl->ecm->Component<ignition::gazebo::components::Pose>(pImpl->getLinkEntity(baseLink));
    assert(canonicalLinkPose);

    // Get the transform of the canonical link
    const ignition::math::Pose3d& model_H_base = canonicalLinkPose->Data();

    // Compute the robot pose that corresponds to the desired base pose
    const ignition::math::Pose3d& world_H_model = world_H_base * model_H_base.Inverse();

    // Get the component that stores the new pose
    auto& modelWorldPoseCmdComponent =
        pImpl->getOrCreateComponent<ignition::gazebo::components::WorldPoseCmd>(
            pImpl->model.Entity());

    // Store the pose data
    modelWorldPoseCmdComponent.Data() = world_H_model;

    // Update the current pose of the model so that callers of getBasePose already
    // read the new one.
    auto* modelWorldPoseComponent =
        pImpl->ecm->Component<ignition::gazebo::components::Pose>(pImpl->model.Entity());
    assert(modelWorldPoseComponent);

    modelWorldPoseComponent->Data() = world_H_model;

    return true;
}

bool IgnitionRobot::resetBaseVelocity(const std::array<double, 3>& linear,
                                      const std::array<double, 3>& angular)
{
    if (!pImpl->floating) {
        gymppError << "The angular velocity of fixed-base robots cannot be changed" << std::endl;
        return false;
    }

    // Get the component that stores the new pose
    auto& modelVelCmdComponent =
        pImpl->getOrCreateComponent<ignition::gazebo::components::WorldVelocityCmd>(
            pImpl->model.Entity());

    // Store the new velocity
    modelVelCmdComponent.Data().linear = Impl::toIgnitionMath(linear);
    modelVelCmdComponent.Data().angular = Impl::toIgnitionMath(angular);

    return true;
}

std::array<double, 6> IgnitionRobot::baseWrench()
{
    assert(false);
    return {};
}
