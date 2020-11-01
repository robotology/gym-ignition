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

#ifndef SCENARIO_GAZEBO_HELPERS_H
#define SCENARIO_GAZEBO_HELPERS_H

#include "scenario/gazebo/Joint.h"
#include "scenario/gazebo/Link.h"
#include "scenario/gazebo/Model.h"
#include "scenario/gazebo/World.h"
#include "scenario/gazebo/exceptions.h"

#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/math/PID.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector4.hh>
#include <ignition/msgs/Utility.hh>
#include <ignition/msgs/contacts.pb.h>
#include <ignition/msgs/vector3d.pb.h>
#include <ignition/msgs/wrench.pb.h>
#include <sdf/Element.hh>
#include <sdf/Joint.hh>
#include <sdf/Root.hh>

#include <algorithm>
#include <array>
#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace scenario::gazebo::utils {

    std::shared_ptr<sdf::Root>
    getSdfRootFromFile(const std::string& sdfFileName);
    std::shared_ptr<sdf::Root>
    getSdfRootFromString(const std::string& sdfString);

    const std::string ScenarioVerboseEnvVar = "SCENARIO_VERBOSE";
    bool verboseFromEnvironment();

    std::chrono::steady_clock::duration
    doubleToSteadyClockDuration(const double durationInSeconds);

    double steadyClockDurationToDouble(
        const std::chrono::steady_clock::duration duration);

    void rowMajorToColumnMajor(std::vector<double>& input,
                               const long rows,
                               const long cols);

    class FixedSizeQueue
    {
    public:
        FixedSizeQueue(const size_t size = 100)
            : m_size(size)
            , m_deque(size, 0.0)
        {}

        void push(const double value)
        {
            if (m_deque.size() == m_size) {
                m_deque.pop_front();
            }

            m_deque.push_back(value);
        }

        void resize(const size_t newSize)
        {
            m_size = newSize;
            m_deque = std::deque<double>(newSize, 0.0);
        }

        inline std::vector<double> toStdVector() const
        {
            return {m_deque.begin(), m_deque.end()};
        }

    private:
        size_t m_size;
        std::deque<double> m_deque;
    };

    template <typename ComponentTypeT, typename ComponentDataTypeT>
    auto getComponent(ignition::gazebo::EntityComponentManager* ecm,
                      const ignition::gazebo::Entity entity,
                      ComponentDataTypeT defaultValue = {});

    template <typename ComponentTypeT>
    auto getExistingComponent(ignition::gazebo::EntityComponentManager* ecm,
                              const ignition::gazebo::Entity entity);

    template <typename ComponentTypeT>
    auto getComponentData(ignition::gazebo::EntityComponentManager* ecm,
                          const ignition::gazebo::Entity entity)
        -> decltype(ComponentTypeT().Data());

    template <typename ComponentTypeT>
    auto getExistingComponentData(ignition::gazebo::EntityComponentManager* ecm,
                                  const ignition::gazebo::Entity entity)
        -> decltype(ComponentTypeT().Data());

    scenario::core::Pose
    fromIgnitionPose(const ignition::math::Pose3d& ignitionPose);

    ignition::math::Pose3d toIgnitionPose(const scenario::core::Pose& pose);

    scenario::core::Contact
    fromIgnitionContactMsgs(ignition::gazebo::EntityComponentManager* ecm,
                            const ignition::msgs::Contact& contactMsg);

    std::vector<scenario::core::Contact>
    fromIgnitionContactsMsgs(ignition::gazebo::EntityComponentManager* ecm,
                             const ignition::msgs::Contacts& contactsMsg);

    sdf::World renameSDFWorld(const sdf::World& world,
                              const std::string& newWorldName);

    bool renameSDFModel(sdf::Root& sdfRoot,
                        const std::string& newModelName,
                        const size_t modelIndex = 0);

    bool updateSDFPhysics(sdf::Root& sdfRoot,
                          const double maxStepSize,
                          const double rtf,
                          const double realTimeUpdateRate,
                          const size_t worldIndex = 0);

    sdf::ElementPtr getPluginSDFElement(const std::string& libName,
                                        const std::string& className);

    sdf::JointType toSdf(const scenario::core::JointType type);
    scenario::core::JointType fromSdf(const sdf::JointType sdfType);

    std::pair<ignition::math::Vector3d, ignition::math::Vector3d>
    fromModelToBaseVelocity(const ignition::math::Vector3d& linModelVelocity,
                            const ignition::math::Vector3d& angModelVelocity,
                            const ignition::math::Pose3d& M_H_B,
                            const ignition::math::Quaterniond& W_R_B);

    std::pair<ignition::math::Vector3d, ignition::math::Vector3d>
    fromBaseToModelVelocity(const ignition::math::Vector3d& linBaseVelocity,
                            const ignition::math::Vector3d& angBaseVelocity,
                            const ignition::math::Pose3d& M_H_B,
                            const ignition::math::Quaterniond& W_R_B);

    std::shared_ptr<World> getParentWorld(const GazeboEntity& gazeboEntity);

    std::shared_ptr<Model> getParentModel(const GazeboEntity& gazeboEntity);

    template <typename ComponentType>
    ignition::gazebo::Entity getFirstParentEntityWithComponent(
        ignition::gazebo::EntityComponentManager* ecm,
        const ignition::gazebo::Entity entity)
    {
        ignition::gazebo::Entity candidateEntity = entity;

        auto hasComponent = [&]() -> bool {
            return ecm->EntityHasComponentType(candidateEntity,
                                               ComponentType::typeId);
        };

        auto isNull = [&]() -> bool {
            return candidateEntity == ignition::gazebo::kNullEntity;
        };

        while (!(hasComponent() || isNull())) {
            candidateEntity = ecm->ParentEntity(candidateEntity);
        }

        return candidateEntity;
    }

    template <typename T>
    auto defaultEqualityOperator(const T& a, const T& b) -> bool
    {
        return a == b;
    }

    template <typename ComponentTypeT, typename ComponentDataTypeT>
    auto setComponentData(
        ignition::gazebo::EntityComponentManager* ecm,
        const ignition::gazebo::Entity entity,
        const ComponentDataTypeT& data,
        const std::function<bool(const ComponentDataTypeT& a,
                                 const ComponentDataTypeT& b)>& eql =
            defaultEqualityOperator<ComponentDataTypeT>);

    template <typename ComponentTypeT, typename ComponentDataTypeT>
    auto setExistingComponentData(
        ignition::gazebo::EntityComponentManager* ecm,
        const ignition::gazebo::Entity entity,
        const ComponentDataTypeT& data,
        const std::function<bool(const ComponentDataTypeT& a,
                                 const ComponentDataTypeT& b)>& eql =
            defaultEqualityOperator<ComponentDataTypeT>);

    static inline std::array<double, 3>
    fromIgnitionVector(const ignition::math::Vector3d& ignitionVector)
    {
        return {ignitionVector.X(), ignitionVector.Y(), ignitionVector.Z()};
    }

    static inline std::array<double, 4> fromIgnitionQuaternion(
        const ignition::math::Quaterniond& ignitionQuaternion)
    {
        return {ignitionQuaternion.W(),
                ignitionQuaternion.X(),
                ignitionQuaternion.Y(),
                ignitionQuaternion.Z()};
    }

    static inline ignition::math::Vector3d
    toIgnitionVector3(const std::array<double, 3>& vector)
    {
        return {vector[0], vector[1], vector[2]};
    }

    static inline ignition::math::Vector4d
    toIgnitionVector4(const std::array<double, 4>& vector)
    {
        return {vector[0], vector[1], vector[2], vector[3]};
    }

    static inline ignition::math::Quaterniond
    toIgnitionQuaternion(const std::array<double, 4>& vector)
    {
        return {vector[0], vector[1], vector[2], vector[3]};
    }

    static inline ignition::math::PID
    toIgnitionPID(const scenario::core::PID& pid)
    {
        return {pid.p,
                pid.i,
                pid.d,
                pid.iMax,
                pid.iMin,
                pid.cmdMax,
                pid.cmdMin,
                pid.cmdOffset};
    }

    static inline scenario::core::PID
    fromIgnitionPID(const ignition::math::PID& pid)
    {
        scenario::core::PID pidScenario(pid.PGain(), pid.IGain(), pid.DGain());
        pidScenario.cmdMin = pid.CmdMin();
        pidScenario.cmdMax = pid.CmdMax();
        pidScenario.cmdOffset = pid.CmdOffset();
        pidScenario.iMin = pid.IMin();
        pidScenario.iMax = pid.IMax();

        return pidScenario;
    }

    class WrenchWithDuration
    {
    public:
        WrenchWithDuration(
            const ignition::msgs::Wrench& wrench,
            const std::chrono::steady_clock::duration& duration,
            const std::chrono::steady_clock::duration& curSimTime)
            : m_wrench(wrench)
            , m_expiration(curSimTime + duration)
        {}

        WrenchWithDuration(
            const ignition::math::Vector3d& force,
            const ignition::math::Vector3d& torque,
            const std::chrono::steady_clock::duration& duration,
            const std::chrono::steady_clock::duration& curSimTime)
            : m_expiration(curSimTime + duration)
        {
            ignition::msgs::Set(m_wrench.mutable_force(), force);
            ignition::msgs::Set(m_wrench.mutable_torque(), torque);
        }

        WrenchWithDuration(
            const std::array<double, 3>& force,
            const std::array<double, 3>& torque,
            const std::chrono::steady_clock::duration& duration,
            const std::chrono::steady_clock::duration& curSimTime)
            : WrenchWithDuration(toIgnitionVector3(force),
                                 toIgnitionVector3(torque),
                                 duration,
                                 curSimTime)
        {}

        inline ignition::msgs::Vector3d force() const
        {
            return m_wrench.force();
        }

        inline ignition::msgs::Vector3d torque() const
        {
            return m_wrench.torque();
        }

        inline std::chrono::steady_clock::duration expiration()
        {
            return m_expiration;
        }

        inline bool
        expired(const std::chrono::steady_clock::duration& curSimTime) const
        {
            return curSimTime >= m_expiration;
        }

    private:
        ignition::msgs::Wrench m_wrench;
        std::chrono::steady_clock::duration m_expiration;
    };

    class LinkWrenchCmd
    {
    public:
        LinkWrenchCmd() = default;

        inline void addWorldWrench(const WrenchWithDuration& wrench)
        {
            m_wrenches.push_back(wrench);
        }

        inline ignition::msgs::Wrench totalWrench() const
        {
            using namespace ignition;

            msgs::Wrench totalWrench;
            msgs::Set(totalWrench.mutable_force(), {0, 0, 0});
            msgs::Set(totalWrench.mutable_torque(), {0, 0, 0});

            for (const auto& wrench : m_wrenches) {
                const auto force = msgs::Convert(wrench.force());
                const auto torque = msgs::Convert(wrench.torque());
                const auto tmpForce = msgs::Convert(totalWrench.force());
                const auto tmpTorque = msgs::Convert(totalWrench.torque());

                msgs::Set(totalWrench.mutable_force(), tmpForce + force);
                msgs::Set(totalWrench.mutable_torque(), tmpTorque + torque);
            }

            return totalWrench;
        }

        inline void cleanExpired(const std::chrono::steady_clock::duration& now)
        {
            auto end = std::remove_if(
                m_wrenches.begin(),
                m_wrenches.end(),
                [&](const WrenchWithDuration& w) { return w.expired(now); });

            m_wrenches.erase(end, m_wrenches.end());
        }

    private:
        std::vector<WrenchWithDuration> m_wrenches;
    };
} // namespace scenario::gazebo::utils

template <typename ComponentTypeT, typename ComponentDataTypeT>
auto scenario::gazebo::utils::getComponent(
    ignition::gazebo::EntityComponentManager* ecm,
    const ignition::gazebo::Entity entity,
    ComponentDataTypeT defaultValue)
{
    if (!ecm) {
        throw std::runtime_error("ECM pointer not valid");
    }

    auto* component = ecm->Component<ComponentTypeT>(entity);

    if (!component) {
        ecm->CreateComponent(entity, ComponentTypeT(defaultValue));
        component = ecm->Component<ComponentTypeT>(entity);
    }

    return component;
}

template <typename ComponentTypeT>
auto scenario::gazebo::utils::getExistingComponent(
    ignition::gazebo::EntityComponentManager* ecm,
    const ignition::gazebo::Entity entity)
{
    if (!ecm) {
        throw std::runtime_error("ECM pointer not valid");
    }

    auto* component = ecm->Component<ComponentTypeT>(entity);

    if (!component) {
        throw exceptions::ComponentNotFound(ComponentTypeT::typeId, entity);
    }

    return component;
}

template <typename ComponentTypeT>
auto scenario::gazebo::utils::getComponentData(
    ignition::gazebo::EntityComponentManager* ecm,
    const ignition::gazebo::Entity entity) -> decltype(ComponentTypeT().Data())
{
    using ComponentDataType =
        typename std::remove_reference<decltype(ComponentTypeT().Data())>::type;

    auto component =
        getComponent<ComponentTypeT, ComponentDataType>(ecm, entity);

    return component->Data();
}

template <typename ComponentTypeT>
auto scenario::gazebo::utils::getExistingComponentData(
    ignition::gazebo::EntityComponentManager* ecm,
    const ignition::gazebo::Entity entity) -> decltype(ComponentTypeT().Data())
{
    auto component = getExistingComponent<ComponentTypeT>(ecm, entity);

    return component->Data();
}

template <typename ComponentTypeT, typename ComponentDataTypeT>
auto scenario::gazebo::utils::setComponentData(
    ignition::gazebo::EntityComponentManager* ecm,
    const ignition::gazebo::Entity entity,
    const ComponentDataTypeT& data,
    const std::function<bool(const ComponentDataTypeT&,
                             const ComponentDataTypeT&)>& eql)
{
    auto component =
        utils::getComponent<ComponentTypeT, ComponentDataTypeT>(ecm, entity);

    component->SetData(data, eql);
}

template <typename ComponentTypeT, typename ComponentDataTypeT>
auto scenario::gazebo::utils::setExistingComponentData(
    ignition::gazebo::EntityComponentManager* ecm,
    const ignition::gazebo::Entity entity,
    const ComponentDataTypeT& data,
    const std::function<bool(const ComponentDataTypeT&,
                             const ComponentDataTypeT&)>& eql)
{
    auto component = utils::getExistingComponent<ComponentTypeT>(ecm, entity);
    component->SetData(data, eql);
}

#endif // SCENARIO_GAZEBO_HELPERS_H
