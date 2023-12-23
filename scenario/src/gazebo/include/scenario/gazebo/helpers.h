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

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/math/PID.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Vector4.hh>
#include <gz/msgs/Utility.hh>
#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/msgs/wrench.pb.h>
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

    bool parentModelJustCreated(const GazeboEntity& gazeboEntity);

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
    auto getComponent(gz::sim::EntityComponentManager* ecm,
                      const gz::sim::Entity entity,
                      ComponentDataTypeT defaultValue = {});

    template <typename ComponentTypeT>
    auto getExistingComponent(gz::sim::EntityComponentManager* ecm,
                              const gz::sim::Entity entity);

    template <typename ComponentTypeT>
    auto getComponentData(gz::sim::EntityComponentManager* ecm,
                          const gz::sim::Entity entity)
        -> decltype(ComponentTypeT().Data());

    template <typename ComponentTypeT>
    auto getExistingComponentData(gz::sim::EntityComponentManager* ecm,
                                  const gz::sim::Entity entity)
        -> decltype(ComponentTypeT().Data());

    scenario::core::Pose
    fromGzPose(const gz::math::Pose3d& gzPose);

    gz::math::Pose3d toGzPose(const scenario::core::Pose& pose);

    scenario::core::Contact
    fromGzContactMsgs(gz::sim::EntityComponentManager* ecm,
                            const gz::msgs::Contact& contactMsg);

    std::vector<scenario::core::Contact>
    fromGzContactsMsgs(gz::sim::EntityComponentManager* ecm,
                             const gz::msgs::Contacts& contactsMsg);

    sdf::World renameSDFWorld(const sdf::World& world,
                              const std::string& newWorldName);

    bool renameSDFModel(sdf::Root& sdfRoot, const std::string& newModelName);

    bool updateSDFPhysics(sdf::Root& sdfRoot,
                          const double maxStepSize,
                          const double rtf,
                          const double realTimeUpdateRate,
                          const size_t worldIndex = 0);

    sdf::ElementPtr getPluginSDFElement(const std::string& libName,
                                        const std::string& className);

    sdf::JointType toSdf(const scenario::core::JointType type);
    scenario::core::JointType fromSdf(const sdf::JointType sdfType);

    gz::math::Vector3d fromModelToBaseLinearVelocity(
        const gz::math::Vector3d& linModelVelocity,
        const gz::math::Vector3d& angModelVelocity,
        const gz::math::Pose3d& M_H_B,
        const gz::math::Quaterniond& W_R_B);

    gz::math::Vector3d fromBaseToModelLinearVelocity(
        const gz::math::Vector3d& linBaseVelocity,
        const gz::math::Vector3d& angBaseVelocity,
        const gz::math::Pose3d& M_H_B,
        const gz::math::Quaterniond& W_R_B);

    std::shared_ptr<World> getParentWorld(const GazeboEntity& gazeboEntity);

    std::shared_ptr<Model> getParentModel(const GazeboEntity& gazeboEntity);

    template <typename ComponentType>
    gz::sim::Entity getFirstParentEntityWithComponent(
        gz::sim::EntityComponentManager* ecm,
        const gz::sim::Entity entity)
    {
        gz::sim::Entity candidateEntity = entity;

        auto hasComponent = [&]() -> bool {
            return ecm->EntityHasComponentType(candidateEntity,
                                               ComponentType::typeId);
        };

        auto isNull = [&]() -> bool {
            return candidateEntity == gz::sim::kNullEntity;
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
        gz::sim::EntityComponentManager* ecm,
        const gz::sim::Entity entity,
        const ComponentDataTypeT& data,
        const std::function<bool(const ComponentDataTypeT& a,
                                 const ComponentDataTypeT& b)>& eql =
            defaultEqualityOperator<ComponentDataTypeT>);

    template <typename ComponentTypeT, typename ComponentDataTypeT>
    auto setExistingComponentData(
        gz::sim::EntityComponentManager* ecm,
        const gz::sim::Entity entity,
        const ComponentDataTypeT& data,
        const std::function<bool(const ComponentDataTypeT& a,
                                 const ComponentDataTypeT& b)>& eql =
            defaultEqualityOperator<ComponentDataTypeT>);

    static inline std::array<double, 3>
    fromGzVector(const gz::math::Vector3d& gzVector)
    {
        return {gzVector.X(), gzVector.Y(), gzVector.Z()};
    }

    static inline std::array<double, 4> fromGzQuaternion(
        const gz::math::Quaterniond& gzQuaternion)
    {
        return {gzQuaternion.W(),
                gzQuaternion.X(),
                gzQuaternion.Y(),
                gzQuaternion.Z()};
    }

    static inline gz::math::Vector3d
    toGzVector3(const std::array<double, 3>& vector)
    {
        return {vector[0], vector[1], vector[2]};
    }

    static inline gz::math::Vector4d
    toGzVector4(const std::array<double, 4>& vector)
    {
        return {vector[0], vector[1], vector[2], vector[3]};
    }

    static inline gz::math::Quaterniond
    toGzQuaternion(const std::array<double, 4>& vector)
    {
        return {vector[0], vector[1], vector[2], vector[3]};
    }

    static inline gz::math::PID
    toGzPID(const scenario::core::PID& pid)
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
    fromGzPID(const gz::math::PID& pid)
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
            const gz::msgs::Wrench& wrench,
            const std::chrono::steady_clock::duration& duration,
            const std::chrono::steady_clock::duration& curSimTime)
            : m_wrench(wrench)
            , m_expiration(curSimTime + duration)
        {}

        WrenchWithDuration(
            const gz::math::Vector3d& force,
            const gz::math::Vector3d& torque,
            const std::chrono::steady_clock::duration& duration,
            const std::chrono::steady_clock::duration& curSimTime)
            : m_expiration(curSimTime + duration)
        {
            gz::msgs::Set(m_wrench.mutable_force(), force);
            gz::msgs::Set(m_wrench.mutable_torque(), torque);
        }

        WrenchWithDuration(
            const std::array<double, 3>& force,
            const std::array<double, 3>& torque,
            const std::chrono::steady_clock::duration& duration,
            const std::chrono::steady_clock::duration& curSimTime)
            : WrenchWithDuration(toGzVector3(force),
                                 toGzVector3(torque),
                                 duration,
                                 curSimTime)
        {}

        const gz::msgs::Vector3d& force() const
        {
            return m_wrench.force();
        }

        const gz::msgs::Vector3d& torque() const
        {
            return m_wrench.torque();
        }

        const std::chrono::steady_clock::duration& expiration()
        {
            return m_expiration;
        }

        inline bool
        expired(const std::chrono::steady_clock::duration& curSimTime) const
        {
            return curSimTime >= m_expiration;
        }

    private:
        gz::msgs::Wrench m_wrench;
        std::chrono::steady_clock::duration m_expiration;
    };

    class LinkWrenchCmd
    {
    public:
        LinkWrenchCmd() = default;

        bool empty() const { return m_wrenches.empty(); }

        inline void addWorldWrench(const WrenchWithDuration& wrench)
        {
            m_wrenches.push_back(wrench);
        }

        inline gz::msgs::Wrench totalWrench() const
        {
            using namespace gz;

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

        void cleanExpired(const std::chrono::steady_clock::duration& now)
        {
            if (m_wrenches.empty())
                return;

            // Remove the expired wrenches
            m_wrenches.erase( //
                std::remove_if(m_wrenches.begin(),
                               m_wrenches.end(),
                               [&](const WrenchWithDuration& w) {
                                   return w.expired(now);
                               }),
                m_wrenches.end());
        }

    private:
        std::vector<WrenchWithDuration> m_wrenches;
    };
} // namespace scenario::gazebo::utils

template <typename ComponentTypeT, typename ComponentDataTypeT>
auto scenario::gazebo::utils::getComponent(
    gz::sim::EntityComponentManager* ecm,
    const gz::sim::Entity entity,
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
    gz::sim::EntityComponentManager* ecm,
    const gz::sim::Entity entity)
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
    gz::sim::EntityComponentManager* ecm,
    const gz::sim::Entity entity) -> decltype(ComponentTypeT().Data())
{
    using ComponentDataType =
        typename std::remove_reference<decltype(ComponentTypeT().Data())>::type;

    auto component =
        getComponent<ComponentTypeT, ComponentDataType>(ecm, entity);

    return component->Data();
}

template <typename ComponentTypeT>
auto scenario::gazebo::utils::getExistingComponentData(
    gz::sim::EntityComponentManager* ecm,
    const gz::sim::Entity entity) -> decltype(ComponentTypeT().Data())
{
    auto component = getExistingComponent<ComponentTypeT>(ecm, entity);

    return component->Data();
}

template <typename ComponentTypeT, typename ComponentDataTypeT>
auto scenario::gazebo::utils::setComponentData(
    gz::sim::EntityComponentManager* ecm,
    const gz::sim::Entity entity,
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
    gz::sim::EntityComponentManager* ecm,
    const gz::sim::Entity entity,
    const ComponentDataTypeT& data,
    const std::function<bool(const ComponentDataTypeT&,
                             const ComponentDataTypeT&)>& eql)
{
    auto component = utils::getExistingComponent<ComponentTypeT>(ecm, entity);
    component->SetData(data, eql);
}

#endif // SCENARIO_GAZEBO_HELPERS_H
