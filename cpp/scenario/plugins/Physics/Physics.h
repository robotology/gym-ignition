/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 * All rights reserved.
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

#ifndef SCENARIO_PLUGINS_GAZEBO_PHYSICS
#define SCENARIO_PLUGINS_GAZEBO_PHYSICS

#include <ignition/gazebo/System.hh>
#include <ignition/physics/FeatureList.hh>
#include <ignition/physics/RequestFeatures.hh>

#include <memory>
#include <unordered_map>
#include <utility>

namespace scenario::plugins::gazebo {
    class Physics;
    template <typename PolicyT,
              typename ToFeatureList,
              typename MinimumFeatureList,
              template <typename, typename>
              class ToEntity,
              template <typename, typename>
              class MinimumEntity>
    ignition::physics::EntityPtr<ToEntity<PolicyT, ToFeatureList>> entityCast(
        ignition::gazebo::Entity _entity,
        const ignition::physics::EntityPtr<
            MinimumEntity<PolicyT, MinimumFeatureList>>& _minimumEntity,
        std::unordered_map<
            ignition::gazebo::Entity,
            ignition::physics::EntityPtr<ToEntity<PolicyT, ToFeatureList>>>&
            _castMap);
} // namespace scenario::plugins::gazebo

class scenario::plugins::gazebo::Physics final
    : public ignition::gazebo::System
    , public ignition::gazebo::ISystemConfigure
    , public ignition::gazebo::ISystemUpdate
{
public:
    explicit Physics();
    ~Physics() override;

    // Documentation inherited
    void Configure(const ignition::gazebo::Entity& entity,
                   const std::shared_ptr<const sdf::Element>& sdf,
                   ignition::gazebo::EntityComponentManager& ecm,
                   ignition::gazebo::EventManager& eventMgr) final;

    void Update(const ignition::gazebo::UpdateInfo& info,
                ignition::gazebo::EntityComponentManager& ecm) override;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

/// \brief Helper function to cast from an entity type with minimum features
/// to an entity with a different set of features. When the entity is cast
/// successfully, it is added to _castMap so that subsequent casts will
/// use the entity from the map.
/// \tparam PolicyT The feature policy, such as
/// `ignition::physics::FeaturePolicy3d`.
/// \tparam ToFeatureList The list of features of the resulting entity.
/// \tparam MinimumFeatureList The minimum list of features.
/// \tparam ToEntity Type of entities with ToFeatureList
/// \tparam MinimumEntity Type of entities with MinimumFeatureList
/// \param[in] _entity Entity ID.
/// \param[in] _minimumEntity Entity pointer with minimum features.
/// \param[in] _castMap Map to store entities that have already been cast.
template <typename PolicyT,
          typename ToFeatureList,
          typename MinimumFeatureList,
          template <typename, typename>
          class ToEntity,
          template <typename, typename>
          class MinimumEntity>
ignition::physics::EntityPtr<ToEntity<PolicyT, ToFeatureList>>
scenario::plugins::gazebo::entityCast(
    ignition::gazebo::Entity _entity,
    const ignition::physics::EntityPtr<
        MinimumEntity<PolicyT, MinimumFeatureList>>& _minimumEntity,
    std::unordered_map<
        ignition::gazebo::Entity,
        ignition::physics::EntityPtr<ToEntity<PolicyT, ToFeatureList>>>&
        _castMap)
{
    // Has already been cast
    auto castIt = _castMap.find(_entity);
    if (castIt != _castMap.end()) {
        return castIt->second;
    }

    ignition::physics::EntityPtr<ToEntity<PolicyT, ToFeatureList>> castEntity;

    // Cast
    castEntity =
        ignition::physics::RequestFeatures<ToFeatureList>::From(_minimumEntity);

    if (castEntity) {
        _castMap.insert(std::make_pair(_entity, castEntity));
    }

    return castEntity;
}

#endif // SCENARIO_PLUGINS_GAZEBO_PHYSICS
