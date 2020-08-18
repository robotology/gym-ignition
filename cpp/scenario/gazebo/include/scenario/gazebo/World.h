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

#ifndef SCENARIO_GAZEBO_WORLD_H
#define SCENARIO_GAZEBO_WORLD_H

#include "scenario/core/World.h"
#include "scenario/gazebo/GazeboEntity.h"

#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace scenario::gazebo {
    class World;
    enum class PhysicsEngine
    {
        Dart,
    };
} // namespace scenario::gazebo

class scenario::gazebo::World final
    : public scenario::core::World
    , public scenario::gazebo::GazeboEntity
    , public std::enable_shared_from_this<scenario::gazebo::World>
{
public:
    World();
    virtual ~World();

    // =============
    // Gazebo Entity
    // =============

    uint64_t id() const override;

    bool initialize(const ignition::gazebo::Entity worldEntity,
                    ignition::gazebo::EntityComponentManager* ecm,
                    ignition::gazebo::EventManager* eventManager) override;

    bool createECMResources() override;

    // ============
    // Gazebo World
    // ============

    /**
     * Insert a Ignition Gazebo plugin to the world.
     *
     * @param libName The library name of the plugin.
     * @param className The class name (or alias) of the plugin.
     * @param context Optional XML plugin context.
     * @return True for success, false otherwise.
     */
    bool insertWorldPlugin(const std::string& libName,
                           const std::string& className,
                           const std::string& context = {});

    /**
     * Set the physics engine of the world.
     *
     * By default, if the world file does not already contain a physics
     * plugin, no physics is loaded by default. This method allows to
     * insert in the simulator a plugin with one of the supported physics
     * engines.
     *
     * @param engine The desired physics engine.
     * @return True for success, false otherwise.
     */
    bool setPhysicsEngine(const PhysicsEngine engine);

    /**
     * Set the gravity of the world.
     *
     * @note This method must be called after setting the physics engine and
     * before performing any physics step.
     *
     * @param gravity The desired gravity vector.
     * @return True for success, false otherwise.
     */
    bool setGravity(const std::array<double, 3>& gravity);

    /**
     * Insert a model in the world.
     *
     * @param modelFile The path to the URDF or SDF file to load.
     * @param pose The optional initial pose of the model.
     * @param overrideModelName The optional name of the model. This is the name
     * used to get the model with ``World::getModel``.
     * @return True for success, false otherwise.
     *
     * @note The default pose and model name are those specified in the
     * robot description. If the pose is not specified, the identity is used.
     *
     * @warning In order to process the model insertion, a simulator step must
     * be executed. It could either be a paused or unpaused step.
     */
    bool insertModel(const std::string& modelFile,
                     const core::Pose& pose = core::Pose::Identity(),
                     const std::string& overrideModelName = {});

    /**
     * Remove a model from the world.
     *
     * @param modelName The name of the model to remove.
     * @return True for success, false otherwise.
     *
     * @warning In order to process the model removal, a simulator step must
     * be executed. It could either be a paused or unpaused step.
     */
    bool removeModel(const std::string& modelName);

    // ==========
    // World Core
    // ==========

    bool valid() const override;

    double time() const override;

    std::string name() const override;

    std::array<double, 3> gravity() const override;

    std::vector<std::string> modelNames() const override;

    scenario::core::ModelPtr
    getModel(const std::string& modelName) const override;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

#endif // SCENARIO_GAZEBO_WORLD_H
