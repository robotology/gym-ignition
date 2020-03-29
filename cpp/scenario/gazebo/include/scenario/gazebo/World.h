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

#include "scenario/gazebo/Link.h"

#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace scenario {
    namespace gazebo {
        class World;
        class Model;
        using ModelPtr = std::shared_ptr<Model>;
        using WorldPtr = std::shared_ptr<World>;
    } // namespace gazebo
} // namespace scenario

class scenario::gazebo::World
{
public:
    World();
    virtual ~World();

    // ============
    // Gazebo World
    // ============

    uint64_t id() const;
    bool initialize(const ignition::gazebo::Entity worldEntity,
                    ignition::gazebo::EntityComponentManager* ecm,
                    ignition::gazebo::EventManager* eventManager);
    bool createECMResources();

    bool insertWorldPlugin(const std::string& libName,
                           const std::string& className,
                           const std::string& context = {});

    std::array<double, 3> gravity() const;
    bool setGravity(const std::array<double, 3>& gravity);

    // ==========
    // World Core
    // ==========

    double time() const;
    std::string name() const;
    std::vector<std::string> modelNames() const;
    scenario::gazebo::ModelPtr getModel(const std::string& modelName) const;

    bool insertModel(const std::string& modelFile,
                     const base::Pose& pose = base::Pose::Identity(),
                     const std::string& overrideModelName = {});
    bool removeModel(const std::string& modelName);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

#endif // SCENARIO_GAZEBO_WORLD_H
