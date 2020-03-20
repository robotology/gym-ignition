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

#include "scenario/plugins/gazebo/ECMProvider.h"
#include "gympp/base/Log.h"
#include "scenario/plugins/gazebo/ECMSingleton.h"

#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/plugin/Register.hh>

#include <cassert>
#include <chrono>
#include <ostream>
#include <string>
#include <unordered_map>

using namespace scenario::plugins::gazebo;

class ECMProvider::Impl
{
public:
    std::string worldName;
};

ECMProvider::ECMProvider()
    : System()
    , pImpl{new Impl()}
{}

ECMProvider::~ECMProvider()
{
    gymppDebug << "Destroying the ECMProvider" << std::endl;
};

void ECMProvider::Configure(const ignition::gazebo::Entity& /*entity*/,
                            const std::shared_ptr<const sdf::Element>& /*sdf*/,
                            ignition::gazebo::EntityComponentManager& ecm,
                            ignition::gazebo::EventManager& eventMgr)
{
    auto worldEntities = ecm.EntitiesByComponents(ignition::gazebo::components::World());

    if (worldEntities.size() == 0) {
        gymppError << "Didn't find any world in the context of the ECMProvider plugin" << std::endl;
        assert(false);
        return;
    }

    assert(worldEntities.size() == 1);
    auto worldEntity = worldEntities[0];

    auto nameComponent = ecm.Component<ignition::gazebo::components::Name>(worldEntity);
    pImpl->worldName = nameComponent->Data();
    assert(!pImpl->worldName.empty());

    // Register the EntityComponentManager and the EventManager in the singleton
    if (!ECMSingleton::get().valid(pImpl->worldName)) {
        gymppDebug << "Inserting ECM for world '" << pImpl->worldName << "'" << std::endl;

        if (!ECMSingleton::get().storePtrs(pImpl->worldName, &ecm, &eventMgr)) {
            gymppError << "Failed to store ECM in the singleton for world '" << pImpl->worldName
                       << "'" << std::endl;
            return;
        }
    }
}

void ECMProvider::PreUpdate(const ignition::gazebo::UpdateInfo& info,
                            ignition::gazebo::EntityComponentManager& /*ecm*/)
{
    // Syncronize entity creation only when the simulation is paused
    if (info.paused) {
        ECMSingleton::get().notifyAndWaitPreUpdate(pImpl->worldName);
    }
}
IGNITION_ADD_PLUGIN(scenario::plugins::gazebo::ECMProvider,
                    scenario::plugins::gazebo::ECMProvider::System,
                    scenario::plugins::gazebo::ECMProvider::ISystemConfigure,
                    scenario::plugins::gazebo::ECMProvider::ISystemPreUpdate)
