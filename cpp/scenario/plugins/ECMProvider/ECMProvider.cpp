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
#include "scenario/gazebo/Log.h"
#include "scenario/plugins/gazebo/ECMSingleton.h"

#include <ignition/gazebo/Entity.hh>
#include <ignition/plugin/Register.hh>

using namespace scenario::plugins::gazebo;

class ECMProvider::Impl
{
public:
};

ECMProvider::ECMProvider()
    : System()
    , pImpl{std::make_unique<Impl>()}
{}

ECMProvider::~ECMProvider()
{
    gymppDebug << "Destroying the ECMProvider" << std::endl;
};

void ECMProvider::Configure(const ignition::gazebo::Entity& entity,
                            const std::shared_ptr<const sdf::Element>& /*sdf*/,
                            ignition::gazebo::EntityComponentManager& ecm,
                            ignition::gazebo::EventManager& eventMgr)
{
    if (ECMSingleton::get().valid()) {
        gymppWarning << "The ECM singleton has been already configured" << std::endl;
        return;
    }

    gymppDebug << "Storing ECM resources in the singleton" << std::endl;

    if (!ECMSingleton::get().storePtrs(&ecm, &eventMgr)) {
        gymppError << "Failed to store ECM in the singleton for entity [" << entity << "]"
                   << std::endl;
        return;
    }
}

IGNITION_ADD_PLUGIN(scenario::plugins::gazebo::ECMProvider,
                    scenario::plugins::gazebo::ECMProvider::System,
                    scenario::plugins::gazebo::ECMProvider::ISystemConfigure)
