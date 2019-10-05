/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "ECMProvider.h"
#include "gympp/Log.h"
#include "gympp/gazebo/ECMSingleton.h"

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

using namespace gympp::gazebo;
using namespace gympp::plugins;

class ECMProvider::Impl
{
public:
    std::string worldName;
};

ECMProvider::ECMProvider()
    : System()
    , pImpl{new Impl(), [](Impl* impl) { delete impl; }}
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

IGNITION_ADD_PLUGIN(gympp::plugins::ECMProvider,
                    gympp::plugins::ECMProvider::System,
                    gympp::plugins::ECMProvider::ISystemConfigure,
                    gympp::plugins::ECMProvider::ISystemPreUpdate)
