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
#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

#include <ignition/plugin/Register.hh>

#include <cassert>
#include <chrono>
#include <ostream>
#include <string>
#include <unordered_map>

using namespace gympp::gazebo;
using namespace gympp::plugins;

ECMProvider::ECMProvider()
    : System()
{}

ECMProvider::~ECMProvider() = default;

void ECMProvider::Configure(const ignition::gazebo::Entity& /*entity*/,
                            const std::shared_ptr<const sdf::Element>& /*sdf*/,
                            ignition::gazebo::EntityComponentManager& ecm,
                            ignition::gazebo::EventManager& eventMgr)
{
    // Register the EntityComponentManager and the EventManager in the singleton
    if (!ECMSingleton::get().valid()) {
        if (!ECMSingleton::get().storePtrs(&ecm, &eventMgr)) {
            gymppError << "Failed to store ECM in the singleton" << std::endl;
            return;
        }
    }
}

IGNITION_ADD_PLUGIN(gympp::plugins::ECMProvider,
                    gympp::plugins::ECMProvider::System,
                    gympp::plugins::ECMProvider::ISystemConfigure)
