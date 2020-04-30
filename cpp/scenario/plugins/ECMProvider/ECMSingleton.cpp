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

#include "scenario/plugins/gazebo/ECMSingleton.h"
#include "scenario/gazebo/Log.h"

#include <mutex>
#include <ostream>
#include <string>
#include <unordered_map>
#include <utility>

using namespace scenario::plugins::gazebo;

class ECMSingleton::Impl
{
public:
    struct ResourcePtrs
    {
        ResourcePtrs() = delete;
        ResourcePtrs(ignition::gazebo::EntityComponentManager* _ecm,
                     ignition::gazebo::EventManager* _eventMgr)
            : ecm(_ecm)
            , eventMgr(_eventMgr)
        {}

        ignition::gazebo::EntityComponentManager* ecm = nullptr;
        ignition::gazebo::EventManager* eventMgr = nullptr;
    };

    mutable std::recursive_mutex mutex;

    using WorldName = std::string;
    std::unordered_map<WorldName, ResourcePtrs> resources;
};

ECMSingleton::ECMSingleton()
    : pImpl{new Impl()}
{}

ECMSingleton::~ECMSingleton() = default;

ECMSingleton& ECMSingleton::Instance()
{
    static ECMSingleton instance;
    return instance;
}

void ECMSingleton::clean(const std::string& worldName)
{
    std::unique_lock lock(pImpl->mutex);

    if (worldName.empty()) {
        pImpl->resources.clear();
        return;
    }

    if (!this->hasWorld(worldName)) {
        sError << "Resources of world " << worldName << " not found"
               << std::endl;
        return;
    }

    pImpl->resources.erase(worldName);
}

bool ECMSingleton::valid(const std::string& worldName) const
{
    std::unique_lock lock(pImpl->mutex);

    if (!this->hasWorld(worldName)) {
        sDebug << "World" << worldName << " not found" << std::endl;
        return false;
    }

    if (!worldName.empty()) {
        const auto& ptrs = pImpl->resources.at(worldName);
        return ptrs.ecm && ptrs.eventMgr;
    }
    else {
        bool valid = true;
        for (const auto& [_, resources] : pImpl->resources) {
            valid = valid && resources.ecm && resources.eventMgr;
        }

        return valid;
    }
}

bool ECMSingleton::hasWorld(const std::string& worldName) const
{
    std::unique_lock lock(pImpl->mutex);

    if (worldName.empty()) {
        return pImpl->resources.size() != 0;
    }

    return pImpl->resources.find(worldName) != pImpl->resources.end();
}

std::vector<std::string> ECMSingleton::worldNames() const
{
    std::unique_lock lock(pImpl->mutex);
    std::vector<std::string> worldNames;

    for (const auto& [key, _] : pImpl->resources) {
        worldNames.emplace_back(key);
    }

    return worldNames;
}

ignition::gazebo::EventManager*
ECMSingleton::getEventManager(const std::string& worldName) const
{
    std::unique_lock lock(pImpl->mutex);

    if (!this->hasWorld(worldName)) {
        sError << "Resources of world " << worldName << " not found"
               << std::endl;
        return nullptr;
    }

    if (!this->valid(worldName)) {
        sError << "Resources of world " << worldName << " not valid"
               << std::endl;
        return nullptr;
    }

    return pImpl->resources.at(worldName).eventMgr;
}

ignition::gazebo::EntityComponentManager*
ECMSingleton::getECM(const std::string& worldName) const
{
    std::unique_lock lock(pImpl->mutex);

    if (!this->hasWorld(worldName)) {
        sError << "Resources of world " << worldName << " not found"
               << std::endl;
        return nullptr;
    }

    if (!this->valid(worldName)) {
        sError << "Resources of world " << worldName << " not valid"
               << std::endl;
        return nullptr;
    }

    return pImpl->resources.at(worldName).ecm;
}

bool ECMSingleton::storePtrs(ignition::gazebo::EntityComponentManager* ecm,
                             ignition::gazebo::EventManager* eventMgr,
                             const std::string& worldName)
{
    if (!ecm || !eventMgr) {
        sError << "The pointer to the ECM or EventManager is not valid"
               << std::endl;
        return false;
    }

    if (worldName.empty()) {
        sError << "The world name is empty" << std::endl;
        return false;
    }

    std::unique_lock lock(pImpl->mutex);

    if (this->hasWorld(worldName)) {
        sError << "Resources of world " << worldName
               << " have been already stored" << std::endl;
        return false;
    }

    pImpl->resources.emplace(worldName, Impl::ResourcePtrs(ecm, eventMgr));
    return true;
}
