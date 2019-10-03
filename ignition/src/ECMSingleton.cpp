/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "gympp/gazebo/ECMSingleton.h"
#include "gympp/Log.h"

#include <mutex>
#include <ostream>
#include <unordered_map>

using namespace gympp::gazebo;
using RobotName = std::string;

struct Pointers
{
    ignition::gazebo::EventManager* eventMgr = nullptr;
    ignition::gazebo::EntityComponentManager* ecm = nullptr;
};

class ECMSingleton::Impl
{
public:
    using WorldName = std::string;
    mutable std::mutex mutex;
    std::unordered_map<WorldName, Pointers> resources;
};

ECMSingleton::ECMSingleton()
    : pImpl{new Impl(), [](Impl* impl) { delete impl; }}
{}

ECMSingleton& ECMSingleton::get()
{
    static ECMSingleton instance;
    return instance;
}

bool ECMSingleton::valid(const std::string& worldName) const
{
    std::lock_guard(pImpl->mutex);
    return exist(worldName) && pImpl->resources.at(worldName).ecm
           && pImpl->resources.at(worldName).eventMgr;
}

bool ECMSingleton::exist(const std::string& worldName) const
{
    if (pImpl->resources.find(worldName) != pImpl->resources.end()) {
        return true;
    }
    else {
        return false;
    }
}

ignition::gazebo::EventManager* ECMSingleton::getEventManager(const std::string& worldName) const
{
    std::lock_guard(pImpl->mutex);

    if (!exist(worldName)) {
        gymppError << "The event manager was never stored" << std::endl;
        return nullptr;
    }

    if (!valid(worldName)) {
        gymppError << "The pointers are not valid" << std::endl;
        return nullptr;
    }

    return pImpl->resources.at(worldName).eventMgr;
}

bool ECMSingleton::storePtrs(const std::string& worldName,
                             ignition::gazebo::EntityComponentManager* ecm,
                             ignition::gazebo::EventManager* eventMgr)
{
    if (!ecm || !eventMgr) {
        gymppError << "The pointer to the ECM or EventManager is null" << std::endl;
        return false;
    }

    if (exist(worldName)) {
        gymppWarning << "The pointers for world '" << worldName << "' have already been stored."
                     << " This method will do nothing" << std::endl;
        return true;
    }

    gymppDebug << "Storing the ECM and the EventManager in the singleton" << std::endl;
    {
        std::lock_guard(pImpl->mutex);
        pImpl->resources[worldName].ecm = ecm;
        pImpl->resources[worldName].eventMgr = eventMgr;
    }

    return true;
}

ignition::gazebo::EntityComponentManager* ECMSingleton::getECM(const std::string& worldName) const
{
    std::lock_guard(pImpl->mutex);

    if (!exist(worldName)) {
        gymppError << "The ECM of world '" << worldName << "' was never stored" << std::endl;
        return nullptr;
    }

    if (!valid(worldName)) {
        gymppError << "The pointers are not valid" << std::endl;
        return nullptr;
    }

    return pImpl->resources.at(worldName).ecm;
}

void ECMSingleton::clean(const std::string& worldName)
{
    pImpl->resources.erase(worldName);
}
