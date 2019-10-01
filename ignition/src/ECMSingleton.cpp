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

class ECMSingleton::Impl
{
public:
    mutable std::mutex mutex;
    ignition::gazebo::EventManager* eventMgr = nullptr;
    ignition::gazebo::EntityComponentManager* ecm = nullptr;
};

ECMSingleton::ECMSingleton()
    : pImpl{new Impl(), [](Impl* impl) { delete impl; }}
{}

ECMSingleton& ECMSingleton::get()
{
    static ECMSingleton instance;
    return instance;
}

bool ECMSingleton::valid() const
{
    std::lock_guard(pImpl->mutex);
    return pImpl->ecm;
}

ignition::gazebo::EventManager* ECMSingleton::getEventManager() const
{
    std::lock_guard(pImpl->mutex);
    return pImpl->eventMgr;
}

bool ECMSingleton::storePtrs(ignition::gazebo::EntityComponentManager* ecm,
                             ignition::gazebo::EventManager* eventMgr)
{
    if (!ecm || !eventMgr) {
        gymppError << "The pointer to the ECM or EventManager is null" << std::endl;
        return false;
    }

    assert((pImpl->ecm && pImpl->eventMgr) || (!pImpl->ecm && !pImpl->eventMgr));

    if (pImpl->ecm) {
        gymppWarning << "The ECM has been stored previously. This call will do nothing."
                     << std::endl;
        return true;
    }

    if (pImpl->ecm && pImpl->eventMgr) {
        gymppWarning << "The ECM and EventManager are already stored. This call will do nothing."
                     << std::endl;
        return true;
    }

    gymppDebug << "Storing the ECM and the EventManager in the singleton" << std::endl;
    {
        std::lock_guard(pImpl->mutex);
        pImpl->ecm = ecm;
        pImpl->eventMgr = eventMgr;
    }

    return true;
}

ignition::gazebo::EntityComponentManager* ECMSingleton::getECM() const
{
    std::lock_guard(pImpl->mutex);

    if (!pImpl->ecm) {
        gymppError << "The ECM was never store in the singleton" << std::endl;
        return nullptr;
    }

    return pImpl->ecm;
}
