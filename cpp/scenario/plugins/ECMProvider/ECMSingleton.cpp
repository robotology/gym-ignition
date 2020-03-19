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

#include <atomic>
#include <ostream>
#include <string>

using namespace scenario::plugins::gazebo;

class ECMSingleton::Impl
{
public:
    std::atomic<ignition::gazebo::EventManager*> eventMgr = nullptr;
    std::atomic<ignition::gazebo::EntityComponentManager*> ecm = nullptr;
};

ECMSingleton::ECMSingleton()
    : pImpl{new Impl()}
{}

ECMSingleton& ECMSingleton::Instance()
{
    static ECMSingleton instance;
    return instance;
}

bool ECMSingleton::valid() const
{
    return pImpl->ecm && pImpl->eventMgr;
}

ignition::gazebo::EventManager* ECMSingleton::getEventManager() const
{
    if (!this->valid()) {
        gymppError << "The pointers are not valid" << std::endl;
        return nullptr;
    }

    return pImpl->eventMgr;
}

bool ECMSingleton::storePtrs(ignition::gazebo::EntityComponentManager* ecm,
                             ignition::gazebo::EventManager* eventMgr)
{
    if (!ecm || !eventMgr) {
        gymppError << "The pointer to the ECM or EventManager is not valid"
                   << std::endl;
        return false;
    }

    pImpl->ecm = ecm;
    pImpl->eventMgr = eventMgr;

    return true;
}

ignition::gazebo::EntityComponentManager* ECMSingleton::getECM() const
{

    if (!this->valid()) {
        gymppError << "The pointers are not valid" << std::endl;
        return nullptr;
    }

    return pImpl->ecm;
}

void ECMSingleton::clean()
{
    pImpl->ecm = nullptr;
    pImpl->eventMgr = nullptr;
}
