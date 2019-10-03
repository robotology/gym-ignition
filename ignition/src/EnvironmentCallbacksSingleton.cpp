/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "gympp/gazebo/EnvironmentCallbacksSingleton.h"
#include "gympp/Log.h"
#include "gympp/gazebo/EnvironmentCallbacks.h"

#include <cassert>
#include <ostream>

using namespace gympp::gazebo;

std::unordered_map<std::string, EnvironmentCallbacks*>
    gympp::gazebo::EnvironmentCallbacksSingleton::m_callbacks = {};

EnvironmentCallbacks* EnvironmentCallbacksSingleton::get(const std::string& label)
{
    if (m_callbacks.find(label) == m_callbacks.end()) {
        gymppError << "Failed to find environment callbacks labelled as '" << label << "'"
                   << std::endl;
        return nullptr;
    }

    assert(m_callbacks.at(label));
    return m_callbacks.at(label);
}

bool EnvironmentCallbacksSingleton::storeEnvironmentCallback(const std::string& label,
                                                             EnvironmentCallbacks* cb)
{
    if (!cb || label.empty()) {
        gymppError << "Trying to store invalid environment callbacks" << std::endl;
        return false;
    }

    if (m_callbacks.find(label) != m_callbacks.end()) {
        gymppError << "Environment callbacks with label '" << label
                   << "' have been already registered" << std::endl;
    }

    gymppDebug << "Storing environment callback of task '" << label << "'" << std::endl;
    m_callbacks[label] = cb;

    return true;
}

bool EnvironmentCallbacksSingleton::deleteEnvironmentCallback(const std::string& label)
{
    if (label.empty()) {
        gymppError << "The label of the callbacks to delete is empty" << std::endl;
        return false;
    }

    if (m_callbacks.find(label) == m_callbacks.end()) {
        gymppError << "The callbacks '" << label << "' have never been stored" << std::endl;
        return false;
    }

    gymppDebug << "Deleting environment callback of task '" << label << "'" << std::endl;
    m_callbacks.erase(label);
    return true;
}
