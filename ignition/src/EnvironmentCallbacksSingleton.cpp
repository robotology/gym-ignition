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
#include <unordered_map>

using namespace gympp::gazebo;

class EnvironmentCallbacksSingleton::Impl
{
public:
    std::unordered_map<std::string, EnvironmentCallbacks*> callbacks;
};

EnvironmentCallbacksSingleton::EnvironmentCallbacksSingleton()
    : pImpl{new Impl(), [](Impl* impl) { delete impl; }}
{}

EnvironmentCallbacks* EnvironmentCallbacksSingleton::get(const std::string& label)
{
    if (pImpl->callbacks.find(label) == pImpl->callbacks.end()) {
        gymppError << "Failed to find environment callbacks labelled as '" << label << "'"
                   << std::endl;
        return nullptr;
    }

    assert(pImpl->callbacks.at(label));
    return pImpl->callbacks.at(label);
}

bool EnvironmentCallbacksSingleton::storeEnvironmentCallback(const std::string& label,
                                                             EnvironmentCallbacks* cb)
{
    if (!cb || label.empty()) {
        gymppError << "Trying to store invalid environment callbacks" << std::endl;
        return false;
    }

    if (pImpl->callbacks.find(label) != pImpl->callbacks.end()) {
        gymppError << "Environment callbacks with label '" << label
                   << "' have been already registered" << std::endl;
    }

    pImpl->callbacks.insert({label, cb});
    return true;
}
