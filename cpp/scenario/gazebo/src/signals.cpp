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

#include "scenario/gazebo/signals.h"
#include "scenario/gazebo/Log.h"

#include <csignal>
#include <mutex>
#include <unordered_map>

namespace scenario::base::detail {
    static std::mutex SignalManagerMutex;
}

using namespace scenario::base;

class SignalManager::Impl
{
public:
    static std::string ToString(const int type);
    static std::unordered_map<SignalManager::SignalType, std::string>
        DefaultTypeToName;

    std::unordered_map<SignalType, SignalCallback> callbacks;
};

SignalManager::SignalManager()
    : pImpl{std::make_unique<Impl>()}
{}

SignalManager::~SignalManager() = default;

void SignalManager::ExecuteCallback(SignalType type)
{
    std::lock_guard lock(detail::SignalManagerMutex);

    gymppDebug << "Received interrupt signal " << Impl::ToString(type)
               << std::endl;
    auto callback = SignalManager::Instance().getCallback(type);

    if (callback) {
        gymppDebug << "Found signal callback" << std::endl;
        callback(type);
        return;
    }

    gymppDebug << "No callback found" << std::endl;
}

SignalManager& SignalManager::Instance()
{
    static SignalManager instance;
    return instance;
}

SignalManager::SignalCallback
SignalManager::getCallback(const SignalType type) const
{
    if (pImpl->callbacks.find(type) != pImpl->callbacks.end()) {
        return pImpl->callbacks.at(type);
    }

    return nullptr;
}

SignalManager::SignalCallback
SignalManager::setCallback(const SignalType type,
                           const SignalManager::SignalCallback& callback)
{
    SignalCallback oldCallback = this->getCallback(type);

    gymppDebug << "Setting callback for signal " << Impl::ToString(type)
               << std::endl;
    std::signal(type, SignalManager::ExecuteCallback);
    pImpl->callbacks[type] = callback;

    return oldCallback;
}

// ==============
// Implementation
// ==============

std::string SignalManager::Impl::ToString(const int type)
{
    if (DefaultTypeToName.find(type) != DefaultTypeToName.end()) {
        return DefaultTypeToName.at(type);
    }
    else {
        return std::to_string(type);
    }
}

std::unordered_map<SignalManager::SignalType, std::string>
    SignalManager::Impl::DefaultTypeToName = {
        {SIGABRT, "SIGABRT"},
        {SIGFPE, "SIGFPE"},
        {SIGILL, "SIGILL"},
        {SIGINT, "SIGINT"},
        {SIGSEGV, "SIGSEGV"},
        {SIGTERM, "SIGTERM"},
};
