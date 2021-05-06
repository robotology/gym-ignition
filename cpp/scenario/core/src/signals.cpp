/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "scenario/core/utils/signals.h"
#include "scenario/core/utils/Log.h"

#include <csignal>
#include <mutex>
#include <unordered_map>

namespace scenario::core::detail {
    static std::mutex SignalManagerMutex;
}

using namespace scenario::core::utils;

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

    sDebug << "Received interrupt signal " << Impl::ToString(type) << std::endl;
    auto callback = SignalManager::Instance().getCallback(type);

    if (callback) {
        sDebug << "Found signal callback" << std::endl;
        callback(type);
        return;
    }

    sDebug << "No callback found" << std::endl;
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
