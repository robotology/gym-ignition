/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef SCENARIO_CORE_UTILS_SIGNALS_H
#define SCENARIO_CORE_UTILS_SIGNALS_H

#include <functional>
#include <memory>

namespace scenario::core::utils {
    class SignalManager;
} // namespace scenario::core::utils

class scenario::core::utils::SignalManager
{
public:
    using SignalType = int;
    using SignalCallback = std::function<void(int)>;

    SignalManager();
    ~SignalManager();

    static void ExecuteCallback(SignalType type);

    static SignalManager& Instance();
    SignalCallback getCallback(const SignalType type) const;
    SignalCallback setCallback(const SignalType type,
                               const SignalCallback& callback);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

#endif // SCENARIO_CORE_UTILS_SIGNALS_H
