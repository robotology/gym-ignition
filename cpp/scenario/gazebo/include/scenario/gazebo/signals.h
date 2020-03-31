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

#ifndef SCENARIO_GAZEBO_SIGNALS_H
#define SCENARIO_GAZEBO_SIGNALS_H

#include <functional>
#include <memory>

namespace scenario {
    namespace base {
        class SignalManager;
    } // namespace base
} // namespace scenario

class scenario::base::SignalManager
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

#endif // SCENARIO_GAZEBO_SIGNALS_H
