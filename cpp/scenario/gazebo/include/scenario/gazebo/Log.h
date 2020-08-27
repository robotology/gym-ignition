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

#ifndef SCENARIO_GAZEBO_LOG
#define SCENARIO_GAZEBO_LOG

#ifndef SCENARIO_LOG_MACROS_DEFINED
#define SCENARIO_LOG_MACROS_DEFINED

#include <ignition/common/Console.hh>
#define sError ::ignition::common::Console::err(__FILE__, __LINE__)
#define sWarning ::ignition::common::Console::warn(__FILE__, __LINE__)
#define sMessage ::ignition::common::Console::msg(__FILE__, __LINE__)
#define sDebug ::ignition::common::Console::dbg(__FILE__, __LINE__)
#define sLog ::ignition::common::Console::log(__FILE__, __LINE__)

#endif // SCENARIO_LOG_MACROS_DEFINED

#endif // SCENARIO_GAZEBO_LOG
