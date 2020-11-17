/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef SCENARIO_CORE_UTILS_LOG
#define SCENARIO_CORE_UTILS_LOG

// Downstream implementations can override logs and then
// use the include order to select the right log backend
#ifndef SCENARIO_LOG_MACROS_DEFINED
#define SCENARIO_LOG_MACROS_DEFINED

#include <iostream>
#define sError std::cerr
#define sWarning std::cerr
#define sMessage std::cout
#define sDebug std::cout
#define sLog std::cout

#endif // SCENARIO_LOG_MACROS_DEFINED

#endif // SCENARIO_CORE_UTILS_LOG
