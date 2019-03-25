/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef GYMPP_LOG
#define GYMPP_LOG

#if defined(USE_IGNITION_LOGS)
#include <ignition/common/Console.hh>
#define gymppError ::ignition::common::Console::err(__FILE__, __LINE__)
#define gymppWarning ::ignition::common::Console::warn(__FILE__, __LINE__)
#define gymppMessage ::ignition::common::Console::msg(__FILE__, __LINE__)
#define gymppDebug ::ignition::common::Console::dbg(__FILE__, __LINE__)
#define gymppLog ::ignition::common::Console::log(__FILE__, __LINE__)
#else
#include <iostream>
#define gymppError std::cerr
#define gymppWarning std::cerr
#define gymppMessage std::cout
#define gymppDebug std::cout
#define gymppLog std::cout
#endif

#endif // GYMPP_LOG
