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
