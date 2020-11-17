/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "scenario/core/utils/utils.h"
#include "scenario/core/utils/Log.h"

using namespace scenario::core;

std::string utils::getInstallPrefix()
{
#ifdef SCENARIO_CMAKE_INSTALL_PREFIX
    return SCENARIO_CMAKE_INSTALL_PREFIX;
#else
    // The install prefix of the User installation can be computed
    // from the Python module path
    return "";
#endif
}
