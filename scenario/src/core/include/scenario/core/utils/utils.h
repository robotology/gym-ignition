/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef SCENARIO_CORE_UTILS_H
#define SCENARIO_CORE_UTILS_H

#include <string>

namespace scenario::core::utils {
    /**
     * Get the install prefix used by the CMake project.
     *
     * @note It is defined only if the project is installed in
     * Developer mode.
     *
     * @return A string with the install prefix if the project is
     * installed in Developer mode, an empty string otherwise.
     */
    std::string getInstallPrefix();
} // namespace scenario::core::utils

#endif // SCENARIO_CORE_UTILS_H
