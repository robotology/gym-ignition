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

#ifndef SCENARIO_GAZEBO_UTILS_H
#define SCENARIO_GAZEBO_UTILS_H

#include <string>

#ifdef NDEBUG
#define DEFAULT_VERBOSITY 2
#else
#define DEFAULT_VERBOSITY 4
#endif

namespace scenario {
    namespace gazebo {
        namespace utils {
            void setVerbosity(const int level = DEFAULT_VERBOSITY);
            std::string findSdfFile(const std::string& fileName);
            bool sdfStringValid(const std::string& sdfString);
            std::string getSdfString(const std::string& fileName);
            std::string getModelNameFromSdf(const std::string& fileName,
                                            const size_t modelIndex = 0);
            std::string getWorldNameFromSdf(const std::string& fileName,
                                            const size_t worldIndex = 0);
            std::string getEmptyWorld();
            std::string getModelFileFromFuel(const std::string& URI,
                                             const bool useCache = false);
            std::string getRandomString(const size_t length);
        } // namespace utils
    } // namespace gazebo
} // namespace scenario

#endif // SCENARIO_GAZEBO_UTILS_H
