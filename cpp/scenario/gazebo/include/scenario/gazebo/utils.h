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
#include <vector>

#ifdef NDEBUG
#define DEFAULT_VERBOSITY 2
#else
#define DEFAULT_VERBOSITY 4
#endif

namespace scenario {
    namespace gazebo {
        namespace utils {
            /**
             * Set the verbosity process-wise.
             *
             * Accepted levels are the following:
             *
             * - ``<= 0``: No messages.
             * - ``1``: Error messages.
             * - ``2``: Error and warning messages.
             * - ``3``: Error, warning, and info messages.
             * - ``>= 4``: Error, warning, info, and debug messages.
             *
             * If called without specifying the level, it will use
             * level 2 or level 4 depending if the project was compiled
             * respectively with Release or Debug flags.
             *
             * @param level The verbosity level.
             */
            void setVerbosity(const int level = DEFAULT_VERBOSITY);

            /**
             * Find a SDF file in the filesystem.
             *
             * The search path is defined with the ``IGN_GAZEBO_RESOURCE_PATH``
             * environment variable.
             *
             * @param fileName The SDF file name.
             * @return The absolute path to the file if found, an empty string
             *         otherwise.
             */
            std::string findSdfFile(const std::string& fileName);

            /**
             * Check if a SDF string is valid.
             *
             * An SDF string could contain for instance an SDF model or
             * an SDF world, and it is valid if it can be parsed successfully
             * by the SDFormat library.
             *
             * @param sdfString The SDF string to check.
             * @return True if the SDF string is valid, false otherwise.
             */
            bool sdfStringValid(const std::string& sdfString);

            /**
             * Get an SDF string from a SDF file.
             *
             * @param fileName An SDF file. It could be either an absolute path
             *        to the file or the file name if the parent folder is part
             *        of the ``IGN_GAZEBO_RESOURCE_PATH`` environment variable.
             * @return The SDF string if the file was found and is valid, an
             *         empty string otherwise.
             */
            std::string getSdfString(const std::string& fileName);

            /**
             * Get the name of a model from a SDF file.
             *
             * @param fileName An SDF file. It could be either an absolute path
             *        to the file or the file name if the parent folder is part
             *        of the ``IGN_GAZEBO_RESOURCE_PATH`` environment variable.
             * @param modelIndex The index of the model in the SDF file. By
             *        default it finds the first model.
             * @return The name of the model.
             */
            std::string getModelNameFromSdf(const std::string& fileName,
                                            const size_t modelIndex = 0);

            /**
             * Get the name of a world from a SDF file.
             *
             * @param fileName An SDF file. It could be either an absolute path
             *        to the file or the file name if the parent folder is part
             *        of the ``IGN_GAZEBO_RESOURCE_PATH`` environment variable.
             * @param worldIndex The index of the world in the SDF file. By
             *        default it finds the first world.
             * @return The name of the world.
             */
            std::string getWorldNameFromSdf(const std::string& fileName,
                                            const size_t worldIndex = 0);

            /**
             * Return a SDF string with an empty world.
             *
             * An empty world only has a sun and the default DART
             * physics profile enabled.
             *
             * @note The empty world does not have any ground plane.
             *
             * @return A SDF string with the empty world.
             */
            std::string getEmptyWorld();

            /**
             * Get a SDF model file from a Fuel URI.
             *
             * A valid URI has the following form:
             *
             * ``https://fuel.ignitionrobotics.org/openrobotics/models/model_name``
             *
             * @param URI A valid Fuel URI.
             * @param useCache Load the model from the local cache.
             * @return The absolute path to the SDF model.
             */
            std::string getModelFileFromFuel(const std::string& URI,
                                             const bool useCache = false);

            /**
             * Generate a random alpha numeric string.
             *
             * @param length The length of the string.
             * @return The random string.
             */
            std::string getRandomString(const size_t length);

            /**
             * Get the install prefix used by the CMake project.
             *
             * @note It is defined only if the project is installed in
             * Developer mode.
             *
             * @return A string with the install prefix if the project is
             *         installed in Developer mode, an empty string otherwise.
             */
            std::string getInstallPrefix();

            /**
             * Convert a URDF file to a SDF string.
             *
             * @param urdfFile The absolute path to the URDF file.
             * @return The SDF string if the file exists and it was successfully
             *         converted, an empty string otherwise.
             */
            std::string URDFFileToSDFString(const std::string& urdfFile);

            /**
             * Convert a URDF string to a SDF string.
             *
             * @param urdfFile A URDF string.
             * @return The SDF string if the URDF string was successfully
             *         converted, an empty string otherwise.
             */
            std::string URDFStringToSDFString(const std::string& urdfString);

            /**
             * Normalize a vector in [-1, 1].
             *
             * The normalization applies the following equation, where
             * \f$ v \f$ is the input, \f$ l \f$ and \f$ h \f$ are respectively
             * the lower and higher limits:
             *
             * \f$ v_{normalized} = 2 \frac{v - l}{h - l} - 1 \f$
             *
             * The input, low and high arguments are broadcasted to a common
             * size. Refer to the following for broadcasting definition:
             *
             * https://numpy.org/doc/stable/user/basics.broadcasting.html
             *
             * @note If the lower limit matches the higher limit, the
             * corresponding input value is not normalized.
             *
             * @throws std::invalid_argument If the arguments cannot be
             * broadcasted.
             * @param input The input vector.
             * @param low The lower limit.
             * @param high The higher limit.
             * @return The normalized input.
             */
            std::vector<double> normalize(const std::vector<double>& input,
                                          const std::vector<double>& low,
                                          const std::vector<double>& high);

            /**
             * Denormalize a vector from [-1, 1].
             *
             * The denormalization applies the following equation, where
             * \f$ v \f$ is the input, \f$ l \f$ and \f$ h \f$ are respectively
             * the lower and higher limits:
             *
             * \f$ v_{denormalized} = \frac{1}{2} (v + 1)(h - l) - l \f$
             *
             * The input, low and high arguments are broadcasted to a common
             * size. Refer to the following for broadcasting definition:
             *
             * https://numpy.org/doc/stable/user/basics.broadcasting.html
             *
             * @throws std::invalid_argument If the arguments cannot be
             * broadcasted.
             * @param input The input vector.
             * @param low The lower limit.
             * @param high The higher limit.
             * @return The denormalized input.
             */
            std::vector<double> denormalize(const std::vector<double>& input,
                                            const std::vector<double>& low,
                                            const std::vector<double>& high);
        } // namespace utils
    } // namespace gazebo
} // namespace scenario

#endif // SCENARIO_GAZEBO_UTILS_H
