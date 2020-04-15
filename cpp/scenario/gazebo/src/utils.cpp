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

#include "scenario/gazebo/utils.h"
#include "scenario/gazebo/Log.h"
#include "scenario/gazebo/helpers.h"

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/common/URI.hh>
#include <ignition/fuel_tools/ClientConfig.hh>
#include <ignition/fuel_tools/FuelClient.hh>
#include <ignition/fuel_tools/Interface.hh>
#include <ignition/fuel_tools/Result.hh>
#include <ignition/gazebo/config.hh>
#include <sdf/Element.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <cassert>
#include <memory>

using namespace scenario::gazebo;

void utils::setVerbosity(const int level)
{
    ignition::common::Console::SetVerbosity(level);
}

std::string utils::findSdfFile(const std::string& fileName)
{
    if (fileName.empty()) {
        gymppError << "The SDF file name is empty" << std::endl;
        return {};
    }

    ignition::common::SystemPaths systemPaths;
    systemPaths.SetFilePathEnv("IGN_GAZEBO_RESOURCE_PATH");
    systemPaths.AddFilePaths(IGN_GAZEBO_WORLD_INSTALL_DIR);

    // Find the file
    std::string sdfFilePath = systemPaths.FindFile(fileName);

    if (sdfFilePath.empty()) {
        gymppError << "Failed to find " << fileName << std::endl;
        gymppError << "Check that it is part of IGN_GAZEBO_RESOURCE_PATH"
                   << std::endl;
        return {};
    }

    return sdfFilePath;
}

bool utils::sdfStringValid(const std::string& sdfString)
{
    return bool(getSdfRootFromString(sdfString));
}

std::string utils::getSdfString(const std::string& fileName)
{
    // NOTE: We could use std::filesystem for the following, but compilers
    //       support is still rough even with C++17 enabled :/
    std::string sdfFileAbsPath;

    if (!ignition::common::isFile(fileName)) {
        sdfFileAbsPath = findSdfFile(fileName);
    }

    if (sdfFileAbsPath.empty()) {
        return {};
    }

    auto root = getSdfRootFromString(sdfFileAbsPath);

    if (!root) {
        return {};
    }

    return root->Element()->ToString("");
}

std::string utils::getModelNameFromSdf(const std::string& fileName,
                                       const size_t modelIndex)
{
    auto root = utils::getSdfRootFromFile(fileName);

    if (!root) {
        return {};
    }

    if (root->ModelCount() == 0) {
        gymppError << "Didn't find any model in file " << fileName << std::endl;
        return {};
    }

    if (modelIndex >= root->ModelCount()) {
        gymppError << "Model with index " << modelIndex
                   << " not found. The model has only " << root->ModelCount()
                   << " model(s)" << std::endl;
        return {};
    }

    return root->ModelByIndex(modelIndex)->Name();
}

std::string utils::getWorldNameFromSdf(const std::string& fileName,
                                       const size_t worldIndex)
{
    auto root = utils::getSdfRootFromFile(fileName);

    if (!root) {
        return {};
    }

    if (root->WorldCount() == 0) {
        gymppError << "Didn't find any world in file " << fileName << std::endl;
        return {};
    }

    if (worldIndex >= root->WorldCount()) {
        gymppError << "Model with index " << worldIndex
                   << " not found. The model has only " << root->WorldCount()
                   << " model(s)" << std::endl;
        return {};
    }

    return root->WorldByIndex(worldIndex)->Name();
}

std::string utils::getEmptyWorld()
{
    const std::string world = R""""(<?xml version="1.0" ?>
<sdf version="1.6">
    <world name="default">
        <physics default="true" type="dart">
        </physics>
        <light type="directional" name="sun">
            <cast_shadows>true</cast_shadows>
            <pose>0 0 10 0 0 0</pose>
            <diffuse>1 1 1 1</diffuse>
            <specular>0.5 0.5 0.5 1</specular>
            <attenuation>
                <range>1000</range>
                <constant>0.9</constant>
                <linear>0.01</linear>
                <quadratic>0.001</quadratic>
            </attenuation>
            <direction>-0.5 0.1 -0.9</direction>
        </light>
    </world>
</sdf>)"""";

    assert(sdfStringValid(world));
    return world;
}

std::string utils::getModelFileFromFuel(const std::string& URI,
                                        const bool useCache)
{
    std::string modelFilePath;
    using namespace ignition;

    if (!useCache) {
        modelFilePath = fuel_tools::fetchResource(URI);

        if (modelFilePath.empty()) {
            gymppError << "Failed to download Fuel model" << std::endl;
            return {};
        }
    }
    else {
        fuel_tools::FuelClient client(fuel_tools::ClientConfig{});

        auto result = client.CachedModel(common::URI(URI), modelFilePath);

        if (result.Type() != fuel_tools::ResultType::FETCH_ALREADY_EXISTS) {
            gymppError << "Fuel model not found locally" << std::endl;
            return {};
        }
    }

    // NOTE: We could use std::filesystem for the following, but compilers
    //       support is still rough even with C++17 enabled :/
    std::string modelFile = common::joinPaths(modelFilePath, "model.sdf");

    if (!common::isFile(modelFile)) {
        gymppError << "The model was downloaded from Fuel but it was not found "
                   << "in the filesystem" << std::endl;
        return {};
    }

    return modelFile;
}

std::string utils::getRandomString(const size_t length)
{
    auto randchar = []() -> char {
        static constexpr char charset[] = "0123456789"
                                          "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                                          "abcdefghijklmnopqrstuvwxyz";
        const int max_index = (sizeof(charset) - 1);
        return charset[rand() % max_index];
    };

    std::string str(length, 0);
    std::generate_n(str.begin(), length, randchar);
    return str;
}
