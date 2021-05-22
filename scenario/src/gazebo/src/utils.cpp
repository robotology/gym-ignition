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

#include <Eigen/Dense>
#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/common/URI.hh>
#include <ignition/fuel_tools/ClientConfig.hh>
#include <ignition/fuel_tools/FuelClient.hh>
#include <ignition/fuel_tools/Interface.hh>
#include <ignition/fuel_tools/Result.hh>
#include <ignition/gazebo/Events.hh>
#include <ignition/gazebo/config.hh>
#include <sdf/Element.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <cassert>
#include <exception>
#include <memory>

using namespace scenario::gazebo;

void utils::setVerbosity(const Verbosity level)
{
    ignition::common::Console::SetVerbosity(static_cast<int>(level));
}

std::string utils::findSdfFile(const std::string& fileName)
{
    if (fileName.empty()) {
        sError << "The SDF file name is empty" << std::endl;
        return {};
    }

    ignition::common::SystemPaths systemPaths;
    systemPaths.SetFilePathEnv("IGN_GAZEBO_RESOURCE_PATH");
    systemPaths.AddFilePaths(IGN_GAZEBO_WORLD_INSTALL_DIR);

    // Find the file
    std::string sdfFilePath = systemPaths.FindFile(fileName);

    if (sdfFilePath.empty()) {
        sError << "Failed to find " << fileName << std::endl;
        sError << "Check that it is part of IGN_GAZEBO_RESOURCE_PATH"
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

std::string utils::getModelNameFromSdf(const std::string& fileName)
{
    std::string absFileName = findSdfFile(fileName);

    if (absFileName.empty()) {
        sError << "Failed to find file " << fileName << std::endl;
        return {};
    }

    const auto root = utils::getSdfRootFromFile(absFileName);

    if (!root) {
        return {};
    }

    if (const auto model = root->Model()) {
        return model->Name();
    }

    sError << "No model found in file " << fileName << std::endl;
    return {};
}

std::string utils::getWorldNameFromSdf(const std::string& fileName,
                                       const size_t worldIndex)
{
    std::string absFileName = findSdfFile(fileName);

    if (absFileName.empty()) {
        sError << "Failed to find file " << fileName << std::endl;
        return {};
    }

    auto root = utils::getSdfRootFromFile(absFileName);

    if (!root) {
        return {};
    }

    if (root->WorldCount() == 0) {
        sError << "Didn't find any world in file " << fileName << std::endl;
        return {};
    }

    if (worldIndex >= root->WorldCount()) {
        sError << "Model with index " << worldIndex
               << " not found. The model has only " << root->WorldCount()
               << " model(s)" << std::endl;
        return {};
    }

    return root->WorldByIndex(worldIndex)->Name();
}

std::string utils::getEmptyWorld()
{
    // The empty world matches default.sdf from upstream without models
    const std::string world = R""""(<?xml version="1.0" ?>
<sdf version="1.6">
    <world name="default">
        <physics default="true" type="ignored">
        </physics>
        <light type="directional" name="sun">
            <cast_shadows>true</cast_shadows>
            <pose>0 0 10 0 0 0</pose>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
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
            sError << "Failed to download Fuel model" << std::endl;
            return {};
        }
    }
    else {
        fuel_tools::FuelClient client(fuel_tools::ClientConfig{});

        auto result = client.CachedModel(common::URI(URI), modelFilePath);

        if (result.Type() != fuel_tools::ResultType::FETCH_ALREADY_EXISTS) {
            sError << "Fuel model not found locally" << std::endl;
            return {};
        }
    }

    // NOTE: We could use std::filesystem for the following, but compilers
    //       support is still rough even with C++17 enabled :/
    std::string modelFile = common::joinPaths(modelFilePath, "model.sdf");

    if (!common::isFile(modelFile)) {
        sError << "The model was downloaded from Fuel but it was not found "
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

std::string utils::URDFFileToSDFString(const std::string& urdfFile)
{
    auto root = getSdfRootFromFile(urdfFile);

    if (!root) {
        return "";
    }

    return root->Element()->ToString("");
}

std::string utils::URDFStringToSDFString(const std::string& urdfString)
{
    auto root = getSdfRootFromString(urdfString);

    if (!root) {
        return "";
    }

    return root->Element()->ToString("");
}

std::vector<double> utils::normalize(const std::vector<double>& input,
                                     const std::vector<double>& low,
                                     const std::vector<double>& high)
{
    bool okInput = input.size() > 0;
    bool okLow = low.size() == input.size() || low.size() == 1;
    bool okHigh = high.size() == input.size() || high.size() == 1;

    if (!okInput || !okLow || !okHigh) {
        throw std::invalid_argument("Wrong input arguments");
    }

    std::vector<double> lowBroadcasted = low;
    std::vector<double> highBroadcasted = high;

    if (low.size() == 1 && input.size() > 1) {
        lowBroadcasted = std::vector<double>(input.size(), low[0]);
    }

    if (high.size() == 1 && input.size() > 1) {
        highBroadcasted = std::vector<double>(input.size(), high[0]);
    }

    std::vector<double> output;
    output.resize(input.size());

    auto inputEigen = Eigen::Map<Eigen::ArrayXd>( //
        const_cast<double*>(input.data()),
        input.size());
    auto outputEigen = Eigen::Map<Eigen::ArrayXd>( //
        output.data(),
        output.size());
    auto lowEigen = Eigen::Map<Eigen::ArrayXd>( //
        lowBroadcasted.data(),
        lowBroadcasted.size());
    auto highEigen = Eigen::Map<Eigen::ArrayXd>( //
        highBroadcasted.data(),
        highBroadcasted.size());

    if (highEigen.isApprox(lowEigen)) {
        return input;
    }

    outputEigen = 2.0 * (inputEigen - lowEigen) / (highEigen - lowEigen) - 1;

    // Replace infinite with the input value
    std::transform(output.begin(),
                   output.end(),
                   input.begin(),
                   output.begin(),
                   [](const double& output, const double& input) {
                       return std::isfinite(output) ? output : input;
                   });

    return output;
}

std::vector<double> utils::denormalize(const std::vector<double>& input,
                                       const std::vector<double>& low,
                                       const std::vector<double>& high)
{
    bool okInput = input.size() > 0;
    bool okLow = low.size() == input.size() || low.size() == 1;
    bool okHigh = high.size() == input.size() || high.size() == 1;

    if (!okInput || !okLow || !okHigh) {
        throw std::invalid_argument("Wrong input arguments");
    }

    std::vector<double> lowBroadcasted = low;
    std::vector<double> highBroadcasted = high;

    if (low.size() == 1 && input.size() > 1) {
        lowBroadcasted = std::vector<double>(input.size(), low[0]);
    }

    if (high.size() == 1 && input.size() > 1) {
        highBroadcasted = std::vector<double>(input.size(), high[0]);
    }

    std::vector<double> output;
    output.resize(input.size());

    auto inputEigen = Eigen::Map<Eigen::ArrayXd>( //
        const_cast<double*>(input.data()),
        input.size());
    auto outputEigen = Eigen::Map<Eigen::ArrayXd>( //
        output.data(),
        output.size());
    auto lowEigen = Eigen::Map<Eigen::ArrayXd>( //
        lowBroadcasted.data(),
        lowBroadcasted.size());
    auto highEigen = Eigen::Map<Eigen::ArrayXd>( //
        highBroadcasted.data(),
        highBroadcasted.size());

    if (highEigen.isApprox(lowEigen)) {
        return input;
    }

    outputEigen = (inputEigen + 1) * (highEigen - lowEigen) / 2.0 + lowEigen;

    return output;
}

bool utils::insertPluginToGazeboEntity(const GazeboEntity& gazeboEntity,
                                       const std::string& libName,
                                       const std::string& className,
                                       const std::string& context)
{
    if (!gazeboEntity.validEntity()) {
        sError << "The Gazebo Entity is not valid" << std::endl;
        return false;
    }

    if (libName.empty() || className.empty()) {
        sError << "Either the library name or the class name are empty strings"
               << std::endl;
        return false;
    }

    sLog << "Triggering plugin loading:" << std::endl;
    sLog << className << " from " << libName << " for entity ["
         << gazeboEntity.entity() << "]" << std::endl;

    // Create a new <plugin name="" filename=""> element without context
    sdf::ElementPtr pluginElement =
        utils::getPluginSDFElement(libName, className);

    // Insert the context into the plugin element
    if (!context.empty()) {

        std::shared_ptr<sdf::Root> contextRoot =
            utils::getSdfRootFromString(context);

        if (!contextRoot) {
            return false;
        }

        // Get the first element of the context
        // (stripping out the <sdf> container)
        auto contextNextElement = contextRoot->Element()->GetFirstElement();

        // Insert the plugin context elements
        while (contextNextElement) {
            pluginElement->InsertElement(contextNextElement);
            contextNextElement = contextNextElement->GetNextElement();
        }
    }

    // The plugin element must be wrapped in another element, otherwise
    // who receives it does not get the additional context
    const auto wrapped = sdf::SDF::WrapInRoot(pluginElement);

    // Trigger the plugin loading
    gazeboEntity.eventManager()->Emit<ignition::gazebo::events::LoadPlugins>(
        gazeboEntity.entity(), wrapped);

    return true;
}
