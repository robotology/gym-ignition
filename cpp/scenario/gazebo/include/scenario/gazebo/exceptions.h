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

#ifndef SCENARIO_GAZEBO_EXCEPTIONS_H
#define SCENARIO_GAZEBO_EXCEPTIONS_H

#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/Types.hh>
#include <ignition/gazebo/components/Factory.hh>

#include <cstring>
#include <stdexcept>
#include <string>

namespace scenario::gazebo {
    namespace exceptions {
        class LinkError;
        class JointError;
        class ModelError;
        class DOFMismatch;
        class LinkNotFound;
        class JointNotFound;
        class ModelNotFound;
        class ComponentNotFound;
        class NotImplementedError;
    } // namespace exceptions
} // namespace scenario::gazebo

class scenario::gazebo::exceptions::LinkError : public std::runtime_error
{
private:
    std::string linkName;

public:
    explicit LinkError(const std::string& msg, const std::string& linkName = {})
        : std::runtime_error(msg)
        , linkName(linkName)
    {}

    const char* what() const noexcept override
    {
        std::string prefix;

        if (!linkName.empty()) {
            prefix = "[" + linkName + "] ";
        }

        std::string what = prefix + std::runtime_error::what();

        char* ptr = new char[what.size() + 1];
        strcpy(ptr, what.c_str());

        return ptr;
    }
};

class scenario::gazebo::exceptions::JointError : public std::runtime_error
{
private:
    std::string jointName;

public:
    explicit JointError(const std::string& msg,
                        const std::string& jointName = {})
        : std::runtime_error(msg)
        , jointName(jointName)
    {}

    const char* what() const noexcept override
    {
        std::string prefix;

        if (!jointName.empty()) {
            prefix = "[" + jointName + "] ";
        }

        std::string what = prefix + std::runtime_error::what();

        char* ptr = new char[what.size() + 1];
        strcpy(ptr, what.c_str());

        return ptr;
    }
};

class scenario::gazebo::exceptions::ModelError : public std::runtime_error
{
private:
    std::string modelName;

public:
    explicit ModelError(const std::string& msg,
                        const std::string& modelName = {})
        : std::runtime_error(msg)
        , modelName(modelName)
    {}

    const char* what() const noexcept override
    {
        std::string prefix;

        if (!modelName.empty()) {
            prefix = "[" + modelName + "] ";
        }

        std::string what = prefix + std::runtime_error::what();

        char* ptr = new char[what.size() + 1];
        strcpy(ptr, what.c_str());

        return ptr;
    }
};

class scenario::gazebo::exceptions::DOFMismatch : public std::runtime_error
{
private:
    size_t dataDoFs;
    size_t jointDoFs;
    std::string jointName;

public:
    explicit DOFMismatch(const size_t jointDoFs,
                         const size_t dataDoFs,
                         const std::string& jointName = {})
        : std::runtime_error("")
        , jointDoFs(jointDoFs)
        , dataDoFs(dataDoFs)
        , jointName(jointName)
    {}

    const char* what() const noexcept override
    {
        std::string prefix;

        if (!jointName.empty()) {
            prefix = "[" + jointName + "] ";
        }

        std::string what = prefix
                           + "Nr of DoFs joint=" + std::to_string(jointDoFs)
                           + " data=" + std::to_string(dataDoFs);

        char* ptr = new char[what.size() + 1];
        strcpy(ptr, what.c_str());

        return ptr;
    }
};

class scenario::gazebo::exceptions::LinkNotFound : public std::runtime_error
{
public:
    explicit LinkNotFound(const std::string& error)
        : std::runtime_error(error)
    {}

    const char* what() const noexcept override
    {
        std::string linkName = std::runtime_error::what();
        std::string prefix = "[" + linkName + "] ";
        std::string what = prefix + "Link does not exist";

        char* ptr = new char[what.size() + 1];
        strcpy(ptr, what.c_str());

        return ptr;
    }
};

class scenario::gazebo::exceptions::JointNotFound : public std::runtime_error
{
public:
    explicit JointNotFound(const std::string& error)
        : std::runtime_error(error)
    {}

    const char* what() const noexcept override
    {
        std::string jointName = std::runtime_error::what();
        std::string prefix = "[" + jointName + "] ";
        std::string what = prefix + "Joint does not exist";

        char* ptr = new char[what.size() + 1];
        strcpy(ptr, what.c_str());

        return ptr;
    }
};

class scenario::gazebo::exceptions::ModelNotFound : public std::runtime_error
{
public:
    explicit ModelNotFound(const std::string& error)
        : std::runtime_error(error)
    {}

    const char* what() const noexcept override
    {
        std::string modelName = std::runtime_error::what();
        std::string prefix = "[" + modelName + "] ";
        std::string what = prefix + "Model does not exist";

        char* ptr = new char[what.size() + 1];
        strcpy(ptr, what.c_str());

        return ptr;
    }
};

class scenario::gazebo::exceptions::NotImplementedError
    : public std::logic_error
{
public:
    explicit NotImplementedError(const std::string& symbol)
        : std::logic_error(symbol)
    {}

    const char* what() const noexcept override
    {
        std::string symbol = std::logic_error::what();
        std::string what = "Symbol not implemented: " + symbol;

        char* ptr = new char[what.size() + 1];
        strcpy(ptr, what.c_str());

        return ptr;
    }
};

class scenario::gazebo::exceptions::ComponentNotFound
    : public std::runtime_error
{
    ignition::gazebo::Entity entity;
    ignition::gazebo::ComponentTypeId id;

public:
    explicit ComponentNotFound(
        const ignition::gazebo::ComponentTypeId id,
        const ignition::gazebo::Entity entity = ignition::gazebo::kNullEntity)
        : std::runtime_error("")
        , entity(entity)
        , id(id)
    {}

    const char* what() const noexcept override
    {
        std::string prefix;

        if (entity != ignition::gazebo::kNullEntity) {
            prefix = "[Entity=" + std::to_string(entity) + "] ";
        }

        // Get the name of the component from the factory
        std::string componentName =
            ignition::gazebo::components::Factory::Instance()->Name(id);

        // Build the message
        std::string what = prefix + "Component not found: " + componentName;

        char* ptr = new char[what.size() + 1];
        strcpy(ptr, what.c_str());

        return ptr;
    }
};

#endif // SCENARIO_GAZEBO_EXCEPTIONS_H
