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

#include "ControllersFactory.h"
#include "scenario/controllers/ComputedTorqueFixedBase.h"
#include "scenario/gazebo/Log.h"

#include <sdf/Param.hh>

#include <algorithm>
#include <array>
#include <cassert>
#include <istream>
#include <locale>
#include <string>
#include <utility>
#include <vector>

using namespace scenario::plugins::gazebo;

class ControllersFactory::Impl
{
public:
    static bool ContextValid(const sdf::ElementPtr context);

    template <typename T>
    static T GetElementValueAs(const std::string& elementName,
                               const sdf::ElementPtr parentContext);

    static void StringToStd(const std::string& string,
                            std::vector<double>& out);

    static void StringToStd(const std::string& string,
                            std::vector<std::string>& out);

    static void StringToStd(const std::string& string, std::string& out);
};

ControllersFactory::ControllersFactory()
    : pImpl{std::make_unique<Impl>()}
{}

ControllersFactory::~ControllersFactory() = default;

scenario::plugins::gazebo::ControllersFactory& ControllersFactory::Instance()
{
    static ControllersFactory instance;
    return instance;
}

scenario::controllers::ControllerPtr
ControllersFactory::get(const sdf::ElementPtr context, core::ModelPtr model)
{
    if (!Impl::ContextValid(context)) {
        sError << "Controller context not valid" << std::endl;
        return nullptr;
    }

    std::string controllerName;
    context->GetAttribute("name")->Get<std::string>(controllerName);
    sDebug << "Found context for " << controllerName << std::endl;

    if (controllerName == "ComputedTorqueFixedBase") {

        bool ok = true;
        ok = ok && context->HasElement("kp");
        ok = ok && context->HasElement("kd");
        ok = ok && context->HasElement("urdf");
        ok = ok && context->HasElement("joints");
        ok = ok && context->HasElement("gravity");

        if (!ok) {
            sError << "Controller context has missing elements" << std::endl;
            return nullptr;
        }

        auto urdf = Impl::GetElementValueAs<std::string>("urdf", context);
        auto kp = Impl::GetElementValueAs<std::vector<double>>("kp", context);
        auto kd = Impl::GetElementValueAs<std::vector<double>>("kd", context);
        auto gravity = Impl::GetElementValueAs< //
            std::vector<double>>("gravity", context);
        auto joints = Impl::GetElementValueAs< //
            std::vector<std::string>>("joints", context);

        if (gravity.size() != 3) {
            sError << "Parsed gravity does not have three elements";
            return nullptr;
        }

        auto controller =
            std::make_shared<controllers::ComputedTorqueFixedBase>(
                urdf,
                model,
                kp,
                kd,
                joints,
                std::array<double, 3>{gravity[0], gravity[1], gravity[2]});

        return controller;
    }

    return nullptr;
}

bool ControllersFactory::Impl::ContextValid(const sdf::ElementPtr context)
{
    // Check that the context is a <controller> element
    if (context->GetName() != "controller") {
        sError << "The first element of the context must be <controller>"
               << std::endl;
        return false;
    }

    // Check that there is only one <controller> element
    if (context->GetNextElement("controller")) {
        sError << "Found multiple <controller> elements in controller context"
               << std::endl;
        return false;
    }

    // Check that there is a name attribute: <controller name="controller_name">
    if (!context->HasAttribute("name")) {
        sError << "Failed to find 'name' attribute in <controller> element"
               << std::endl;
        return false;
    }

    return true;
}

template <typename T>
T ControllersFactory::Impl::GetElementValueAs(
    const std::string& elementName,
    const sdf::ElementPtr parentContext)
{
    if (!parentContext->HasElement(elementName)) {
        sError << "Failed to find element <" << elementName << ">" << std::endl;
        return {};
    }

    sdf::ElementPtr element = parentContext->GetElement(elementName);
    assert(element);

    sdf::ParamPtr value = element->GetValue();

    if (!value) {
        sError << "Failed to get value of element <" << elementName << ">"
               << std::endl;
        return {};
    }

    T output;
    std::string valueString = value->GetAsString();

    Impl::StringToStd(valueString, output);
    return output;
}

void ControllersFactory::Impl::StringToStd(const std::string& string,
                                           std::vector<double>& out)
{
    double value;
    std::stringstream in(string);

    // Locale independent floating point conversion
    // Note: not yet supported by GCC8
    //
    //    auto toDouble = [](const std::string& s) -> double {
    //        double output;
    //        std::from_chars(s.data(), s.data() + s.size(), output);
    //        return output;
    //    };

    // Manually set the locale
    in.imbue(std::locale::classic());

    out.clear();

    while (in >> value) {
        out.push_back(value);
    }
}

void ControllersFactory::Impl::StringToStd(const std::string& string,
                                           std::vector<std::string>& out)
{
    std::string value;
    std::stringstream in(string);

    out.clear();

    while (in >> value) {
        out.push_back(value);
    }
}

void ControllersFactory::Impl::StringToStd(const std::string& string,
                                           std::string& out)
{
    out = string;
}
