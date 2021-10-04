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

#ifndef SCENARIO_PLUGINS_CONTROLLERSFACTORY_H
#define SCENARIO_PLUGINS_CONTROLLERSFACTORY_H

#include "scenario/controllers/Controller.h"
#include "scenario/core/Model.h"

#include <sdf/Element.hh>

#include <memory>

namespace scenario::plugins::gazebo {
    class ControllersFactory;
} // namespace scenario::plugins::gazebo

class scenario::plugins::gazebo::ControllersFactory
{
public:
    ControllersFactory();
    ~ControllersFactory();

    static ControllersFactory& Instance();
    controllers::ControllerPtr get(const sdf::ElementPtr context,
                                   scenario::core::ModelPtr model);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

#endif // SCENARIO_PLUGINS_CONTROLLERSFACTORY_H
