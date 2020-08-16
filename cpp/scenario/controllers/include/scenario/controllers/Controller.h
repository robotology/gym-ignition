/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef SCENARIO_CONTROLLERS_CONTROLLER_H
#define SCENARIO_CONTROLLERS_CONTROLLER_H

#include "scenario/controllers/References.h"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace scenario::controllers {
    class Controller;
    class UseScenarioModel;
    class SetBaseReferences;
    class SetJointReferences;
    using ControllerPtr = std::shared_ptr<Controller>;
    constexpr std::array<double, 3> g = {0, 0, -9.80665};
} // namespace scenario::controllers

namespace scenario::core {
    class Model;
    using ModelPtr = std::shared_ptr<Model>;
} // namespace scenario::core

class scenario::controllers::Controller
    : public std::enable_shared_from_this<scenario::controllers::Controller>
{
public:
    using StepSize = std::chrono::duration<double>;

    Controller() = default;
    virtual ~Controller() = default;

    virtual bool initialize() = 0;
    virtual bool step(const StepSize& dt) = 0;
    virtual bool terminate() = 0;
};

class scenario::controllers::UseScenarioModel
{
public:
    UseScenarioModel() = default;
    virtual ~UseScenarioModel() = default;

    virtual bool updateStateFromModel() = 0;

protected:
    core::ModelPtr m_model;
};

class scenario::controllers::SetBaseReferences
{
public:
    SetBaseReferences() = default;
    virtual ~SetBaseReferences() = default;

    virtual bool setBaseReferences(const BaseReferences& jointReferences) = 0;
};

class scenario::controllers::SetJointReferences
{
public:
    SetJointReferences() = default;
    virtual ~SetJointReferences() = default;

    virtual const std::vector<std::string>& controlledJoints() = 0;
    virtual bool setJointReferences(const JointReferences& jointReferences) = 0;

protected:
    std::vector<std::string> m_controlledJoints;
};

#endif // SCENARIO_CONTROLLERS_CONTROLLER_H
