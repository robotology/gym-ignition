/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef GYMPP_GYMFACTORY
#define GYMPP_GYMFACTORY

#include "gympp/Environment.h"
#include <ignition/common/SingletonT.hh>

#include <functional>
#include <memory>
#include <string>

namespace gympp {
    class GymFactory;
    class PluginMetadata;
} // namespace gympp

class gympp::GymFactory : public ignition::common::SingletonT<gympp::GymFactory>
{
private:
    class Impl;
    std::unique_ptr<Impl, std::function<void(Impl*)>> pImpl;

public:
    GymFactory();

    gympp::EnvironmentPtr make(const std::string& envName);
    bool registerPlugin(const PluginMetadata& md);
};

#endif // GYMPP_GYMFACTORY
