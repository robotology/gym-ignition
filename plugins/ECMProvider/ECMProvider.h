/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef GYMPP_PLUGINS_ECMPROVIDER
#define GYMPP_PLUGINS_ECMPROVIDER

#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>
#include <ignition/gazebo/System.hh>
#include <sdf/Element.hh>

#include <functional>
#include <memory>
#include <optional>

namespace gympp {
    namespace plugins {
        class ECMProvider;
    } // namespace plugins
} // namespace gympp

class gympp::plugins::ECMProvider final
    : public ignition::gazebo::System
    , public ignition::gazebo::ISystemConfigure
    , public ignition::gazebo::ISystemPreUpdate
{
private:
    class Impl;
    std::unique_ptr<Impl, std::function<void(Impl*)>> pImpl = nullptr;

public:
    ECMProvider();
    ~ECMProvider() override;

    void Configure(const ignition::gazebo::Entity& entity,
                   const std::shared_ptr<const sdf::Element>& sdf,
                   ignition::gazebo::EntityComponentManager& ecm,
                   ignition::gazebo::EventManager& eventMgr) override;

    void PreUpdate(const ignition::gazebo::UpdateInfo& info,
                   ignition::gazebo::EntityComponentManager& ecm) override;
};

#endif // GYMPP_PLUGINS_ECMPROVIDER
