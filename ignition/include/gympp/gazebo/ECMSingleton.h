/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef GYMPP_GAZEBO_ECMSINGLETON_H
#define GYMPP_GAZEBO_ECMSINGLETON_H

#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/EventManager.hh"

#include <functional>
#include <memory>
#include <string>

namespace gympp {
    namespace gazebo {
        class ECMSingleton;
    } // namespace gazebo
} // namespace gympp

class gympp::gazebo::ECMSingleton
{
private:
    class Impl;
    std::unique_ptr<Impl, std::function<void(Impl*)>> pImpl;

public:
    ECMSingleton();
    ~ECMSingleton() = default;
    ECMSingleton(ECMSingleton&) = delete;
    void operator=(const ECMSingleton&) = delete;

    static ECMSingleton& get();

    bool valid() const;

    ignition::gazebo::EventManager* getEventManager() const;
    ignition::gazebo::EntityComponentManager* getECM() const;

    bool storePtrs(ignition::gazebo::EntityComponentManager* ecm,
                   ignition::gazebo::EventManager* eventMgr);
};

#endif // GYMPP_GAZEBO_ECMSINGLETON_H
