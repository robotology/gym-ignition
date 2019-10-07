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

    void clean(const std::string& worldName);
    bool valid(const std::string& worldName) const;

    bool exist(const std::string& worldName) const;

    ignition::gazebo::EventManager* getEventManager(const std::string& worldName) const;
    ignition::gazebo::EntityComponentManager* getECM(const std::string& worldName) const;

    bool storePtrs(const std::string& worldName,
                   ignition::gazebo::EntityComponentManager* ecm,
                   ignition::gazebo::EventManager* eventMgr);

    void notifyExecutorFinished(const std::string& worldName);
    void notifyAndWaitPreUpdate(const std::string& worldName);
    std::unique_lock<std::mutex> waitPreUpdate(const std::string& worldName);
};

#endif // GYMPP_GAZEBO_ECMSINGLETON_H
