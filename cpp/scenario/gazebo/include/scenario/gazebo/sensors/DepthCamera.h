/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef SCENARIO_GAZEBO_SENSORS_DEPTHCAMERA_H
#define SCENARIO_GAZEBO_SENSORS_DEPTHCAMERA_H

#include "scenario/gazebo/GazeboEntity.h"

#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>

#include <memory>
#include <string>
#include <vector>

namespace scenario::gazebo::sensors {
    class DepthCamera;
} // namespace scenario::gazebo::sensors

class scenario::gazebo::sensors::DepthCamera final
    : public scenario::gazebo::GazeboEntity
{
public:
    DepthCamera();
    virtual ~DepthCamera();

    // =============
    // Gazebo Entity
    // =============

    uint64_t id() const override;

    bool initialize(const ignition::gazebo::Entity parentEntity,
                    ignition::gazebo::EntityComponentManager* ecm,
                    ignition::gazebo::EventManager* eventManager) override;

    bool createECMResources() override;

    // ===========
    // Camera Core
    // ===========

    std::string name(const bool scoped = false) const;

    double width() const;
    double height() const;

    double farClip() const;
    double nearClip() const;

    const std::vector<float>& image() const;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

#endif // SCENARIO_GAZEBO_SENSORS_DEPTHCAMERA_H
