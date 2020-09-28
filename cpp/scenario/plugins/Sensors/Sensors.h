/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
 *
 */

#ifndef SCENARIO_PLUGINS_GAZEBO_SENSORS_H
#define SCENARIO_PLUGINS_GAZEBO_SENSORS_H

#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/config.hh>
#include <ignition/sensors/Sensor.hh>
#include <sdf/Sensor.hh>

#include <memory>
#include <string>

namespace scenario::plugins::gazebo {
    class Sensors;
} // namespace scenario::plugins::gazebo

class scenario::plugins::gazebo::Sensors
    : public ignition::gazebo::System
    , public ignition::gazebo::ISystemConfigure
    , public ignition::gazebo::ISystemPostUpdate
{
public:
    explicit Sensors();

    ~Sensors() override;

    void Configure(const ignition::gazebo::Entity& id,
                   const std::shared_ptr<const sdf::Element>& sdf,
                   ignition::gazebo::EntityComponentManager& ecm,
                   ignition::gazebo::EventManager& eventMgr) final;

    void PostUpdate(const ignition::gazebo::UpdateInfo& info,
                    const ignition::gazebo::EntityComponentManager& ecm) final;

private:
    /// \brief Create a rendering sensor from sdf
    /// \param[in] _sdf SDF description of the sensor
    /// \param[in] _parentName Name of parent that the sensor is attached to
    /// \param[in] _ecm The EntityComponentManager of the simulation instance
    /// \return Sensor name
    std::string CreateSensor(const sdf::Sensor& _sdf,
                             const std::string& _parentName,
                             ignition::gazebo::EntityComponentManager& _ecm);

    /// \brief Private data pointer.
    class SensorsPrivate;
    std::unique_ptr<SensorsPrivate> dataPtr;
};

#endif // SCENARIO_PLUGINS_GAZEBO_SENSORS_H
