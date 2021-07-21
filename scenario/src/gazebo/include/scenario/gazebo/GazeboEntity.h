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

#ifndef SCENARIO_GAZEBO_GAZEBOENTITY_H
#define SCENARIO_GAZEBO_GAZEBOENTITY_H

#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>

#include <cstdint>

namespace scenario::gazebo {
    class GazeboEntity;
} // namespace scenario::gazebo

class scenario::gazebo::GazeboEntity
{
public:
    GazeboEntity() = default;
    virtual ~GazeboEntity() = default;

    /**
     * Get the unique id of the object.
     *
     * @note It might differ from the entity number since a multi-world setting
     * with the same models inserted in the same order would result to same
     * numbering.
     *
     * @return The unique object id. Invalid objects return 0.
     */
    virtual uint64_t id() const = 0;

    /**
     * Initialize the object with entity data.
     *
     * @param linkEntity The entity of the ECM.
     * @param ecm The pointer to the ECM.
     * @param eventManager The pointer to the EventManager.
     * @return True for success, false otherwise.
     */
    virtual bool initialize(const ignition::gazebo::Entity linkEntity,
                            ignition::gazebo::EntityComponentManager* ecm,
                            ignition::gazebo::EventManager* eventManager) = 0;

    /**
     * Initialize the object.
     *
     * @note This method has to be called after ``GazeboEntity::initialize``.
     *
     * @return True for success, false otherwise.
     */
    virtual bool createECMResources() = 0;

    /**
     * Return the entity of this object.
     *
     * @return The entity that corresponds to this object.
     */
    inline ignition::gazebo::Entity entity() const { return this->m_entity; }

    /**
     * Return the pointer to the event manager.
     *
     * @return The pointer to the event manager.
     */
    inline ignition::gazebo::EventManager* eventManager() const
    {
        return this->m_eventManager;
    }

    /**
     * Return the pointer to the Entity Component Manager.
     *
     * @return The pointer to the Entity Component Manager.
     */
    inline ignition::gazebo::EntityComponentManager* ecm() const
    {
        return this->m_ecm;
    }

    /**
     * Checks if the GazeboEntity is valid.
     *
     * @return True if the GazeboEntity is valid, false otherwise.
     */
    inline bool validEntity() const
    {
        return m_eventManager && m_ecm
               && m_entity != ignition::gazebo::kNullEntity;
    }

protected:
    ignition::gazebo::EventManager* m_eventManager = nullptr;
    ignition::gazebo::EntityComponentManager* m_ecm = nullptr;
    ignition::gazebo::Entity m_entity = ignition::gazebo::kNullEntity;
};

#endif // SCENARIO_GAZEBO_GAZEBOENTITY_H
