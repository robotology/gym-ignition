/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef SCENARIO_CORE_WORLD_H
#define SCENARIO_CORE_WORLD_H

#include "scenario/core/Link.h"

#include <memory>
#include <string>
#include <vector>

namespace scenario::core {
    class World;
    class Model;
    using ModelPtr = std::shared_ptr<Model>;
    using WorldPtr = std::shared_ptr<World>;
} // namespace scenario::core

class scenario::core::World
{
public:
    World() = default;
    virtual ~World() = default;

    /**
     * Check if the world is valid.
     *
     * @return True if the world is valid, false otherwise.
     */
    virtual bool valid() const = 0;

    /**
     * Get the simulated time.
     *
     * @note A physics plugin need to be part of the simulation
     * in order to make the time flow.
     *
     * @return The simulated time.
     */
    virtual double time() const = 0;

    /**
     * Get the name of the world.
     *
     * @return The name of the world.
     */
    virtual std::string name() const = 0;

    /**
     * Get the gravity vector.
     * @return The gravity vector.
     */
    virtual std::array<double, 3> gravity() const = 0;

    /**
     * Get the name of the models that are part of the world.
     *
     * @return The list of model names.
     */
    virtual std::vector<std::string> modelNames() const = 0;

    /**
     * Get a model part of the world.
     *
     * @param modelName The name of the model to get.
     * @return The model if it is part of the world, ``nullptr`` otherwise.
     */
    virtual ModelPtr getModel(const std::string& modelName) const = 0;
};

#endif // SCENARIO_CORE_WORLD_H
