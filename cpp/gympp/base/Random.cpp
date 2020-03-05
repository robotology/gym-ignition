/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "gympp/Random.h"
#include <random>

static std::random_device rd;
static size_t Seed = rd();
static std::default_random_engine Engine(Seed);

gympp::Random& gympp::Random::instance()
{
    static Random random;
    return random;
}

size_t gympp::Random::seed()
{
    return Seed;
}

std::default_random_engine& gympp::Random::engine()
{
    return Engine;
}

void gympp::Random::setSeed(size_t seed)
{
    Seed = seed;
    Engine = std::default_random_engine(seed);
}
