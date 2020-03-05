/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef GYMPP_RANDOM
#define GYMPP_RANDOM

#include <random>

namespace gympp {
    class Random;
}

class gympp::Random
{
public:
    Random() = default;
    ~Random() = default;

    static Random& instance();
    static size_t seed();
    static std::default_random_engine& engine();
    static void setSeed(size_t seed);
};

#endif // GYMPP_RANDOM
