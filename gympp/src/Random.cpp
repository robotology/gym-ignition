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
