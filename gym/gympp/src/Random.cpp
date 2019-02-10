#include "gympp/Random.h"
#include <random>

std::random_device rd;
static unsigned Seed = rd();
static std::default_random_engine Engine(Seed);

gympp::Random& gympp::Random::instance()
{
    static Random random;
    return random;
}

unsigned gympp::Random::seed()
{
    return Seed;
}

std::default_random_engine& gympp::Random::engine()
{
    return Engine;
}

void gympp::Random::setSeed(unsigned seed)
{
    Seed = seed;
    Engine = std::default_random_engine(seed);
}
