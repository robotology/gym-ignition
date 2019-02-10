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
    static unsigned seed();
    static std::default_random_engine& engine();
    static void setSeed(unsigned seed);
};

#endif // GYMPP_RANDOM
