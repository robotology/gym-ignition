#ifndef GYMPP_GYMFACTORY
#define GYMPP_GYMFACTORY

#include "gympp/Gympp.h"

namespace gympp {
    class GymFactory;
}

class gympp::GymFactory
{
public:
    static gympp::EnvironmentPtr make(const std::string& envName);
};

#endif // GYMPP_GYMFACTORY
