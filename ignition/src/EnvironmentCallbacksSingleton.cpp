#include "gympp/gazebo/EnvironmentCallbacksSingleton.h"
#include "gympp/Log.h"

#include <unordered_map>

using namespace gympp::gazebo;

class EnvironmentCallbacksSingleton::Impl
{
public:
    std::unordered_map<std::string, EnvironmentCallbacks*> callbacks;
};

EnvironmentCallbacksSingleton::EnvironmentCallbacksSingleton()
    : pImpl{new Impl(), [](Impl* impl) { delete impl; }}
{}

EnvironmentCallbacks* EnvironmentCallbacksSingleton::get(const std::string& label)
{
    if (pImpl->callbacks.find(label) == pImpl->callbacks.end()) {
        gymppError << "Failed to find environment callbacks labelled as '" << label << "'"
                   << std::endl;
        return nullptr;
    }

    assert(pImpl->callbacks.at(label));
    return pImpl->callbacks.at(label);
}

bool EnvironmentCallbacksSingleton::storeEnvironmentCallback(const std::string& label,
                                                             EnvironmentCallbacks* cb)
{
    if (!cb || label.empty()) {
        gymppError << "Trying to store invalid environment callbacks" << std::endl;
        return false;
    }

    if (pImpl->callbacks.find(label) != pImpl->callbacks.end()) {
        gymppError << "Environment callbacks with label '" << label
                   << "' have been already registered" << std::endl;
    }

    pImpl->callbacks.insert({label, cb});
    return true;
}
