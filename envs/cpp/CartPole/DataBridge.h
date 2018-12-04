#ifndef GYMPP_IGNITION_PLUGINS_DATABRIGE
#define GYMPP_IGNITION_PLUGINS_DATABRIGE

#include <memory>

#include "ignition/gazebo/System.hh"

namespace gympp {
    //    namespace ignition {
    namespace plugins {
        class DataBridge;
    }
    //    } // namespace ignition
} // namespace gympp

class gympp::plugins::DataBridge final
    : public ::ignition::gazebo::System
    , public ::ignition::gazebo::ISystemPreUpdate
    , public ::ignition::gazebo::ISystemUpdate
    , public ::ignition::gazebo::ISystemPostUpdate
{
public:
    void PreUpdate(const ::ignition::gazebo::UpdateInfo& info,
                   ::ignition::gazebo::EntityComponentManager& manager) override;

    void Update(const ::ignition::gazebo::UpdateInfo& info,
                ::ignition::gazebo::EntityComponentManager& manager) override;

    void PostUpdate(const ::ignition::gazebo::UpdateInfo& info,
                    const ::ignition::gazebo::EntityComponentManager& manager) override;

    using CallbackType = std::function<void(const ::ignition::gazebo::UpdateInfo&,
                                            const ::ignition::gazebo::EntityComponentManager&)>;

    CallbackType preUpdateCallback;
    CallbackType updateCallback;
    CallbackType postUpdateCallback;
};

#endif // GYMPP_IGNITION_PLUGINS_DATABRIGE
