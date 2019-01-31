#ifndef GYMPP_PLUGINS_CARTPOLE
#define GYMPP_PLUGINS_CARTPOLE

#include "gympp/gyms/Ignition.h"

#include <ignition/gazebo/System.hh>

#include <memory>

namespace gympp {
    //    namespace ignition {
    namespace plugins {
        class CartPole;
    } // namespace plugins
    //    } // namespace ignition
} // namespace gympp

// using TypedEnvironmentBehavior = gympp::gyms::EnvironmentBehavior<size_t, double>;

class gympp::plugins::CartPole final
    : public ignition::gazebo::System
    , public ignition::gazebo::ISystemConfigure
    , public ignition::gazebo::ISystemPreUpdate
    , public ignition::gazebo::ISystemPostUpdate
    , public gympp::gyms::EnvironmentBehavior
//    , public ::ignition::gazebo::ISystemUpdate
//    , public TypedEnvironmentBehavior
{
private:
    class Impl;
    std::unique_ptr<Impl, std::function<void(Impl*)>> pImpl = nullptr;

public:
    CartPole();
    ~CartPole() override = default;

    void Configure(const ignition::gazebo::EntityId& _id,
                   const std::shared_ptr<const sdf::Element>& _sdf,
                   ignition::gazebo::EntityComponentManager& _ecm,
                   ignition::gazebo::EventManager& _eventMgr) override;

    void PreUpdate(const ignition::gazebo::UpdateInfo& info,
                   ignition::gazebo::EntityComponentManager& manager) override;

    //    void Update(const ::ignition::gazebo::UpdateInfo& info,
    //                ::ignition::gazebo::EntityComponentManager& manager) override;

    void PostUpdate(const ignition::gazebo::UpdateInfo& info,
                    const ignition::gazebo::EntityComponentManager& manager) override;

    bool isDone() override;
    bool setAction(const Action& action) override;
    std::optional<Reward> computeReward() override;
    std::optional<Observation> getObservation() override;

    // using CallbackType = std::function<void(const ::ignition::gazebo::UpdateInfo&,
    //                                         const ::ignition::gazebo::EntityComponentManager&)>;

    //    CallbackType preUpdateCallback;
    //    CallbackType updateCallback;
    //    CallbackType postUpdateCallback;
};

#endif // GYMPP_PLUGINS_CARTPOLE
