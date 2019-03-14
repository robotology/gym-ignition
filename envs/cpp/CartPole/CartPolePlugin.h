#ifndef GYMPP_PLUGINS_CARTPOLE
#define GYMPP_PLUGINS_CARTPOLE

#include "gympp/gyms/Ignition.h"

#include <ignition/gazebo/System.hh>

#include <memory>
#include <optional>

namespace gympp {
    namespace plugins {
        class CartPole;
    } // namespace plugins
} // namespace gympp

class gympp::plugins::CartPole final
    : public ignition::gazebo::System
    , public ignition::gazebo::ISystemConfigure
    , public ignition::gazebo::ISystemPreUpdate
    , public ignition::gazebo::ISystemPostUpdate
    , public gympp::gyms::EnvironmentBehavior
{
private:
    class Impl;
    std::unique_ptr<Impl, std::function<void(Impl*)>> pImpl = nullptr;

public:
    CartPole();
    ~CartPole() override = default;

    void Configure(const ignition::gazebo::Entity& id,
                   const std::shared_ptr<const sdf::Element>& sdf,
                   ignition::gazebo::EntityComponentManager& ecm,
                   ignition::gazebo::EventManager& eventMgr) override;

    void PreUpdate(const ignition::gazebo::UpdateInfo& info,
                   ignition::gazebo::EntityComponentManager& manager) override;

    void PostUpdate(const ignition::gazebo::UpdateInfo& info,
                    const ignition::gazebo::EntityComponentManager& manager) override;

    bool isDone() override;
    bool reset() override;
    bool setAction(const Action& action) override;
    std::optional<Reward> computeReward() override;
    std::optional<Observation> getObservation() override;
};

#endif // GYMPP_PLUGINS_CARTPOLE
