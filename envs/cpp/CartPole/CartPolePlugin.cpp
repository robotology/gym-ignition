#include "CartPolePlugin.h"

#include <ignition/gazebo/Model.hh>
#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>

#include <memory>

using namespace gympp::plugins;

class CartPole::Impl
{
public:
    bool firstRun = true;
    mutable std::mutex mutex;
    ignition::gazebo::Model model;
};

CartPole::CartPole()
    : pImpl{new Impl(), [](Impl* impl) { delete impl; }}
{}

// TODO: this is not called if the plugin is not put in the sdf.
//       Probably this action will be supported in the future.
void CartPole::Configure(const ignition::gazebo::EntityId& _id,
                         const std::shared_ptr<const sdf::Element>& /*_sdf*/,
                         ignition::gazebo::EntityComponentManager& _ecm,
                         ignition::gazebo::EventManager& /*_eventMgr*/)
{
    igndbg << "Configuring!" << std::endl;
    this->pImpl->model = ignition::gazebo::Model(_id);

    auto link = this->pImpl->model.LinkByName(_ecm, "pole");

    // Fail to create component if link is not found
    if (link == ignition::gazebo::kNullEntity) {
        ignerr << "Failed to find link" << std::endl;

        return;
    }
}

using namespace ignition::gazebo;
void CartPole::PreUpdate(const ignition::gazebo::UpdateInfo& /*info*/,
                         ignition::gazebo::EntityComponentManager& manager)
{
    auto testLink = [&](const ignition::gazebo::EntityId&,
                        const components::Link* /*_link*/,
                        const components::Name* _name,
                        const components::Pose * /*_pose*/) -> bool {
        std::cout << _name->Data() << std::endl << std::flush;
        return true;
    };
    manager.Each<ignition::gazebo::components::Link,
                 ignition::gazebo::components::Name,
                 ignition::gazebo::components::Pose>(testLink);

    auto testJoint = [&](const ignition::gazebo::EntityId&,
                         const components::Joint* /*_joint*/,
                         const components::Name* _name,
                         const components::Pose * /*_pose*/) -> bool {
        std::cout << _name->Data() << std::endl << std::flush;
        return true;
    };
    manager.Each<ignition::gazebo::components::Joint,
                 ignition::gazebo::components::Name,
                 ignition::gazebo::components::Pose>(testJoint);

    if (pImpl->firstRun) {
        auto id = manager.EntityByComponents(ignition::gazebo::components::Model(),
                                             ignition::gazebo::components::Name("cartpole_xacro"));
        this->pImpl->model = ignition::gazebo::Model(id);

        auto link = this->pImpl->model.LinkByName(manager, "cartpole_xacro::pole");

        // Fail to create component if link is not found
        if (link == ignition::gazebo::kNullEntity) {
            ignerr << "Failed to find link" << std::endl;
            return;
        }

        auto joint = this->pImpl->model.JointByName(manager, "cartpole_xacro::pivot");

        // Fail to create component if link is not found
        if (joint == ignition::gazebo::kNullEntity) {
            ignerr << "Failed to find joint" << std::endl;
            return;
        }

        pImpl->firstRun = false;
    }

    //    if (this->preUpdateCallback) {
    //        this->preUpdateCallback(info, manager);
    //    }
    igndbg << "PreUpdate!" << std::endl;
}

// void CartPole::Update(const ignition::gazebo::UpdateInfo& info,
//                        ignition::gazebo::EntityComponentManager& manager)
//{
//    if (this->updateCallback) {
//        this->updateCallback(info, manager);
//    }
//}

void CartPole::PostUpdate(const ignition::gazebo::UpdateInfo& /*info*/,
                          const ignition::gazebo::EntityComponentManager& /*manager*/)
{
    //    if (this->postUpdateCallback) {
    //        this->postUpdateCallback(info, manager);
    //    }
}

bool CartPole::isDone()
{
    std::lock_guard lock(pImpl->mutex);
    return true;
}

bool CartPole::setAction(const gympp::gyms::EnvironmentBehavior::Action& /*action*/)
{
    std::lock_guard lock(pImpl->mutex);
    return true;
}

std::optional<gympp::gyms::EnvironmentBehavior::Reward> CartPole::computeReward()
{
    std::lock_guard lock(pImpl->mutex);
    return {};
}

std::optional<gympp::gyms::EnvironmentBehavior::Observation> CartPole::getObservation()
{
    std::lock_guard lock(pImpl->mutex);
    return {};
}

IGNITION_ADD_PLUGIN(gympp::plugins::CartPole,
                    gympp::plugins::CartPole::System, // TODO: check
                    gympp::plugins::CartPole::ISystemConfigure,
                    gympp::plugins::CartPole::ISystemPreUpdate,
                    // gympp::plugins::CartPole::ISystemUpdate,
                    gympp::plugins::CartPole::ISystemPostUpdate,
                    gympp::gyms::EnvironmentBehavior)
