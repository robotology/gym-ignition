#include "CartPolePlugin.h"
#include "gympp/Random.h"

#include <ignition/gazebo/Model.hh>
#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>

#include <cmath>
#include <memory>

using namespace gympp::plugins;
using namespace ignition::gazebo::components;

enum CartPoleAction
{
    MOVE_RIGHT,
    MOVE_LEFT,
    DONT_MOVE,
};

class CartPole::Impl
{
public:
    unsigned seed;
    bool firstRun = true;
    mutable std::mutex mutex;
    ignition::gazebo::Model model;

    CartPoleAction action;
    BufferDouble observationBuffer;

    ignition::gazebo::EntityComponentManager* manager = nullptr;

    double getRandomTheta();
};

double CartPole::Impl::getRandomTheta()
{
    std::uniform_real_distribution<> distr(-25.0 * (M_PI / 180.0), 25.0 * (M_PI / 180.0));
    return distr(gympp::Random::engine());
}

CartPole::CartPole()
    : pImpl{new Impl(), [](Impl* impl) { delete impl; }}
{
    // - Pole angular position [deg]
    // - Cart linear position [m]
    pImpl->observationBuffer.resize(2);
}

// TODO: this is not called if the plugin is not put in the sdf.
//       Probably this action will be supported in the future.
void CartPole::Configure(const ignition::gazebo::Entity& /*entity*/,
                         const std::shared_ptr<const sdf::Element>& /*_sdf*/,
                         ignition::gazebo::EntityComponentManager& manager,
                         ignition::gazebo::EventManager& /*_eventMgr*/)
{
    // Save the manager. It is used in reset().
    pImpl->manager = &manager;

    // Get the model from the entity manager
    auto id = manager.EntityByComponents(ignition::gazebo::components::Model(),
                                         ignition::gazebo::components::Name("cartpole_xacro"));
    this->pImpl->model = ignition::gazebo::Model(id);

    // Get the links from the model
    auto pole = this->pImpl->model.LinkByName(manager, "cartpole_xacro::pole");
    auto cart = this->pImpl->model.LinkByName(manager, "cartpole_xacro::cart");
    auto rail = this->pImpl->model.LinkByName(manager, "cartpole_xacro::rail");

    // Get the joints
    auto pivot = this->pImpl->model.JointByName(manager, "cartpole_xacro::pivot");
    auto linear = this->pImpl->model.JointByName(manager, "cartpole_xacro::linear");

    // Check that all link exist
    if (pole == ignition::gazebo::kNullEntity) {
        ignerr << "Failed to find 'pole' link" << std::endl;
        return;
    }
    if (cart == ignition::gazebo::kNullEntity) {
        ignerr << "Failed to find 'cart' link" << std::endl;
        return;
    }
    if (rail == ignition::gazebo::kNullEntity) {
        ignerr << "Failed to find 'rail' link" << std::endl;
        return;
    }

    // Check that all joint exist
    if (pivot == ignition::gazebo::kNullEntity) {
        ignerr << "Failed to find 'pivot' joint" << std::endl;
        return;
    }
    if (linear == ignition::gazebo::kNullEntity) {
        ignerr << "Failed to find 'linear' joint" << std::endl;
        return;
    }

    // Generate a random initial angle
    auto theta0 = pImpl->getRandomTheta();

    // Get the joint position component
    auto poleJoint = this->pImpl->model.JointByName(manager, "cartpole_xacro::pivot");
    auto polePos = manager.Component<JointPosition>(poleJoint);

    // Set the random initial angle of the pole
    if (!polePos) {
        manager.CreateComponent(poleJoint, JointPosition(theta0));
    }
    else {
        *polePos = JointPosition(theta0);
    }
}

void CartPole::PreUpdate(const ignition::gazebo::UpdateInfo& /*info*/,
                         ignition::gazebo::EntityComponentManager& manager)
{
    // TODO: the Configure method is called only if this plugin is added
    //       in the sdf file. Right now gym-ignition loads the plugin internally.
    if (pImpl->firstRun) {
        ignition::gazebo::EventManager eventManager;
        Configure(0, nullptr, manager, eventManager);
        pImpl->firstRun = false;
    }

    // Get the cart joint position
    auto cartJoint = this->pImpl->model.JointByName(manager, "cartpole_xacro::linear");
    auto cartPos = manager.Component<JointPosition>(cartJoint);

    if (!cartPos) {
        igndbg << "Cart position not yet initialized. Skipping PreUpdate." << std::endl;
        return;
    }

    // Set the action
    // TODO
    switch (pImpl->action) {
        case MOVE_LEFT:
            *cartPos = JointPosition(cartPos->Data() + 0.02);
            break;
        case MOVE_RIGHT:
            *cartPos = JointPosition(cartPos->Data() - 0.02);
            break;
        case DONT_MOVE:
            break;
    }
}

void CartPole::PostUpdate(const ignition::gazebo::UpdateInfo& /*info*/,
                          const ignition::gazebo::EntityComponentManager& manager)
{
    auto cartJoint = this->pImpl->model.JointByName(manager, "cartpole_xacro::linear");
    auto cartPos = manager.Component<JointPosition>(cartJoint);
    // auto cartVel = manager.Component<components::JointVelocity>(cartJoint);

    auto poleJoint = this->pImpl->model.JointByName(manager, "cartpole_xacro::pivot");
    auto polePos = manager.Component<JointPosition>(poleJoint);
    // auto poleVel = manager.Component<components::JointVelocity>(poleJoint);

    if (!cartPos) {
        igndbg << "Cart position not yet initialized. Skipping PostUpdate." << std::endl;
        return;
    }

    if (!polePos) {
        igndbg << "Pose position not yet initialized. Skipping PostUpdate." << std::endl;
        return;
    }

    {
        std::lock_guard lock(pImpl->mutex);
        pImpl->observationBuffer[0] = (180.0 / M_PI) * polePos->Data();
        pImpl->observationBuffer[1] = cartPos->Data();
    }
}

bool CartPole::isDone()
{
    std::lock_guard lock(pImpl->mutex);

    if (std::abs(pImpl->observationBuffer[0]) > 60.0
        || std::abs(pImpl->observationBuffer[1]) > 0.98) {
        return true;
    }

    return false;
}

bool CartPole::reset()
{
    assert(pImpl->manager);

    // Reset the pole position
    auto poleJoint = this->pImpl->model.JointByName(*pImpl->manager, "cartpole_xacro::pivot");
    auto polePos = pImpl->manager->Component<JointPosition>(poleJoint);
    assert(polePos);
    auto theta0 = pImpl->getRandomTheta();
    *polePos = JointPosition(theta0);

    // Reset the cart position
    auto cartJoint = this->pImpl->model.JointByName(*pImpl->manager, "cartpole_xacro::linear");
    auto cartPos = pImpl->manager->Component<JointPosition>(cartJoint);
    assert(cartPos);
    *cartPos = JointPosition(0);

    {
        // TODO: store the entities and access them direcly in getObservation?
        std::lock_guard lock(pImpl->mutex);
        pImpl->observationBuffer[0] = (180.0 / M_PI) * polePos->Data();
        pImpl->observationBuffer[1] = cartPos->Data();
    }

    return true;
}

bool CartPole::setAction(const gympp::gyms::EnvironmentBehavior::Action& action)
{
    std::lock_guard lock(pImpl->mutex);
    assert(action.get<size_t>());
    auto actionValue = (*action.get<size_t>())[0];

    if (actionValue == 0) {
        pImpl->action = MOVE_LEFT;
    }
    else if (actionValue == 1) {
        pImpl->action = MOVE_RIGHT;
    }
    else if (actionValue == 2) {
        pImpl->action = DONT_MOVE;
    }
    else {
        return false;
    }

    return true;
}

std::optional<gympp::gyms::EnvironmentBehavior::Reward> CartPole::computeReward()
{
    std::lock_guard lock(pImpl->mutex);
    return 1.0; // TODO
}

std::optional<gympp::gyms::EnvironmentBehavior::Observation> CartPole::getObservation()
{
    std::lock_guard lock(pImpl->mutex);
    return Observation(pImpl->observationBuffer);
}

IGNITION_ADD_PLUGIN(gympp::plugins::CartPole,
                    gympp::plugins::CartPole::System, // TODO: check
                    gympp::plugins::CartPole::ISystemConfigure,
                    gympp::plugins::CartPole::ISystemPreUpdate,
                    // gympp::plugins::CartPole::ISystemUpdate,
                    gympp::plugins::CartPole::ISystemPostUpdate,
                    gympp::gyms::EnvironmentBehavior)
