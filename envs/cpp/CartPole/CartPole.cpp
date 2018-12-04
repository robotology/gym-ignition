#include "CartPole.h"

#include "gympp/Log.h"
#include "gympp/spaces/Space.h"

#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/SystemManager.hh>
#include <ignition/plugin/SpecializedPluginPtr.hh> // TODO: cmake find_package
#include <sdf/Joint.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

// TODO: figure out how to properly get this header since it is the child class and not the IF
#include "DataBridge.h"

#include <cassert>

const std::string ModelName = "cartpole_xacro";
const std::string PoleJointName = "pivot";
const std::string CartJointName = "linear";

// using namespace ignition;
using namespace gympp::env;

struct SdfData
{
    std::string file;
    const sdf::World* world = nullptr;
    std::unique_ptr<sdf::Root> root;

    struct JointState
    {
        double position = 0;
        double velocity = 0;
    };

    JointState pivot;
    JointState linear;
};

class CartPole::Impl
{
public:
    bool getPoleAngle();

    void OnPreUpdate(gympp::plugins::DataBridge::CallbackType cb)
    {
        // TODO: check pointer
        dataBridge->preUpdateCallback = cb;
    }

    void OnUpdate(gympp::plugins::DataBridge::CallbackType cb) { dataBridge->updateCallback = cb; }

    void OnPostUpdate(gympp::plugins::DataBridge::CallbackType cb)
    {
        dataBridge->postUpdateCallback = cb;
    }

    SdfData sdfData;
    gympp::env::CartPole* cartpole = nullptr;

    //    CallbackType postUpdateCallback;
    gympp::plugins::DataBridge* dataBridge;
    ignition::gazebo::SystemPluginPtr systemPluginPtr;
};

using OSpace = gympp::spaces::Box;
using ASpace = gympp::spaces::Discrete;

CartPole::CartPole(const std::string& sdfFile, double updateRate, uint64_t iterations)
    //    : gyms::IgnitionGazebo(
    : gyms::IgnitionGazebo<AType, OType>(
          std::make_shared<ASpace>(2),
          std::make_shared<OSpace>(OSpace::Limit{-90, -20}, OSpace::Limit{90, 20}), // TODO
          sdfFile,
          updateRate,
          iterations)
    , pImpl{new Impl(), [](Impl* impl) { delete impl; }}
{
    // This pImpl needs the pointer to the parent object
    pImpl->cartpole = this;

    // NOTE: Do not load the sdf file here. The parent class loads the sdf before the first step()
    // and it handles properly possible parsing errors.
    pImpl->sdfData.file = sdfFile;
}

bool CartPole::Impl::getPoleAngle()
{
    if (!cartpole->m_server) {
        gymppError << "Failed to get instance of Ignition Server" << std::endl;
        return false;
    }

    // Allocate the SDF parser lazily
    if (!sdfData.root) {
        sdfData.root = std::make_unique<sdf::Root>();
        assert(sdfData.root);

        // Load the sdf
        auto errors = sdfData.root->Load(sdfData.file);

        if (errors.size() > 0) {
            gymppError << "Failed to load sdf file" << std::endl;
            return false;
        }

        // Get the world from its index
        sdfData.world = sdfData.root->WorldByIndex(0);

        if (!sdfData.world) {
            gymppError << "Failed to get the world from the sdf" << std::endl;
            return false;
        }
    }

    // Load the plugin
    ignition::gazebo::SystemManager sm;
    sm.AddSystemPluginPath("/home/dferigo/git/gym-ignition/build2/lib"); // TODO
    auto plugin = sm.LoadPlugin("libDataBridge.so", "gympp::plugins::DataBridge", nullptr);
    assert(plugin.has_value());
    systemPluginPtr = plugin.value();

    // Get the child class out of it
    dataBridge = static_cast<gympp::plugins::DataBridge*>(
        systemPluginPtr->QueryInterface<ignition::gazebo::System>());

    auto postUpdateCallback = [&](const ignition::gazebo::UpdateInfo&,
                                  const ignition::gazebo::EntityComponentManager& _ecm) {
        using namespace ignition::gazebo::v0;

        //        auto testJoint = [&](const ignition::gazebo::EntityId&,
        //                             const components::Joint* _joint,
        //                             const components::Name* _name) -> bool {
        //            std::cout << _name->Data() << std::endl;
        //            std::cout << _joint << std::endl;
        //            return true;
        //        };
        auto testLink = [&](const ignition::gazebo::EntityId&,
                            const components::Link* _link,
                            const components::Name* _name,
                            const components::Pose* _pose) -> bool {
            auto pos = _pose->Data().Pos();
            //            std::cout << "x: " << pos[0] << std::endl;
            std::cout << _name->Data() << " [y - z] " << pos[1] << " - " << pos[2] << std::endl
                      << std::flush;
            return true;
        };

        //        _ecm.Each<components::Joint, components::Name>(testJoint);
        _ecm.Each<components::Link, components::Name, components::Pose>(testLink);
    };

    OnPostUpdate(postUpdateCallback);

    auto ok = cartpole->m_server->AddSystem(systemPluginPtr);
    assert(ok.has_value());
    assert(ok);

    return true;
}

bool CartPole::isDone()
{
    return false;
}

bool CartPole::setAction(const TypedEnvironmentBehavior::Action& /*action*/)
{
    return true;
}

std::optional<CartPole::TypedEnvironmentBehavior::Reward> CartPole::computeReward()
{
    return CartPole::TypedEnvironmentBehavior::Reward{1};
}

std::optional<CartPole::TypedEnvironmentBehavior::Observation> CartPole::getObservation()
{
    pImpl->getPoleAngle();
    auto obs = CartPole::TypedEnvironmentBehavior::Observation{20, 9};
    return obs;
}
