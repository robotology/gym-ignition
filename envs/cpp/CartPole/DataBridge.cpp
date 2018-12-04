#include "DataBridge.h"

#include <ignition/plugin/Register.hh>

// using namespace gympp::ignition::plugins;
using namespace gympp::plugins;

void DataBridge::PreUpdate(const ignition::gazebo::UpdateInfo& info,
                           ignition::gazebo::EntityComponentManager& manager)
{
    if (this->preUpdateCallback) {
        this->preUpdateCallback(info, manager);
    }
}

void DataBridge::Update(const ignition::gazebo::UpdateInfo& info,
                        ignition::gazebo::EntityComponentManager& manager)
{
    if (this->updateCallback) {
        this->updateCallback(info, manager);
    }
}

void DataBridge::PostUpdate(const ignition::gazebo::UpdateInfo& info,
                            const ignition::gazebo::EntityComponentManager& manager)
{
    if (this->postUpdateCallback) {
        this->postUpdateCallback(info, manager);
    }
}

IGNITION_ADD_PLUGIN(gympp::plugins::DataBridge,
                    gympp::plugins::DataBridge::System, // TODO: check
                    gympp::plugins::DataBridge::ISystemPreUpdate,
                    gympp::plugins::DataBridge::ISystemUpdate,
                    gympp::plugins::DataBridge::ISystemPostUpdate)
