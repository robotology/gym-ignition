/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef GYMPP_GAZEBO_PLUGINDATABASE_H
#define GYMPP_GAZEBO_PLUGINDATABASE_H

#include "gympp/base/Log.h"
#include "gympp/base/Space.h"
#include "gympp/gazebo/GymFactory.h"
#include "gympp/gazebo/Metadata.h"

class GymppPluginRegistrator_CartPole
{
public:
    GymppPluginRegistrator_CartPole()
    {
        auto factory = gympp::gazebo::GymFactory::Instance();
        gympp::gazebo::PluginMetadata cartPoleMetadata;
        gymppDebug << "Registering 'CartPole' plugin" << std::endl;

        cartPoleMetadata.setEnvironmentName("CartPole");
        cartPoleMetadata.setLibraryName("CartPolePlugin");
        cartPoleMetadata.setClassName("gympp::plugins::CartPole");
        cartPoleMetadata.setModelFileName("CartPole/CartPole.urdf");
        cartPoleMetadata.setWorldFileName("DefaultEmptyWorld.world");

        gympp::gazebo::SpaceMetadata actionSpaceMetadata;
        actionSpaceMetadata.setDimensions({3});
        actionSpaceMetadata.setType(gympp::gazebo::SpaceType::Discrete);

        const double xThreshold = 2.5;
        const double thetaThreshold = 24;

        gympp::gazebo::SpaceMetadata observationSpaceMetadata;
        observationSpaceMetadata.setType(gympp::gazebo::SpaceType::Box);
        double maxDouble = std::numeric_limits<double>::max();
        observationSpaceMetadata.setLowLimit(gympp::base::spaces::Box::Limit{
            -xThreshold, -maxDouble, -thetaThreshold, -maxDouble});
        observationSpaceMetadata.setHighLimit(gympp::base::spaces::Box::Limit{
            xThreshold, maxDouble, thetaThreshold, maxDouble});

        cartPoleMetadata.setActionSpaceMetadata(actionSpaceMetadata);
        cartPoleMetadata.setObservationSpaceMetadata(observationSpaceMetadata);

        gympp::gazebo::PhysicsData physicsData;
        physicsData.rtf = 1E9;
        physicsData.maxStepSize = 0.001;
        cartPoleMetadata.setPhysicsData(physicsData);

        cartPoleMetadata.setAgentRate(1000);

        factory->registerPlugin(cartPoleMetadata);
    }
};

static GymppPluginRegistrator_CartPole plugin;

#endif // GYMPP_GAZEBO_PLUGINDATABASE_H
