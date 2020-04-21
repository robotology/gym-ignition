# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from . import cartpole
from . import gympp_env
from gym.envs import register


register(
    id='CartPoleDiscrete-Gympp-v0',
    max_episode_steps=5000,
    entry_point='gym_ignition.experimental.gympp.cartpole:CartPoleDiscrete')


def get_empty_world() -> str:

    empty_world = f"""<?xml version="1.0" ?>
<sdf version="1.6">
    <world name="default">
        <physics default="true" type="dart">
        </physics>
        <plugin filename="libPhysicsSystem.so"
                name="scenario::plugins::gazebo::Physics">
        </plugin>
        <plugin filename="libECMProvider.so"
                name="scenario::plugins::gazebo::ECMProvider">
        </plugin>
        <plugin filename="libignition-gazebo-scene-broadcaster-system.so"
                name="ignition::gazebo::systems::SceneBroadcaster">
        </plugin>
        <light type="directional" name="sun">
            <cast_shadows>true</cast_shadows>
            <pose>0 0 10 0 0 0</pose>
            <diffuse>1 1 1 1</diffuse>
            <specular>0.5 0.5 0.5 1</specular>
            <attenuation>
                <range>1000</range>
                <constant>0.9</constant>
                <linear>0.01</linear>
                <quadratic>0.001</quadratic>
            </attenuation>
            <direction>-0.5 0.1 -0.9</direction>
        </light>
    </world>
</sdf>"""

    return empty_world
