# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from gym.envs.registration import register
from gym_ignition.envs.base.ignition_env import IgnitionEnv

register(
    id='CartPoleIgnition-v0',
    entry_point='gym_ignition.envs.cartpole:CartPoleEnv')
