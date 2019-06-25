# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# Import gympp bindings
# See https://github.com/robotology/gym-ignition/issues/7
import sys
if sys.platform.startswith('linux') or sys.platform.startswith('darwin'):
    import ctypes
    sys.setdlopenflags(sys.getdlopenflags() | ctypes.RTLD_GLOBAL)
import gympp_bindings

# Import abstract classes
from gym_ignition.base.task import Task
from gym_ignition.base.robot import Robot

# =========================
# REGISTER THE ENVIRONMENTS
# =========================

from gym.envs.registration import register

# Import the robots
from gym_ignition.robots import rt
from gym_ignition.robots import sim

# Import the tasks
from gym_ignition.tasks import cartpole_discrete
from gym_ignition.tasks import cartpole_continuous

# ======================
# GYMPP C++ ENVIRONMENTS
# ======================

import numpy as np
max_float = float(np.finfo(np.float32).max)

register(
    id='CartPoleGympp-Discrete-v0',
    max_episode_steps=5000,
    entry_point='gym_ignition.gympp.cartpole:CartPoleDiscrete')

# ==========================
# GYMPPY PYTHON ENVIRONMENTS
# ==========================

register(
    id='CartPoleGymppy-Discrete-v0',
    entry_point='gym_ignition.base.gazebo_env:GazeboEnv',
    max_episode_steps=5000,
    kwargs={'task': cartpole_discrete.CartPoleDiscrete,
            'robot': sim.cartpole.CartPoleRobot,
            'sdf': "CartPole/CartPole.sdf",
            'world': "DefaultEmptyWorld.world",
            'rtf': max_float,
            'agent_rate': 1000,
            'physics_rate': 1000,
            })

register(
    id='CartPoleGymppy-Continuous-v0',
    entry_point='gym_ignition.base.gazebo_env:GazeboEnv',
    max_episode_steps=5000,
    kwargs={'task': cartpole_continuous.CartPoleContinuous,
            'robot': sim.cartpole.CartPoleRobot,
            'sdf': "CartPole/CartPole.sdf",
            'world': "DefaultEmptyWorld.world",
            'rtf': max_float,
            'agent_rate': 1000,
            'physics_rate': 1000,
            })
