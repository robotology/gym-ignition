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

# =========================
# REGISTER THE ENVIRONMENTS
# =========================

from gym.envs.registration import register
from gym_ignition.utils import resource_finder

# Import the robots
from gym_ignition.robots import rt
from gym_ignition.robots import sim

# Import the tasks
from gym_ignition.tasks import cartpole_discrete
from gym_ignition.tasks import cartpole_continuous

# ======================
# GYMPP C++ ENVIRONMENTS
# ======================

import numpy
max_float = float(numpy.finfo(numpy.float32).max)

register(
    id='CartPoleGympp-Discrete-v0',
    max_episode_steps=5000,
    entry_point='gym_ignition.gympp.cartpole:CartPoleDiscrete')

# ==========================
# GYMPPY PYTHON ENVIRONMENTS
# ==========================

register(
    id='CartPoleGymppy-Discrete-v0',
    entry_point='gym_ignition.runtimes.gazebo_runtime:GazeboRuntime',
    max_episode_steps=5000,
    kwargs={'task_cls': cartpole_discrete.CartPoleDiscrete,
            'robot_cls': sim.gazebo.cartpole.CartPoleGazeboRobot,
            'model': "CartPole/CartPole.sdf",
            'world': "DefaultEmptyWorld.world",
            'rtf': max_float,
            'agent_rate': 1000,
            'physics_rate': 1000,
            'hard_reset': False,
            })

register(
    id='CartPoleGymppy-Continuous-v0',
    entry_point='gym_ignition.runtimes.gazebo_runtime:GazeboRuntime',
    max_episode_steps=5000,
    kwargs={'task_cls': cartpole_continuous.CartPoleContinuous,
            'robot_cls': sim.gazebo.cartpole.CartPoleGazeboRobot,
            'model': "CartPole/CartPole.sdf",
            'world': "DefaultEmptyWorld.world",
            'rtf': max_float,
            'agent_rate': 1000,
            'physics_rate': 1000,
            'hard_reset': False,
            })

# =====================
# PYBULLET ENVIRONMENTS
# =====================

# Add the folders specified in IGN_GAZEBO_RESOURCE_PATH to the search path
resource_finder.add_path_from_env_var("IGN_GAZEBO_RESOURCE_PATH")

register(
    id='CartPole-PyBullet-Discrete-v0',
    entry_point='gym_ignition.runtimes.pybullet_runtime:PyBulletRuntime',
    max_episode_steps=5000,
    kwargs={
            # PyBulletRuntime
            'task_cls': cartpole_discrete.CartPoleDiscrete,
            'robot_cls': sim.pybullet.cartpole.CartPolePyBulletRobot,
            'model': "CartPole/CartPole.urdf",
            'world': "plane_implicit.urdf",
            'rtf': 1.0,
            'agent_rate': 250,
            'physics_rate': 250,
            'hard_reset': False,
            })
