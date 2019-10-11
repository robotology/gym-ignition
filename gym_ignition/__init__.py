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

# Configure OS environment variables
from gym_ignition.utils import gazebo_env_vars, resource_finder
gazebo_env_vars.setup_gazebo_env_vars()
resource_finder.add_path_from_env_var("IGN_GAZEBO_RESOURCE_PATH")

# =========================
# REGISTER THE ENVIRONMENTS
# =========================

from gym.envs.registration import register
from gym_ignition.utils import resource_finder

# Import the robots
from gym_ignition.robots import rt
from gym_ignition.robots import sim

# Import the tasks
from gym_ignition.tasks import pendulum_swingup
from gym_ignition.tasks import cartpole_discrete
from gym_ignition.tasks import cartpole_continuous

# ======================
# GYMPP C++ ENVIRONMENTS
# ======================

import numpy
max_float = float(numpy.finfo(numpy.float32).max)

register(
    id='CartPoleDiscrete-Gympp-v0',
    max_episode_steps=5000,
    entry_point='gym_ignition.gympp.cartpole:CartPoleDiscrete')

# ============================
# IGNITION GAZEBO ENVIRONMENTS
# ============================

register(
    id='Pendulum-Gazebo-v0',
    entry_point='gym_ignition.runtimes.gazebo_runtime:GazeboRuntime',
    max_episode_steps=5000,
    kwargs={'task_cls': pendulum_swingup.PendulumSwingUp,
            'robot_cls': sim.gazebo.pendulum.PendulumGazeboRobot,
            'model': "Pendulum/Pendulum.sdf",
            'world': "DefaultEmptyWorld.world",
            'rtf': max_float,
            'agent_rate': 1000,
            'physics_rate': 1000,
            'hard_reset': True,
            })

register(
    id='CartPoleDiscrete-Gazebo-v0',
    entry_point='gym_ignition.runtimes.gazebo_runtime:GazeboRuntime',
    max_episode_steps=5000,
    kwargs={'task_cls': cartpole_discrete.CartPoleDiscrete,
            'robot_cls': sim.gazebo.cartpole.CartPoleGazeboRobot,
            'model': "CartPole/CartPole.sdf",
            'world': "DefaultEmptyWorld.world",
            'rtf': max_float,
            'agent_rate': 1000,
            'physics_rate': 1000,
            'hard_reset': True,
            })

register(
    id='CartPoleContinuous-Gazebo-v0',
    entry_point='gym_ignition.runtimes.gazebo_runtime:GazeboRuntime',
    max_episode_steps=5000,
    kwargs={'task_cls': cartpole_continuous.CartPoleContinuous,
            'robot_cls': sim.gazebo.cartpole.CartPoleGazeboRobot,
            'model': "CartPole/CartPole.sdf",
            'world': "DefaultEmptyWorld.world",
            'rtf': max_float,
            'agent_rate': 1000,
            'physics_rate': 1000,
            'hard_reset': True,
            })

# =====================
# PYBULLET ENVIRONMENTS
# =====================

register(
    id='Pendulum-PyBullet-v0',
    entry_point='gym_ignition.runtimes.pybullet_runtime:PyBulletRuntime',
    max_episode_steps=5000,
    kwargs={'task_cls': pendulum_swingup.PendulumSwingUp,
            'robot_cls': sim.pybullet.pendulum.PendulumPyBulletRobot,
            'model': "Pendulum/Pendulum.urdf",
            'world': "plane_implicit.urdf",
            'rtf': max_float,
            'agent_rate': 1000,
            'physics_rate': 1000,
            'hard_reset': True,
            })

register(
    id='CartPoleDiscrete-PyBullet-v0',
    entry_point='gym_ignition.runtimes.pybullet_runtime:PyBulletRuntime',
    max_episode_steps=5000,
    kwargs={
            # PyBulletRuntime
            'task_cls': cartpole_discrete.CartPoleDiscrete,
            'robot_cls': sim.pybullet.cartpole.CartPolePyBulletRobot,
            'model': "CartPole/CartPole.urdf",
            'world': "plane_implicit.urdf",
            'rtf': max_float,
            'agent_rate': 1000,
            'physics_rate': 1000,
            'hard_reset': True,
            })
