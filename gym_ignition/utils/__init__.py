# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from .typing import *
from . import resource_finder

# If not installed in editable mode, insert gym_ignition_data path to the search path
try:
    import gym_ignition_data
    resource_finder.add_path(gym_ignition_data.get_data_path())
    resource_finder.add_path(gym_ignition_data.get_data_path() + "/worlds")
except ModuleNotFoundError:
    pass

import contextlib
