# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from typing import Dict, List, NewType, Tuple, Union

import gym.spaces
import numpy as np

Terminated = NewType("Terminated", bool)
Done = NewType("Done", bool)
Truncated = NewType("Truncated", bool)
Info = NewType("Info", Dict)
Reward = NewType("Reward", float)
Observation = NewType("Observation", np.ndarray)
ResetReturn = NewType("ResetReturn", Tuple[Observation, Info])
Action = NewType("Action", Union[np.ndarray, np.number])

SeedList = NewType("SeedList", List[int])

State = NewType("State", Tuple[Observation, Reward, Terminated, Truncated, Info])

ActionSpace = NewType("ActionSpace", gym.spaces.Space)
ObservationSpace = NewType("ObservationSpace", gym.spaces.Space)
