# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from typing import Dict, List, NewType, Tuple, Union

import gym.spaces
import numpy as np

Done = NewType("Done", bool)
Info = NewType("Info", Dict)
Reward = NewType("Reward", float)
Observation = NewType("Observation", np.ndarray)
Action = NewType("Action", Union[np.ndarray, np.number])

SeedList = NewType("SeedList", List[int])

State = NewType("State", Tuple[Observation, Reward, Done, Info])

ActionSpace = NewType("ActionSpace", gym.spaces.Space)
ObservationSpace = NewType("ObservationSpace", gym.spaces.Space)
