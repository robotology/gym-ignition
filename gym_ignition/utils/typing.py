import numpy as np
from typing import List, Tuple, Dict, Union, NewType

Action = NewType('Action', Union[np.ndarray, np.number])
Observation = NewType('Observation', np.ndarray)
Reward = NewType('Reward', float)

SeedList = NewType('SeedList', List[int])

State = NewType('State', Tuple[Observation, Reward, bool, Dict])
