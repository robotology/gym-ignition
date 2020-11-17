# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import numpy as np
from numbers import Number
from typing import Union, List
from scenario import gazebo as scenario


def normalize(input: Union[Number, List[Number], np.ndarray],
              low: Union[Number, List[Number], np.ndarray],
              high: Union[Number, List[Number], np.ndarray]) -> Union[Number, np.ndarray]:

    if low is None or high is None:
        return input

    if isinstance(input, Number):
        input = [input]

    if isinstance(low, Number):
        low = [low]

    if isinstance(high, Number):
        high = [high]

    output = scenario.normalize(list(input), list(low), list(high))

    if len(output) == 1:
        return output[0]
    else:
        return np.array(output)


def denormalize(input: Union[Number, List[Number], np.ndarray],
                low: Union[Number, List[Number], np.ndarray],
                high: Union[Number, List[Number], np.ndarray]) \
        -> Union[Number, np.ndarray]:

    if low is None or high is None:
        return input

    if isinstance(input, Number):
        input = [input]

    if isinstance(low, Number):
        low = [low]

    if isinstance(high, Number):
        high = [high]

    output = scenario.denormalize(list(input), list(low), list(high))

    if len(output) == 1:
        return output[0]
    else:
        return np.array(output)
