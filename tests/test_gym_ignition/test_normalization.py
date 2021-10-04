# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest

pytestmark = pytest.mark.gym_ignition

from gym_ignition.utils.math import denormalize, normalize

test_matrix = [
    # Float, None
    (1, None, None, 1),
    (1, 0, None, 1),
    (1, None, 0, 1),
    # Float / Float
    (0, -1, 1, 0),
    (-1, -1, 1, -1),
    (1, -1, 1, 1),
    # List / Float
    ([-1, 0, 1, 2], -2, 2, [-0.5, 0, 0.5, 1]),
    ([-1, 0, 1, 2], -2.0, 2.0, [-0.5, 0, 0.5, 1]),
    ([-1.0, 0, 1, 2], -2, 2, [-0.5, 0, 0.5, 1]),
    ([-1.0, 0, 1, 2], 1, 1, [-1.0, 0, 1, 2]),
    # List / List
    ([-1, 0, 2.0], [-1, -2, 1], [-1, 4, 3], [-1, -0.3333333, 0]),
]


@pytest.mark.parametrize("input,low, high, output", test_matrix)
def test_normalization(input, low, high, output):

    normalized_input = normalize(input=input, low=low, high=high)

    assert output == pytest.approx(normalized_input)
    assert input == pytest.approx(
        denormalize(input=normalized_input, low=low, high=high)
    )
