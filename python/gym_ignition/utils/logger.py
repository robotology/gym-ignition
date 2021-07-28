# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import contextlib
import warnings

import gym
from gym import logger
from gym.logger import debug, error, info
from gym.utils import colorize


def custom_formatwarning(msg, *args, **kwargs):
    """
    Custom format that overrides :py:func:`warnings.formatwarning`.
    """

    if logger.MIN_LEVEL is logger.DEBUG:
        warning = "{}:{} {}: {}\n".format(args[1], args[2], args[0].__name__, msg)
    else:
        warning = "{}\n".format(msg)

    return warning


def warn(msg: str, *args) -> None:
    """
    Custom definition of :py:func:`gym.logger.warn` function.
    """

    if logger.MIN_LEVEL <= logger.WARN:
        warnings.warn(colorize("%s: %s" % ("WARN", msg % args), "yellow"), stacklevel=2)


# Monkey patch warning formatting
warnings.formatwarning = custom_formatwarning


def set_level(level: int, scenario_level: int = None) -> None:
    """
    Set the verbosity level of both :py:mod:`gym` and :py:mod:`gym_ignition`.

    Accepted values:

    - :py:const:`gym.logger.DEBUG` (10)
    - :py:const:`gym.logger.INFO` (20)
    - :py:const:`gym.logger.WARN` (30)
    - :py:const:`gym.logger.ERROR` (40)
    - :py:const:`gym.logger.DISABLED` (50)

    Args:
        level: The desired verbosity level.
        scenario_level: The desired ScenarIO verbosity level (defaults to ``level``).
    """

    # Set the gym verbosity
    logger.set_level(level)

    try:
        from scenario import gazebo as scenario
    except ImportError:
        return

    if scenario_level is None:
        scenario_level = level

    # Set the ScenarI/O verbosity
    if scenario_level <= logger.DEBUG:
        scenario.set_verbosity(scenario.Verbosity_debug)
    elif scenario_level <= logger.INFO:
        scenario.set_verbosity(scenario.Verbosity_info)
    elif scenario_level <= logger.WARN:
        scenario.set_verbosity(scenario.Verbosity_warning)
    elif scenario_level <= logger.ERROR:
        scenario.set_verbosity(scenario.Verbosity_error)
    else:
        scenario.set_verbosity(scenario.Verbosity_suppress_all)


@contextlib.contextmanager
def gym_verbosity(level: int):

    old_level = gym.logger.MIN_LEVEL
    gym.logger.set_level(level=level)
    yield None
    gym.logger.set_level(old_level)
