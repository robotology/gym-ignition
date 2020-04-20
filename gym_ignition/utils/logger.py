# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import gym
import warnings
import contextlib
from gym import logger
from gym.utils import colorize
from gym.logger import debug, info, error

try:
    from gym_ignition import scenario_bindings as bindings
    bindings_found = True
except ImportError:
    bindings_found = False


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
        warnings.warn(colorize('%s: %s' % ('WARN', msg % args), 'yellow'), stacklevel=2)


# Monkey patch warning formatting
warnings.formatwarning = custom_formatwarning


def set_level(level: int) -> None:
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
    """

    # Set the gym verbosity
    logger.set_level(level)

    if not bindings_found:
        return

    # Set the gympp verbosity
    if logger.MIN_LEVEL <= logger.DEBUG:
        bindings.setVerbosity(4)
    elif logger.MIN_LEVEL <= logger.INFO:
        bindings.setVerbosity(3)
    elif logger.MIN_LEVEL <= logger.WARN:
        bindings.setVerbosity(2)
    elif logger.MIN_LEVEL <= logger.ERROR:
        bindings.setVerbosity(1)
    else:
        raise Exception("Verbosity level not recognized")


@contextlib.contextmanager
def verbosity(level: int):

    old_level = gym.logger.MIN_LEVEL
    gym.logger.set_level(level=level)
    yield None
    gym.logger.set_level(old_level)