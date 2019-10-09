# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import warnings
from gym import logger
from gym.utils import colorize
from gym.logger import debug, info, error

try:
    from gym_ignition import gympp_bindings as bindings
    bindings_found = True
except:
    bindings_found = False


def custom_formatwarning(msg, *args, **kwargs):
    if logger.MIN_LEVEL is logger.DEBUG:
        warning = "{}:{} {}: {}\n".format(args[1], args[2], args[0].__name__, msg)
    else:
        warning = "{}\n".format(msg)

    return warning


def warn(msg: str, *args) -> None:
    if logger.MIN_LEVEL <= logger.WARN:
        warnings.warn(colorize('%s: %s' % ('WARN', msg % args), 'yellow'), stacklevel=2)


# Monkey patch warning formatting
warnings.formatwarning = custom_formatwarning


def set_level(level: int) -> None:
    # Set the gym verbosity
    logger.set_level(level)

    if not bindings_found:
        return

    # Set the gympp verbosity
    if logger.MIN_LEVEL <= logger.DEBUG:
        bindings.GazeboWrapper.setVerbosity(4)
    elif logger.MIN_LEVEL <= logger.INFO:
        bindings.GazeboWrapper.setVerbosity(3)
    elif logger.MIN_LEVEL <= logger.WARN:
        bindings.GazeboWrapper.setVerbosity(2)
    elif logger.MIN_LEVEL <= logger.ERROR:
        bindings.GazeboWrapper.setVerbosity(1)
    else:
        raise Exception("Verbosity level not recognized")
