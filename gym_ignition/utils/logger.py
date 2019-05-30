from gym import logger
from gym.logger import debug, info, warn, error
from gym_ignition import gympp_bindings as bindings


def set_level(level):
    # Set the gym verbosity
    logger.set_level(level)

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
