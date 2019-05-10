from gym import logger
from gym.logger import debug, info, warn, error
from gym_ignition import gympp


def set_level(level):
    # Set the gym verbosity
    logger.set_level(level)

    # Set the gympp verbosity
    if logger.MIN_LEVEL <= logger.DEBUG:
        gympp.GazeboWrapper.setVerbosity(4)
    elif logger.MIN_LEVEL <= logger.INFO:
        gympp.GazeboWrapper.setVerbosity(3)
    elif logger.MIN_LEVEL <= logger.WARN:
        gympp.GazeboWrapper.setVerbosity(2)
    elif logger.MIN_LEVEL <= logger.ERROR:
        gympp.GazeboWrapper.setVerbosity(1)
    else:
        raise Exception("Verbosity level not recognized")



