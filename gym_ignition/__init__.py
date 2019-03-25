from gym.envs.registration import register
from gym import logger
from gym_ignition.envs.ignition_env import IgnitionEnv

# Set gym verbosity
logger.set_level(logger.ERROR)

register(
    id='CartPoleIgnition-v0',
    entry_point='gym_ignition.envs.cartpole:CartPoleEnv')
