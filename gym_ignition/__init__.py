from gym.envs.registration import register

register(
    id='ignition-v0',
    entry_point='gym_ignition.envs:IgnitionEnv',
)
