from gym_ignition import IgnitionEnv
from gympp import PluginMetadata, SpaceMetadata, SpaceType_Discrete, SpaceType_Box


class CartPoleEnv(IgnitionEnv):
    def __init__(self):
        IgnitionEnv.__init__(self)

    def _plugin_metadata(self):
        md = PluginMetadata()
        md.setEnvironmentName("CartPole")
        md.setLibraryName("CartPolePlugin")
        md.setClassName("gympp::plugins::CartPole")
        md.setWorldFileName("CartPole.world")
        md.setModelNames(["cartpole_xacro"])

        action_space_md = SpaceMetadata()
        action_space_md.setType(SpaceType_Discrete)
        action_space_md.setDimensions([3])

        observation_space_md = SpaceMetadata()
        observation_space_md.setType(SpaceType_Box)
        observation_space_md.setLowLimit([-360.0, -1.0])
        observation_space_md.setHighLimit([360.0, 1.0])

        md.setActionSpaceMetadata(action_space_md)
        md.setObservationSpaceMetadata(observation_space_md)

        return md
