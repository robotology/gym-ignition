# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import os
import gym.logger
from pathlib import Path
from gym_ignition.utils import logger


def setup_environment() -> None:
    """
    Configure the environment depending on the detected installation method
    (User or Developer).
    """

    # Make sure that the dot folder in the user's home exists
    Path("~/.ignition/gazebo").expanduser().mkdir(mode=0o755,
                                                  parents=True,
                                                  exist_ok=True)

    # Configure the environment
    ign_gazebo_system_plugin_path = ""

    if "IGN_GAZEBO_SYSTEM_PLUGIN_PATH" in os.environ:
        ign_gazebo_system_plugin_path = os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH")

    # Get the install prefix from C++. It is defined only in Developer mode.
    from gym_ignition import scenario_bindings
    install_prefix = scenario_bindings.get_install_prefix()

    if install_prefix != "":
        detected_mode = "Developer"
    else:
        detected_mode = "User"

    logger.debug(f"{detected_mode} setup")

    # Configure verbosity
    if detected_mode == "Developer":
        logger.set_level(gym.logger.INFO)
    else:
        logger.set_level(gym.logger.WARN)

    # Add the plugins path
    if detected_mode == "Developer":
        logger.debug(f"Detected install prefix: '{install_prefix}'")
        ign_gazebo_system_plugin_path += f":{install_prefix}/lib/gympp/plugins"
        ign_gazebo_system_plugin_path += f":{install_prefix}/lib/scenario/plugins"
    else:
        import gym_ignition
        ign_gazebo_system_plugin_path += f":{gym_ignition.__path__[0]}/plugins"

    os.environ["IGN_GAZEBO_SYSTEM_PLUGIN_PATH"] = ign_gazebo_system_plugin_path
    logger.debug(f"IGN_GAZEBO_SYSTEM_PLUGIN_PATH={ign_gazebo_system_plugin_path}")
