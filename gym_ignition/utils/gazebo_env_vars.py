# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import os
from gym_ignition.utils import logger


def setup_gazebo_env_vars() -> None:
    """
    Configure the environment depending on the detected installation method
    (User or Developer).
    """

    # Configure the environment
    ign_gazebo_system_plugin_path = ""

    if "IGN_GAZEBO_SYSTEM_PLUGIN_PATH" in os.environ:
        ign_gazebo_system_plugin_path = os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH")

    # Get the install prefix from C++. It is defined only in Developer mode.
    from gym_ignition import scenario_bindings
    install_prefix = scenario_bindings.getInstallPrefix()

    if install_prefix != "":
        detected_mode = "Developer"
    else:
        detected_mode = "User"

    logger.debug(f"{detected_mode} setup")

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
