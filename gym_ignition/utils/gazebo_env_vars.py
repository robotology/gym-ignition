# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import os
from pathlib import Path
from gym_ignition.utils import logger, misc


def setup_gazebo_env_vars() -> None:
    # Configure the environment
    ign_gazebo_resource_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH")
    ign_gazebo_system_plugin_path = os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH")

    if not ign_gazebo_resource_path:
        ign_gazebo_resource_path = ""

    if not ign_gazebo_system_plugin_path:
        ign_gazebo_system_plugin_path = ""

    if misc.installed_in_editable_mode():
        logger.debug("Developer setup")

        # Detect the install prefix
        import gympp_bindings
        site_packages_path = Path(gympp_bindings.__file__).parent
        install_prefix = site_packages_path.parent.parent.parent
        logger.debug(f"Detected install prefix: '{install_prefix}'")

        # Add the plugins path
        ign_gazebo_system_plugin_path += f":{install_prefix}/lib/gympp/plugins"

        # Add the worlds and models path
        ign_gazebo_resource_path += f":{install_prefix}/share/gympp/gazebo/worlds"
        ign_gazebo_resource_path += f":{install_prefix}/share/gympp/gazebo/models"
    else:
        logger.debug("User setup")

        # Add the plugins path
        import gym_ignition
        ign_gazebo_system_plugin_path += f":{gym_ignition.__path__[0]}/plugins"

        # Add the worlds and models path
        import gym_ignition_data
        data_path = gym_ignition_data.__path__[0]
        ign_gazebo_resource_path += f":{data_path}:/{data_path}/worlds"

    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = ign_gazebo_resource_path
    os.environ["IGN_GAZEBO_SYSTEM_PLUGIN_PATH"] = ign_gazebo_system_plugin_path
    logger.debug(f"IGN_GAZEBO_RESOURCE_PATH={ign_gazebo_resource_path}")
    logger.debug(f"IGN_GAZEBO_SYSTEM_PLUGIN_PATH={ign_gazebo_system_plugin_path}")
