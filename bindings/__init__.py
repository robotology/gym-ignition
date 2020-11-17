# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import os
import sys
from pathlib import Path
from enum import auto, Enum


class InstallMode(Enum):
    User = auto()
    Developer = auto()


def detect_install_mode() -> InstallMode:

    import scenario.bindings.core
    install_prefix = scenario.bindings.core.get_install_prefix()
    return InstallMode.User if install_prefix == "" else InstallMode.Developer


def setup_gazebo_environment() -> None:

    import scenario.bindings.core

    # Configure the environment
    ign_gazebo_system_plugin_path = ""

    if "IGN_GAZEBO_SYSTEM_PLUGIN_PATH" in os.environ:
        ign_gazebo_system_plugin_path = os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH")

    # Add the plugins path
    if detect_install_mode() == InstallMode.Developer:
        install_prefix = Path(scenario.bindings.core.get_install_prefix())
    else:
        install_prefix = Path(os.path.dirname(__file__))

    plugin_dir = install_prefix / "lib" / "scenario" / "plugins"
    ign_gazebo_system_plugin_path += f":{str(plugin_dir)}"

    os.environ["IGN_GAZEBO_SYSTEM_PLUGIN_PATH"] = ign_gazebo_system_plugin_path


def import_gazebo() -> None:

    # Check the the module was never loaded by someone else
    if "scenario.bindings._gazebo" in sys.modules:
        raise ImportError("Failed to load ScenarI/O Gazebo with custom dlopen flags")

    # Import SWIG bindings
    # See https://github.com/robotology/gym-ignition/issues/7
    #     https://stackoverflow.com/a/45473441/12150968
    if sys.platform.startswith('linux') or sys.platform.startswith('darwin'):

        # Update the dlopen flags
        dlopen_flags = sys.getdlopenflags()
        sys.setdlopenflags(dlopen_flags | os.RTLD_GLOBAL)

        import scenario.bindings.gazebo

        # Restore the flags
        sys.setdlopenflags(dlopen_flags)

    else:
        import scenario.bindings.gazebo


def create_home_dot_folder() -> None:

    # Make sure that the dot folder in the user's home exists
    Path("~/.ignition/gazebo").expanduser().mkdir(mode=0o755,
                                                  parents=True,
                                                  exist_ok=True)

# ===================
# Import the bindings
# ===================

try:
    import_gazebo()
    create_home_dot_folder()
    setup_gazebo_environment()
    from .bindings import gazebo
except ImportError:
    pass

try:
    from .bindings.yarp import yarp
except ImportError:
    pass

from .bindings import core
