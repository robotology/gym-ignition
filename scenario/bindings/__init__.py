# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import os
import sys
from enum import Enum, auto
from pathlib import Path

import packaging.specifiers
import packaging.version


def supported_versions_specifier_set() -> packaging.specifiers.SpecifierSet:

    # If 6 is the Ignition distribution major version, the following specifier enables
    # the compatibility with all the following versions:
    #
    # 6.Y.Z.devK
    # 6.Y.Z.alphaK
    # 6.Y.Z.betaK
    # 6.Y.Z.rcK
    # 6.Y.Z.preK
    # 6.Y.Z.postK
    #
    return packaging.specifiers.SpecifierSet("~=6.0.0.dev")


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


def preload_tensorflow_shared_libraries() -> None:

    # Check if tensorflow is installed
    import importlib.util

    spec = importlib.util.find_spec("tensorflow")

    if spec is None:
        return

    # Get the tensorflow __init__ location
    import pathlib

    init = pathlib.Path(spec.origin)

    # Get the tensorflow top-level folder
    tensorflow_dir = init.parent
    assert tensorflow_dir.is_dir()

    # Get the tensorflow/python folder
    tensorflow_python_dir = tensorflow_dir / "python"
    assert tensorflow_python_dir.is_dir()

    # Load the main shared library
    for lib in tensorflow_dir.glob("*tensorflow*.so*"):
        import ctypes

        ctypes.CDLL(str(lib))

    # Load all the shared libraries inside tensorflow/python
    for lib in tensorflow_python_dir.glob("_*.so"):
        import ctypes

        ctypes.CDLL(str(lib))


def pre_import_gym() -> None:

    # Check if gym is installed
    import importlib.util

    spec = importlib.util.find_spec("gym")

    if spec is None:
        return

    import gym


def check_gazebo_installation() -> None:

    import subprocess

    try:
        command = ["ign", "gazebo", "--versions"]
        result = subprocess.run(command, capture_output=True, text=True, check=True)
    except FileNotFoundError:
        msg = "Failed to find the 'ign' command in your PATH. "
        msg += "Make sure that Ignition is installed "
        msg += "and your environment is properly configured."
        raise RuntimeError(msg)
    except subprocess.CalledProcessError:
        raise RuntimeError(f"Failed to execute command: {' '.join(command)}")  # noqa

    # Strip the command output
    gazebo_versions_string = result.stdout.strip()

    # Get the gazebo version from the command line.
    # Since the releases could be in the "6.0.0~preK" form, we replace '~' with '.' to
    # be compatible with the 'packaging' package.
    gazebo_version_string_normalized = gazebo_versions_string.replace("~", ".")

    # The output could be multiline, listing all the Ignition Gazebo versions found
    gazebo_versions = gazebo_version_string_normalized.split(sep=os.linesep)

    try:
        # Parse the gazebo versions
        gazebo_versions_parsed = [packaging.version.Version(v) for v in gazebo_versions]
    except:
        raise RuntimeError(
            f"Failed to parse the output of: {' '.join(command)} ({gazebo_versions})"
        )

    for version in gazebo_versions_parsed:
        if version in supported_versions_specifier_set():
            return

    msg = f"Failed to find Ignition Gazebo {supported_versions_specifier_set()} "
    msg += f"(found incompatible version(s): {gazebo_versions_parsed})"
    raise RuntimeError(msg)


def import_gazebo() -> None:

    # Check the the module was never loaded by someone else
    if "scenario.bindings._gazebo" in sys.modules:
        raise ImportError("Failed to load ScenarIO Gazebo with custom dlopen flags")

    # Preload the shared libraries of tensorflow if the package is installed.
    # If tensorflow is imported after scenario.bindings.gazebo, the application segfaults.
    if os.environ.get("SCENARIO_DISABLE_TENSORFLOW_PRELOAD") != "1":
        preload_tensorflow_shared_libraries()

    # Import gym before scenario.bindings.gazebo. Similarly to tensorflow, also gym
    # includes a module that imports protobuf, producing a similar segfault.
    if os.environ.get("SCENARIO_DISABLE_GYM_PREIMPORT") != "1":
        pre_import_gym()

    # Import SWIG bindings
    # See https://github.com/robotology/gym-ignition/issues/7
    #     https://stackoverflow.com/a/45473441/12150968
    if sys.platform.startswith("linux") or sys.platform.startswith("darwin"):

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
    Path("~/.ignition/gazebo").expanduser().mkdir(
        mode=0o755, parents=True, exist_ok=True
    )


# ===================
# Import the bindings
# ===================

# Find the _gazebo.* shared lib
if len(list((Path(__file__).parent / "bindings").glob(pattern="_gazebo.*"))) == 1:

    check_gazebo_installation()
    import_gazebo()
    create_home_dot_folder()
    setup_gazebo_environment()
    from .bindings import gazebo

# Find the _yarp.* shared lib
if len(list((Path(__file__).parent / "bindings").glob(pattern="_yarp.*"))) == 1:
    from .bindings.yarp import yarp

from .bindings import core
