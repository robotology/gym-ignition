# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import os
from typing import List
from os.path import exists, isfile
from gym_ignition.utils import logger

GYM_IGNITION_DATA_PATH = []


def get_search_paths() -> List[str]:
    global GYM_IGNITION_DATA_PATH
    return GYM_IGNITION_DATA_PATH


def add_path(data_path: str) -> None:
    if not exists(data_path):
        logger.warn(f"The path '{data_path}' does not exist. Not added to the data path.")
        return

    global GYM_IGNITION_DATA_PATH

    for path in GYM_IGNITION_DATA_PATH:
        if path == data_path:
            logger.debug(f"The path '{data_path}' is already present in the data path")
            return

    logger.debug(f"Adding new search path: '{data_path}'")
    GYM_IGNITION_DATA_PATH.append(data_path)


def add_path_from_env_var(env_variable: str) -> None:
    if env_variable not in os.environ:
        logger.warn(f"Failed to find '{env_variable}' environment variable")
        return

    # Get the environment variable
    env_var_content = os.environ[env_variable]

    # Remove leading ':' characters
    if env_var_content[0] == ':':
        env_var_content = env_var_content[1:]

    # Split multiple value
    env_var_paths = env_var_content.split(":")

    for path in env_var_paths:
        add_path(path)


def find_resource(file_name: str) -> str:
    file_abs_path = ""
    global GYM_IGNITION_DATA_PATH

    logger.debug(f"Looking for file '{file_name}'")

    # Handle if the path is absolute
    if os.path.isabs(file_name):
        if isfile(file_name):
            logger.debug(f"  Found resource: '{file_name}'")
            return file_name
        else:
            raise FileNotFoundError(f"Failed to find resource '{file_name}'")

    # Handle if the path is relative
    for path in GYM_IGNITION_DATA_PATH:
        logger.debug(f"  Exploring folder '{path}'")
        path_with_slash = path if path[-1] == '/' else path + "/"
        candidate_abs_path = path_with_slash + file_name

        if isfile(candidate_abs_path):
            logger.debug(f"  Found resource: '{candidate_abs_path}'")
            file_abs_path = candidate_abs_path
            break

    if not file_abs_path:
        raise FileNotFoundError(f"Failed to find resource '{file_name}'")

    return file_abs_path
