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
        logger.warn("The path '{}' does not exist. Not added to the data path.".format(
            data_path))
        return

    global GYM_IGNITION_DATA_PATH

    for path in GYM_IGNITION_DATA_PATH:
        if path == data_path:
            logger.debug("The path '{}' is already present in the data path".format(
                data_path))
            return

    logger.debug("Adding new search path: '{}'".format(data_path))
    GYM_IGNITION_DATA_PATH.append(data_path)


def add_path_from_env_var(env_variable: str) -> None:
    if not env_variable in os.environ:
        logger.warn("Failed to find '{}' environment variable".format(env_variable))
        return

    env_var_paths = os.environ[env_variable].split(":")

    for path in env_var_paths:
        add_path(path)


def find_resource(file_name: str) -> str:
    file_abs_path = ""
    global GYM_IGNITION_DATA_PATH

    logger.debug("Looking for file '{}'".format(file_name))

    # Handle if the path is absolute
    if os.path.isabs(file_name):
        if isfile(file_name):
            logger.debug("  Found resource: '{}'".format(file_name))
            return file_name
        else:
            raise Exception("Failed to find resource '{}'".format(file_name))

    # Handle if the path is relative
    for path in GYM_IGNITION_DATA_PATH:
        logger.debug("  Exploring folder '{}'".format(path))
        path_with_slash = path if path[-1] is '/' else path + "/"
        candidate_abs_path = path_with_slash + file_name

        if isfile(candidate_abs_path):
            logger.debug("  Found resource: '{}'".format(candidate_abs_path))
            file_abs_path = candidate_abs_path
            break

    if not file_abs_path:
        raise Exception("Failed to find resource '{}'".format(file_name))

    return file_abs_path
