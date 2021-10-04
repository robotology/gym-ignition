# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# In this initial implementation, we support only specifying
# the Ignition Gazebo version to determine the Ignition distribution.
# In the future we could pull and parse the tags.yaml file.
set(IGNITION-GAZEBO_CITADEL_VER 3)
set(IGNITION-GAZEBO_DOME_VER 4)
set(IGNITION-GAZEBO_EDIFICE_VER 5)
set(IGNITION-GAZEBO_FORTRESS_VER 6)

macro(find_ignition_distribution)

    set(_prefix "fid")
    string(TOUPPER ${_prefix} _prefix)

    set(_oneValueArgs
        CODENAME
        REQUIRED)

    set(_multiValueArgs
        PACKAGES)

    # For each {one,multi}value variable FOO, this command creates
    # the variable ${_prefix}_FOO with the passed arg(s)
    cmake_parse_arguments(${_prefix}
        "${_options}"
        "${_oneValueArgs}"
        "${_multiValueArgs}"
        "${ARGN}")

    unset(_options)
    unset(_oneValueArgs)
    unset(_multiValueArgs)

    # Example:
    # ${Citadel_FOUND}=TRUE
    set(${${_prefix}_CODENAME}_FOUND TRUE)
    set(IGNITION_FOUND FALSE)

    set(_pkgs_not_found)

    # Example:
    # ${PKG}=ignition-gazebo
    foreach(PKG IN LISTS ${_prefix}_PACKAGES)

        # Example:
        # ${_codename_upper}=CITADEL
        string(TOUPPER ${${_prefix}_CODENAME} _codename_upper)

        # Example:
        # ${_pkg_upper}=IGNITION-GAZEBO
        string(TOUPPER ${PKG} _pkg_upper)

        # Example:
        # ${_rel_variable_name}=IGNITION-GAZEBO_CITADEL_VER
        set(_rel_variable_name ${_pkg_upper}_${_codename_upper}_VER)

        if(NOT ${_rel_variable_name})
            message(FATAL_ERROR "Failed to find variable ${_rel_variable_name}")
        endif()

        # Example:
        # ${_pkg_name}=ignition-gazebo3
        set(_pkg_name ${PKG}${${_rel_variable_name}})

        # Find the package
        if(${${_prefix}_REQUIRED})
            find_package(${_pkg_name} REQUIRED)
        else()
            find_package(${_pkg_name} QUIET)
        endif()

        if(NOT ${_pkg_name}_FOUND)
            # Example:
            # ${Citadel_FOUND}=FALSE
            set(${${_prefix}_CODENAME}_FOUND FALSE)
            list(APPEND _pkgs_not_found ${_pkg_name})
        endif()

    endforeach()

    # Print missing packages
    if(NOT ${${_prefix}_CODENAME}_FOUND)
        message(DEBUG "Missing packages of Ignition ${${_prefix}_CODENAME}:")
        foreach(PKG IN LISTS _pkgs_not_found)
            message(DEBUG "  ${PKG}")
        endforeach()
    endif()

    if(${${_prefix}_CODENAME}_FOUND)
        set(IGNITION_FOUND TRUE)
    endif()

    unset(_prefix)
    unset(_codename_upper)
    unset(_pkg_upper)
    unset(_rel_variable_name)
    unset(_pkg_name)
    unset(_pkgs_not_found)

endmacro()
