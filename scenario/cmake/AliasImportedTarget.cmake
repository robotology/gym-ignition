# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

macro(alias_imported_target)

    set(_prefix "ait")
    string(TOUPPER ${_prefix} _prefix)

    set(_oneValueArgs
        PACKAGE_ORIG
        PACKAGE_DEST
        NAMESPACE_ORIG
        NAMESPACE_DEST
        REQUIRED)

    set(_multiValueArgs
        COMPONENTS
        TARGETS_ORIG
        TARGETS_DEST)

    set(_options)

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

    if(${${_prefix}_REQUIRED})
        find_package(
            ${${_prefix}_PACKAGE_ORIG}
            COMPONENTS ${${_prefix}_COMPONENTS}
            REQUIRED)
    else()
        find_package(
            ${${_prefix}_PACKAGE_ORIG}
            COMPONENTS ${${_prefix}_COMPONENTS}
            QUIET)
    endif()

    # Example:
    #  ${${_prefix}_PACKAGE_ORIG} = ign-gazebo3
    if(${${_prefix}_PACKAGE_ORIG}_FOUND)

        message(DEBUG "Processing package: ${${_prefix}_PACKAGE_ORIG}")

        # Check length of lists
        list(LENGTH ${_prefix}_TARGETS_ORIG _num_targets_orig)
        list(LENGTH ${_prefix}_TARGETS_DEST _num_targets_dest)
        math(EXPR _comparison "${_num_targets_orig} - ${_num_targets_dest}"
             OUTPUT_FORMAT DECIMAL)

         if(${_num_targets_orig} STREQUAL 0)
             message(FATAL_ERROR "No targets passed")
         endif()

        if(NOT ${_comparison} STREQUAL 0)
            message(FATAL_ERROR "Number or TARGETS_ elements do not match")
        endif()

        # Example:
        # ${_package_name} = ignition-gazebo3
        set(_package_name ${${_prefix}_PACKAGE_ORIG})

        # Example:
        # ${_variable_name} = ignition-gazebo
        set(_variable_name ${${_prefix}_PACKAGE_DEST})

        message(DEBUG "  Setting: ${_variable_name}=${_package_name}")
        set(${_variable_name} ${_package_name})

        # TODO Use ZIP_LISTS with CMake > 3.17
        math(EXPR _num_targets "${_num_targets_orig} - 1")
        foreach(idx RANGE ${_num_targets})

            list(GET ${_prefix}_TARGETS_ORIG ${idx} _target_orig)
            list(GET ${_prefix}_TARGETS_DEST ${idx} _target_dest)

            # Example:
            # ${_target_name} = ignition-gazebo3::core
            set(_target_name ${${_prefix}_NAMESPACE_ORIG}::${_target_orig})

            # Example:
            # ${_target_name} = ignition-gazebo.core
            set(_variable_name ${${_prefix}_NAMESPACE_DEST}.${_target_dest})

            if(NOT TARGET ${_target_name})
                message(FATAL_ERROR "Could not find target ${_target_name}")
            endif()

            message(DEBUG "  Setting: ${_variable_name}=${_target_name}")
            set(${_variable_name} ${_target_name})

        endforeach()

    endif()

    unset(_prefix)
    unset(_num_targets_orig)
    unset(_num_targets_dest)
    unset(_comparison)
    unset(_package_name)
    unset(_num_targets)
    unset(_target_orig)
    unset(_target_dest)
    unset(_target_name)
    unset(_variable_name)

endmacro()
