# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

include(CMakeParseArguments)

macro(add_new_build_mode)

# ===================
# PARSE THE ARGUMENTS
# ===================

set(_options)
set(_oneValueArgs NAME TEMPLATE)
set(_multiValueArgs)
set(PREFIX "anbm")
string(TOUPPER ${PREFIX} PREFIX)

cmake_parse_arguments(${PREFIX}
    "${_options}"
    "${_oneValueArgs}"
    "${_multiValueArgs}"
    "${ARGN}")

unset(_options)
unset(_oneValueArgs)
unset(_multiValueArgs)

# ==================================================
# ADD THE NEW BUILD MODE TO THE CMAKE CACHE VARIABLE
# ==================================================

# Get all the current build types
get_property(${PREFIX}_all_types CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS)

# Add the new one
list(APPEND ${PREFIX}_all_types ${${PREFIX}_NAME})
list(REMOVE_DUPLICATES ${PREFIX}_all_types)

# Update the displayed list in ccmake
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${${PREFIX}_all_types})
unset(${PREFIX}_all_types)

# ================================
# COPY THE FLAGS FROM THE TEMPLATE
# ================================

string(TOUPPER ${${PREFIX}_NAME} ${PREFIX}_NAME)
string(TOUPPER ${${PREFIX}_TEMPLATE} ${PREFIX}_TEMPLATE)

# https://gitlab.kitware.com/cmake/community/wikis/FAQ#how-can-i-extend-the-build-modes-with-a-custom-made-one-
set(CMAKE_CXX_FLAGS_${${PREFIX}_NAME}
    "${CMAKE_CXX_FLAGS_${${PREFIX}_TEMPLATE}}" CACHE
    STRING "Flags used by the C++ compiler during ${${PREFIX}_NAME} builds."
    FORCE)
set(CMAKE_C_FLAGS_${${PREFIX}_NAME}
    "${CMAKE_C_FLAGS_${${PREFIX}_TEMPLATE}}" CACHE
    STRING "Flags used by the C compiler during ${${PREFIX}_NAME} builds."
    FORCE)
set(CMAKE_EXE_LINKER_FLAGS_${${PREFIX}_NAME}
    "${CMAKE_EXE_LINKER_FLAGS_${${PREFIX}_TEMPLATE}}" CACHE
    STRING "Flags used for linking binaries during ${${PREFIX}_NAME} builds."
    FORCE)
set(CMAKE_SHARED_LINKER_FLAGS_${${PREFIX}_NAME}
    "${CMAKE_SHARED_LINKER_FLAGS_${${PREFIX}_TEMPLATE}}" CACHE
    STRING "Flags used by the shared libraries linker during ${${PREFIX}_NAME} builds."
    FORCE)

mark_as_advanced(
    CMAKE_CXX_FLAGS_${${PREFIX}_NAME}
    CMAKE_C_FLAGS_${${PREFIX}_NAME}
    CMAKE_EXE_LINKER_FLAGS_${${PREFIX}_NAME}
    CMAKE_SHARED_LINKER_FLAGS_${${PREFIX}_NAME})

unset(${PREFIX}_NAME})
unset(${PREFIX}_TEMPLATE})
unset(${PREFIX}_UNPARSED_ARGUMENTS})
unset(PREFIX)

endmacro()
