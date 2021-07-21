# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# Useful folders:
#
# 1. The folder containing the Python packages
# 2. The folder containing the bindings Python modules
#
set(PYTHON_PACKAGES_DIR "${CMAKE_SOURCE_DIR}/python")
set(BINDINGS_MODULES_DIR "${PROJECT_BINARY_DIR}/scenario/bindings")

# =============
# APIDOC TARGET
# =============

# This target generates the apidoc files of the Python modules and stores
# them in the docs/sphinx/ source folder.
# In order to make the multiversion website work, they need to be committed
# when they change.
#
# The files in the following folders are replaced:
#
# - docs/sphinx/scenario
# - docs/sphinx/gym_ignition
# - docs/sphinx/gym_ignition_environments

if (NOT TARGET ScenarioSwig::Core)
    message(FATAL_ERROR "Target ScenarioSwig::Core not found")
endif()

if (NOT TARGET ScenarioSwig::Gazebo)
    message(FATAL_ERROR "Target ScenarioSwig::Gazebo not found")
endif()

find_package(SphinxApidoc REQUIRED)
add_custom_target(apidoc ALL DEPENDS
    doxygen ScenarioSwig::Core ScenarioSwig::Gazebo)

add_custom_command(
    TARGET apidoc POST_BUILD
    COMMAND
    ${SPHINX_APIDOC_EXECUTABLE} --force
    --no-toc --module-first --maxdepth 4
    -t ${CMAKE_CURRENT_SOURCE_DIR}/_templates
    -o ${CMAKE_CURRENT_SOURCE_DIR}/apidoc/gym-ignition
    ${PYTHON_PACKAGES_DIR}/gym_ignition
    COMMENT "Building <gym-ignition> apidoc")

add_custom_command(
    TARGET apidoc POST_BUILD
    COMMAND
    ${SPHINX_APIDOC_EXECUTABLE} --force
    --no-toc --module-first --maxdepth 4
    -t ${CMAKE_CURRENT_SOURCE_DIR}/_templates
    -o ${CMAKE_CURRENT_SOURCE_DIR}/apidoc/gym-ignition-environments
    ${PYTHON_PACKAGES_DIR}/gym_ignition_environments
    COMMENT "Building <gym-ignition-environments> apidoc")

add_custom_command(
    TARGET apidoc POST_BUILD
    COMMAND
    ${SPHINX_APIDOC_EXECUTABLE} --force
    --no-toc --module-first --maxdepth 4
    -t ${CMAKE_CURRENT_SOURCE_DIR}/_templates
    -o ${CMAKE_CURRENT_SOURCE_DIR}/apidoc/scenario
    ${BINDINGS_MODULES_DIR}
    COMMENT "Building <scenario> apidoc")

# =============
# SPHINX TARGET
# =============

find_package(Sphinx REQUIRED)
find_package(SphinxMultiVersion REQUIRED)

if(${DOXYGEN_OUTPUT_DIRECTORY} STREQUAL "")
    message(FATAL_ERROR "Doxygen was not configured properly")
endif()

set(SPHINX_BUILD ${CMAKE_BINARY_DIR}/html)
set(SPHINX_SOURCE ${CMAKE_CURRENT_SOURCE_DIR})

# Extend the Python path including before generating the website
set(SPHINX_CMD export PYTHONPATH="${PYTHON_PACKAGES_DIR}:${BINDINGS_MODULES_DIR}:$ENV{PYTHONPATH}" &&)

# Sphinx build command
list(APPEND SPHINX_CMD
    ${SPHINX_MULTIVERSION_EXECUTABLE}
    ${SPHINX_SOURCE} ${SPHINX_BUILD}
    -D breathe_projects.scenario="${DOXYGEN_OUTPUT_DIRECTORY}/xml")

# Generate the website
add_custom_target(sphinx ALL
    DEPENDS apidoc
    COMMAND ${SPHINX_CMD}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Generating documentation with Sphinx")

# Disable GitHub pages autogeneration
add_custom_command(
    TARGET sphinx POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E touch "${SPHINX_BUILD}/.nojekyll"
    COMMENT "Disabling Jekyll in html folder")

# Handle redirect
add_custom_command(
    TARGET sphinx POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/index.html" "${SPHINX_BUILD}"
    COMMENT "Copying html redirect to html folder")
