# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# Look for an executable called sphinx-multiversion
find_program(SPHINX_MULTIVERSION_EXECUTABLE
             NAMES sphinx-multiversion
             DOC "Path to sphinx-multiversion executable")

include(FindPackageHandleStandardArgs)

# Handle standard arguments to find_package like REQUIRED and QUIET
find_package_handle_standard_args(SphinxMultiVersion
                                  "Failed to find sphinx-multiversion executable"
                                  SPHINX_MULTIVERSION_EXECUTABLE)
