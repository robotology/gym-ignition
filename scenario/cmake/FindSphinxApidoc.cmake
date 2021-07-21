# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# Look for an executable called sphinx-apidoc
find_program(SPHINX_APIDOC_EXECUTABLE
             NAMES sphinx-apidoc
             DOC "Path to sphinx-apidoc executable")

include(FindPackageHandleStandardArgs)

# Handle standard arguments to find_package like REQUIRED and QUIET
find_package_handle_standard_args(SphinxApidoc
                                  "Failed to find sphinx-apidoc executable"
                                  SPHINX_APIDOC_EXECUTABLE)
