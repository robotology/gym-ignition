# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

find_package(Doxygen REQUIRED)

set(DOXYGEN_GENERATE_XML YES)
set(DOXYGEN_GENERATE_HTML NO)
set(DOXYGEN_GENERATE_LATEX YES)
set(DOXYGEN_FILE_PATTERNS "*.h")
set(DOXYGEN_BUILTIN_STL_SUPPORT YES)
set(DOXYGEN_EXCLUDE_PATTERNS
    */components/* */plugins/* */details/* */private/*
    */examples/* */tests/* */gympp/*)

doxygen_add_docs(
    doxygen
    ${PROJECT_SOURCE_DIR}
    COMMENT "Generate man pages"
)

# This is required by Sphinx to find Doxygen XMLs
set(DOXYGEN_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR} PARENT_SCOPE)
