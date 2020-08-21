# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

include(FetchContent)

# ===
# YCM
# ===

find_package(YCM QUIET)
option(GYMIGNITION_USE_SYSTEM_YCM
    "Use system-installed YCM, rather than a private copy"
    ${YCM_FOUND})

if(NOT ${YCM_FOUND} AND NOT ${GYMIGNITION_USE_SYSTEM_YCM})

    FetchContent_Declare(
        ycm
        GIT_REPOSITORY https://github.com/robotology/ycm.git)

    FetchContent_GetProperties(ycm)

    if(NOT ycm_POPULATED)
        FetchContent_Populate(ycm)

        add_subdirectory(${ycm_SOURCE_DIR}
                         ${ycm_BINARY_DIR}
                         EXCLUDE_FROM_ALL)
    endif()

    set(CMAKE_MODULE_PATH
        "${CMAKE_MODULE_PATH};${ycm_SOURCE_DIR}/modules" PARENT_SCOPE)
else()
    set(CMAKE_MODULE_PATH
        "${CMAKE_MODULE_PATH};${YCM_MODULE_PATH}" PARENT_SCOPE)
endif()

# ====================
# TINY-PROCESS-LIBRARY
# ====================

find_package(tiny-process-library QUIET)
option(GYMIGNITION_USE_SYSTEM_TPL
    "Use system-installed tiny-process-library, rather than a private copy"
    ${tiny-process-library_FOUND})

if(NOT ${tiny-process-library_FOUND} AND NOT ${GYMIGNITION_USE_SYSTEM_TPL})

    FetchContent_Declare(
        tinyprocesslibrary
        GIT_REPOSITORY https://gitlab.com/eidheim/tiny-process-library.git)

    FetchContent_GetProperties(tinyprocesslibrary)

    if(NOT tinyprocesslibrary_POPULATED)
        FetchContent_Populate(tinyprocesslibrary)

        # We don't want to install this library in the system, we instead
        # compile it as an OBJECT library and embed in either the shared or
        # static libraries that need it.
        # From https://gitlab.kitware.com/cmake/cmake/-/issues/18935 it seems
        # that OBJECT libraries that are not installed become INTERFACE when
        # part of an EXPORT set.
        # This behaviour allows setting transitively tiny-process-library infos
        # to the consuming targets while not breaking the EXPORT process. In fact,
        # the conversion to INTERFACE allows to add tiny-process-library to the
        # targets of the EXPORT that contains targets linking against it.
        # See also https://cmake.org/pipermail/cmake/2018-September/068250.html.

        if(WIN32)
            add_library(tiny-process-library OBJECT
                ${tinyprocesslibrary_SOURCE_DIR}/process.cpp
                ${tinyprocesslibrary_SOURCE_DIR}/process_win.cpp)
            #If compiled using MSYS2, use sh to run commands
            if(MSYS)
                target_compile_definitions(tiny-process-library
                    PUBLIC MSYS_PROCESS_USE_SH)
            endif()
        else()
            add_library(tiny-process-library OBJECT
                ${tinyprocesslibrary_SOURCE_DIR}/process.cpp
                ${tinyprocesslibrary_SOURCE_DIR}/process_unix.cpp)
        endif()

        if(MSVC)
            target_compile_definitions(tiny-process-library
                PRIVATE /D_CRT_SECURE_NO_WARNINGS)
        else()
            target_compile_options(tiny-process-library
                PRIVATE -std=c++11 -Wall -Wextra)
        endif()

        find_package(Threads REQUIRED)

        target_link_libraries(tiny-process-library PRIVATE
            ${CMAKE_THREAD_LIBS_INIT})
        target_include_directories(tiny-process-library PUBLIC
            $<BUILD_INTERFACE:${tinyprocesslibrary_SOURCE_DIR}>)

    endif()
endif()

# =====
# CLARA
# =====

add_library(Clara INTERFACE)
target_sources(Clara INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/clara/clara.hpp)
target_include_directories(Clara INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/clara>)
