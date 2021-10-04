# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

include(FetchContent)

# ===
# YCM
# ===

find_package(YCM QUIET)

option(SCENARIO_USE_SYSTEM_YCM
    "Use system-installed YCM, rather than a private copy"
    ${YCM_FOUND})

if(SCENARIO_USE_SYSTEM_YCM AND NOT ${YCM_FOUND})
    message(FATAL_ERROR "Failed to find system YCM")
endif()

if(NOT ${SCENARIO_USE_SYSTEM_YCM})

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

option(SCENARIO_USE_SYSTEM_TPL
    "Use system-installed tiny-process-library, rather than a private copy"
    ${tiny-process-library_FOUND})

if(SCENARIO_USE_SYSTEM_TPL AND NOT ${tiny-process-library_FOUND})
    message(FATAL_ERROR "Failed to find system tiny-process-library")
endif()

if(NOT ${SCENARIO_USE_SYSTEM_TPL})

    FetchContent_Declare(
        tinyprocesslibrary
        GIT_REPOSITORY https://gitlab.com/eidheim/tiny-process-library.git)

    FetchContent_GetProperties(tinyprocesslibrary)

    if(NOT tinyprocesslibrary_POPULATED)
        FetchContent_Populate(tinyprocesslibrary)

        # We don't want to install this library in the system, we instead
        # compile it as an OBJECT library and embed in either the shared or
        # static libraries that need it.

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
        endif()

        find_package(Threads REQUIRED)

        target_link_libraries(tiny-process-library PRIVATE
            ${CMAKE_THREAD_LIBS_INIT})

        target_include_directories(tiny-process-library PUBLIC
            $<BUILD_INTERFACE:${tinyprocesslibrary_SOURCE_DIR}>)

        add_library(tiny-process-library::tiny-process-library ALIAS tiny-process-library)

    endif()
endif()

# =====
# CLARA
# =====

add_library(Clara INTERFACE)
target_sources(Clara INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/clara/clara.hpp)
target_include_directories(Clara INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/clara>)
