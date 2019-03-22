include(FetchContent)

find_package(tiny-process-library QUIET)

if(NOT tiny-process-library_FOUND)
    message(STATUS "Bootstrapping Tiny Process Library")

    FetchContent_Declare(
        TinyProcessLibrary
        GIT_REPOSITORY https://gitlab.com/eidheim/tiny-process-library.git
        )

    FetchContent_GetProperties(TinyProcessLibrary)
    if(NOT TinyProcessLibrary_POPULATED)
        FetchContent_Populate(TinyProcessLibrary)
        add_subdirectory(${tinyprocesslibrary_SOURCE_DIR} ${tinyprocesslibrary_BINARY_DIR})

        set_target_properties(tiny-process-library PROPERTIES
            PUBLIC_HEADER ${tinyprocesslibrary_SOURCE_DIR}/process.hpp
            POSITION_INDEPENDENT_CODE ON)

        # Install the target
        install(
            TARGETS tiny-process-library
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
            PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
    endif()
endif()
