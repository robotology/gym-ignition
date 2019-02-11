include(FetchContent)

FetchContent_Declare(
  TinyProcessLibrary
  GIT_REPOSITORY https://gitlab.com/eidheim/tiny-process-library.git
)

FetchContent_GetProperties(TinyProcessLibrary)
if(NOT TinyProcessLibrary_POPULATED)
    FetchContent_Populate(TinyProcessLibrary)
    # Build position independent code
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
    add_subdirectory(${tinyprocesslibrary_SOURCE_DIR} ${tinyprocesslibrary_BINARY_DIR})
endif()
