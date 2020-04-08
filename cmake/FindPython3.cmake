# Taken from: https://github.com/Kitware/CMake/blob/v3.17.0/Modules/FindPython3.cmake

set (_PYTHON_PREFIX Python3)

set (_Python3_REQUIRED_VERSION_MAJOR 3)

include (${CMAKE_CURRENT_LIST_DIR}/FindPython/Support.cmake)

if (COMMAND __Python3_add_library)
  macro (Python3_add_library)
    __Python3_add_library (Python3 ${ARGV})
  endmacro()
endif()

unset (_PYTHON_PREFIX)
