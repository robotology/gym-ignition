# TODO: only required for logging messages
find_package(ignition-common3 REQUIRED)

add_library(gympp
    include/gympp/Gympp.h
    include/gympp/common.h
    include/gympp/Log.h
    include/gympp/Random.h
    include/gympp/spaces/Space.h
    src/Space.cpp
    src/Random.cpp)

target_link_libraries(gympp PUBLIC
    ignition-common3::ignition-common3)

file(GLOB_RECURSE GYMPP_HEADERS_INSTALL
#    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/include
    include/*.h)

set_target_properties(gympp PROPERTIES
    PUBLIC_HEADER "${GYMPP_HEADERS_INSTALL}"
    POSITION_INDEPENDENT_CODE ON)

target_include_directories(gympp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

install(
    TARGETS gympp
    EXPORT gympp
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gympp)
