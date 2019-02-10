find_package(ignition-gazebo COMPONENTS all REQUIRED)

add_library(gympp-ignition
    include/gympp/gyms/Ignition.h
    src/Ignition.cpp)

target_link_libraries(gympp-ignition
    PUBLIC
    gympp
    PRIVATE
    ignition-gazebo::ignition-gazebo)

target_include_directories(gympp-ignition PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

install(
    TARGETS gympp-ignition
    EXPORT gympp
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gympp)
