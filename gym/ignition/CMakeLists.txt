find_package(ignition-gazebo COMPONENTS all REQUIRED)
find_package(ignition-gui1 REQUIRED)

add_library(gympp-ignition
    include/gympp/gyms/Ignition.h
    src/Ignition.cpp)

target_link_libraries(gympp-ignition
    PUBLIC
    gympp
    ignition-gazebo::ignition-gazebo # TODO: private
    PRIVATE
    #ignition-gazebo::ign-gazebo-gui
    ignition-gui1::ignition-gui1)

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
