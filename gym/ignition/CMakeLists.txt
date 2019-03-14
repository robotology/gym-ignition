add_library(gympp-ignition
    include/gympp/gyms/Ignition.h
    src/Ignition.cpp)

target_link_libraries(gympp-ignition
    PUBLIC
    gympp
    PRIVATE
    tiny-process-library
    ignition-gazebo1::core)

set_target_properties(gympp-ignition PROPERTIES
    PUBLIC_HEADER include/gympp/gyms/Ignition.h)

target_include_directories(gympp-ignition PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# ========================
# IGNITION ROBOT SINGLETON
# ========================

# This class must be shared, otherwise plugins do not share static objects like singletons
# with the memory of the program that loaded them

add_library(RobotSingleton SHARED
    include/gympp/robot/RobotSingleton.h
    src/RobotSingleton.cpp)

target_link_libraries(RobotSingleton
    PUBLIC
    gympp
    ignition-gazebo1::core)

set_target_properties(RobotSingleton PROPERTIES
    PUBLIC_HEADER include/gympp/robot/RobotSingleton.h)

target_include_directories(RobotSingleton PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

install(
    TARGETS gympp-ignition RobotSingleton
    EXPORT gympp
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gympp/gyms)

# ====================
# IgnitionRobot PLUGIN
# ====================

add_library(IgnitionRobot SHARED
    include/gympp/robot/IgnitionRobot.h
    src/IgnitionRobot.cpp)

target_link_libraries(IgnitionRobot
    PUBLIC
    gympp
    gympp-ignition
    RobotSingleton
    ignition-gazebo1::core)

target_include_directories(IgnitionRobot PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

install(
    TARGETS IgnitionRobot
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/gympp
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/gympp
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/gympp)

