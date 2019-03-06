add_library(GymFactory
    include/gympp/GymFactory.h
    src/GymFactory.cpp)

target_link_libraries(GymFactory
    PUBLIC gympp
    PRIVATE gympp-ignition ignition-gazebo::ignition-gazebo)

target_include_directories(GymFactory PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_compile_definitions(GymFactory PRIVATE
    CARTPOLE_PLUGIN_PATH="$<TARGET_FILE_DIR:CartPolePlugin>")
