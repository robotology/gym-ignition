find_package(sdformat8 REQUIRED)
find_package(ignition-gazebo REQUIRED)
find_package(ignition-common3 REQUIRED)

# =================
# DataBridge PLUGIN
# =================

add_library(DataBridge SHARED
    DataBridge.h
    DataBridge.cpp)

target_link_libraries(DataBridge
    PUBLIC
    ignition-gazebo::ignition-gazebo)

target_include_directories(DataBridge PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR})

# ====================
# CartPole ENVIRONMENT
# ====================

add_library(CartPole
    CartPole.h
    CartPole.cpp)

target_link_libraries(CartPole
    PUBLIC
    gympp
    gympp-ignition
    PRIVATE
    sdformat8::sdformat8
    ignition-gazebo::ignition-gazebo
    ignition-common3::ignition-common3)

target_include_directories(CartPole PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR})
