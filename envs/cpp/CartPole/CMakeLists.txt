find_package(sdformat8 REQUIRED)
find_package(ignition-gazebo REQUIRED)
find_package(ignition-common3 REQUIRED)
find_package(ignition-physics1 REQUIRED)

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

# ===============
# CartPole PLUGIN
# ===============

add_library(CartPolePlugin SHARED
    CartPolePlugin.h
    CartPolePlugin.cpp)

target_link_libraries(CartPolePlugin
    PUBLIC
    gympp-ignition
    ignition-gazebo::ignition-gazebo)

target_include_directories(CartPolePlugin PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR})

if(MSVC)
    # Import math symbols from standard cmath
    target_compile_definitions(CartPolePlugin PRIVATE "_USE_MATH_DEFINES")
endif()

# ====================
# CartPole ENVIRONMENT
# ====================

#add_library(CartPole
#    CartPole.h
#    CartPole.cpp)

#target_link_libraries(CartPole
#    PUBLIC
#    gympp
#    gympp-ignition
#    PRIVATE
#    ignition-physics1::ignition-physics1
#    sdformat8::sdformat8
#    ignition-gazebo::ignition-gazebo
#    ignition-common3::ignition-common3)

#target_include_directories(CartPole PUBLIC
#    ${CMAKE_CURRENT_SOURCE_DIR})
