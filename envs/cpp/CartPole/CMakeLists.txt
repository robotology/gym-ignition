find_package(ignition-gazebo REQUIRED)

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
