# ===============
# CartPole PLUGIN
# ===============

add_library(CartPolePlugin SHARED
    CartPolePlugin.h
    CartPolePlugin.cpp)

target_link_libraries(CartPolePlugin
    PUBLIC
    IgnitionEnvironment
    RobotSingleton
    ignition-gazebo1::core)

target_include_directories(CartPolePlugin PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR})

if(MSVC)
    # Import math symbols from standard cmath
    target_compile_definitions(CartPolePlugin PRIVATE "_USE_MATH_DEFINES")
endif()

install(
    TARGETS CartPolePlugin
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/gympp
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/gympp
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/gympp)
