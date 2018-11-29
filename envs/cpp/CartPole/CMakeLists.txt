add_library(CartPole
    CartPole.h
    CartPole.cpp)

target_link_libraries(CartPole
    PUBLIC
    gympp
    gympp-ignition)

target_include_directories(CartPole PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR})
