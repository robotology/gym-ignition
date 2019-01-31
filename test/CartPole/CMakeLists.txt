#add_executable(CartPoleTest CartPoleTest.cpp)
#target_link_libraries(CartPoleTest PRIVATE
#    CartPole)

add_executable(CartPolePluginTest CartPolePluginTest.cpp)
target_link_libraries(CartPolePluginTest PRIVATE
    gympp-ignition)
