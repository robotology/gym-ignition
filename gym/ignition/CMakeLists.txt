add_library(gympp-ignition
    include/gympp/gyms/Ignition.h
    src/Ignition.cpp)

target_link_libraries(gympp-ignition
    PUBLIC
    gympp
    PRIVATE
    tiny-process-library
    ignition-gazebo::ignition-gazebo)

set_target_properties(gympp-ignition PROPERTIES
    PUBLIC_HEADER include/gympp/gyms/Ignition.h)

target_include_directories(gympp-ignition PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

install(
    TARGETS gympp-ignition
    EXPORT gympp
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gympp/gyms)
