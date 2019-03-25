find_package(ignition-gazebo COMPONENTS all REQUIRED)
find_package(ignition-gui1 REQUIRED)
find_package(ignition-common3 REQUIRED)

add_executable(LaunchIgnGazebo launchIgnGazebo.cpp)

target_link_libraries(LaunchIgnGazebo PRIVATE
    ignition-gazebo::ignition-gazebo
    ignition-gui1::ignition-gui1
    ignition-common3::ignition-common3)
