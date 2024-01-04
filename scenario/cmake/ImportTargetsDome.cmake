include(AliasImportedTarget)

# https://ignitionrobotics.org/docs/dome/install#dome-libraries

alias_imported_target(
    PACKAGE_ORIG sdformat10
    PACKAGE_DEST sdformat
    TARGETS_ORIG sdformat10
    TARGETS_DEST sdformat
    NAMESPACE_ORIG sdformat10
    NAMESPACE_DEST sdformat
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-gazebo4
    PACKAGE_DEST gz-sim
    TARGETS_ORIG ignition-gazebo4 core
    TARGETS_DEST gz-sim  core
    NAMESPACE_ORIG ignition-gazebo4
    NAMESPACE_DEST gz-sim
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-common3
    PACKAGE_DEST gz-common
    TARGETS_ORIG ignition-common3
    TARGETS_DEST gz-common
    NAMESPACE_ORIG ignition-common3
    NAMESPACE_DEST gz-common
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-sensors4-all
    PACKAGE_DEST gz-sensors-all
    TARGETS_ORIG ignition-sensors4-all
    TARGETS_DEST gz-sensors-all
    NAMESPACE_ORIG ignition-sensors4
    NAMESPACE_DEST gz-sensors
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-rendering4
    PACKAGE_DEST gz-rendering
    TARGETS_ORIG ignition-rendering4
    TARGETS_DEST gz-rendering
    NAMESPACE_ORIG ignition-rendering4
    NAMESPACE_DEST gz-rendering
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-gazebo4-rendering
    PACKAGE_DEST gz-sim-rendering
    TARGETS_ORIG ignition-gazebo4-rendering
    TARGETS_DEST gz-sim-rendering
    NAMESPACE_ORIG ignition-gazebo4
    NAMESPACE_DEST gz-sim
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-physics3
    PACKAGE_DEST gz-physics
    TARGETS_ORIG ignition-physics3
    TARGETS_DEST gz-physics
    NAMESPACE_ORIG ignition-physics3
    NAMESPACE_DEST gz-physics
    REQUIRED TRUE
    )

    alias_imported_target(
    PACKAGE_ORIG ignition-fuel_tools5
    PACKAGE_DEST ignition-fuel_tools
    TARGETS_ORIG ignition-fuel_tools5
    TARGETS_DEST ignition-fuel_tools
    NAMESPACE_ORIG ignition-fuel_tools5
    NAMESPACE_DEST ignition-fuel_tools
    REQUIRED TRUE
    )
