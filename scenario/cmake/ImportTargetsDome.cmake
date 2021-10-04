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
    PACKAGE_DEST ignition-gazebo
    TARGETS_ORIG ignition-gazebo4 core
    TARGETS_DEST ignition-gazebo  core
    NAMESPACE_ORIG ignition-gazebo4
    NAMESPACE_DEST ignition-gazebo
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-common3
    PACKAGE_DEST ignition-common
    TARGETS_ORIG ignition-common3
    TARGETS_DEST ignition-common
    NAMESPACE_ORIG ignition-common3
    NAMESPACE_DEST ignition-common
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-sensors4-all
    PACKAGE_DEST ignition-sensors-all
    TARGETS_ORIG ignition-sensors4-all
    TARGETS_DEST ignition-sensors-all
    NAMESPACE_ORIG ignition-sensors4
    NAMESPACE_DEST ignition-sensors
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-rendering4
    PACKAGE_DEST ignition-rendering
    TARGETS_ORIG ignition-rendering4
    TARGETS_DEST ignition-rendering
    NAMESPACE_ORIG ignition-rendering4
    NAMESPACE_DEST ignition-rendering
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-gazebo4-rendering
    PACKAGE_DEST ignition-gazebo-rendering
    TARGETS_ORIG ignition-gazebo4-rendering
    TARGETS_DEST ignition-gazebo-rendering
    NAMESPACE_ORIG ignition-gazebo4
    NAMESPACE_DEST ignition-gazebo
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-physics3
    PACKAGE_DEST ignition-physics
    TARGETS_ORIG ignition-physics3
    TARGETS_DEST ignition-physics
    NAMESPACE_ORIG ignition-physics3
    NAMESPACE_DEST ignition-physics
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
