include(AliasImportedTarget)

# https://ignitionrobotics.org/docs/citadel/install#citadel-libraries

alias_imported_target(
    PACKAGE_ORIG sdformat9
    PACKAGE_DEST sdformat
    TARGETS_ORIG sdformat9
    TARGETS_DEST sdformat
    NAMESPACE_ORIG sdformat9
    NAMESPACE_DEST sdformat
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-gazebo3
    PACKAGE_DEST ignition-gazebo
    TARGETS_ORIG ignition-gazebo3 core
    TARGETS_DEST ignition-gazebo  core
    NAMESPACE_ORIG ignition-gazebo3
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
    PACKAGE_ORIG ignition-sensors3-all
    PACKAGE_DEST ignition-sensors-all
    TARGETS_ORIG ignition-sensors3-all
    TARGETS_DEST ignition-sensors-all
    NAMESPACE_ORIG ignition-sensors3
    NAMESPACE_DEST ignition-sensors
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-rendering3
    PACKAGE_DEST ignition-rendering
    TARGETS_ORIG ignition-rendering3
    TARGETS_DEST ignition-rendering
    NAMESPACE_ORIG ignition-rendering3
    NAMESPACE_DEST ignition-rendering
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-gazebo3-rendering
    PACKAGE_DEST ignition-gazebo-rendering
    TARGETS_ORIG ignition-gazebo3-rendering
    TARGETS_DEST ignition-gazebo-rendering
    NAMESPACE_ORIG ignition-gazebo3
    NAMESPACE_DEST ignition-gazebo
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-physics2
    PACKAGE_DEST ignition-physics
    TARGETS_ORIG ignition-physics2
    TARGETS_DEST ignition-physics
    NAMESPACE_ORIG ignition-physics2
    NAMESPACE_DEST ignition-physics
    REQUIRED TRUE
    )
