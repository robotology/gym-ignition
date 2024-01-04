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
    PACKAGE_DEST gz-sim
    TARGETS_ORIG ignition-gazebo3 core
    TARGETS_DEST gz-sim  core
    NAMESPACE_ORIG ignition-gazebo3
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
    PACKAGE_ORIG ignition-sensors3-all
    PACKAGE_DEST gz-sensors-all
    TARGETS_ORIG ignition-sensors3-all
    TARGETS_DEST gz-sensors-all
    NAMESPACE_ORIG ignition-sensors3
    NAMESPACE_DEST gz-sensors
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-rendering3
    PACKAGE_DEST gz-rendering
    TARGETS_ORIG ignition-rendering3
    TARGETS_DEST gz-rendering
    NAMESPACE_ORIG ignition-rendering3
    NAMESPACE_DEST gz-rendering
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-gazebo3-rendering
    PACKAGE_DEST gz-sim-rendering
    TARGETS_ORIG ignition-gazebo3-rendering
    TARGETS_DEST gz-sim-rendering
    NAMESPACE_ORIG ignition-gazebo3
    NAMESPACE_DEST gz-sim
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-physics2
    PACKAGE_DEST gz-physics
    TARGETS_ORIG ignition-physics2
    TARGETS_DEST gz-physics
    NAMESPACE_ORIG ignition-physics2
    NAMESPACE_DEST gz-physics
    REQUIRED TRUE
    )
