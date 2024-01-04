include(AliasImportedTarget)

# https://ignitionrobotics.org/docs/fortress/install#fortress-libraries

alias_imported_target(
    PACKAGE_ORIG sdformat12
    PACKAGE_DEST sdformat
    TARGETS_ORIG sdformat12
    TARGETS_DEST sdformat
    NAMESPACE_ORIG sdformat12
    NAMESPACE_DEST sdformat
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-gazebo6
    PACKAGE_DEST gz-sim
    TARGETS_ORIG ignition-gazebo6 core
    TARGETS_DEST gz-sim  core
    NAMESPACE_ORIG ignition-gazebo6
    NAMESPACE_DEST gz-sim
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-common4
    PACKAGE_DEST gz-common
    TARGETS_ORIG ignition-common4
    TARGETS_DEST gz-common
    NAMESPACE_ORIG ignition-common4
    NAMESPACE_DEST gz-common
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-sensors6-all
    PACKAGE_DEST gz-sensors-all
    TARGETS_ORIG ignition-sensors6-all
    TARGETS_DEST gz-sensors-all
    NAMESPACE_ORIG ignition-sensors6
    NAMESPACE_DEST gz-sensors
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-rendering6
    PACKAGE_DEST gz-rendering
    TARGETS_ORIG ignition-rendering6
    TARGETS_DEST gz-rendering
    NAMESPACE_ORIG ignition-rendering6
    NAMESPACE_DEST gz-rendering
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-gazebo6-rendering
    PACKAGE_DEST gz-sim-rendering
    TARGETS_ORIG ignition-gazebo6-rendering
    TARGETS_DEST gz-sim-rendering
    NAMESPACE_ORIG ignition-gazebo6
    NAMESPACE_DEST gz-sim
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-physics5
    PACKAGE_DEST gz-physics
    TARGETS_ORIG ignition-physics5
    TARGETS_DEST gz-physics
    NAMESPACE_ORIG ignition-physics5
    NAMESPACE_DEST gz-physics
    REQUIRED TRUE
    )
