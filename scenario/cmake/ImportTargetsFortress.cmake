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
    PACKAGE_DEST ignition-gazebo
    TARGETS_ORIG ignition-gazebo6 core
    TARGETS_DEST ignition-gazebo  core
    NAMESPACE_ORIG ignition-gazebo6
    NAMESPACE_DEST ignition-gazebo
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-common4
    PACKAGE_DEST ignition-common
    TARGETS_ORIG ignition-common4
    TARGETS_DEST ignition-common
    NAMESPACE_ORIG ignition-common4
    NAMESPACE_DEST ignition-common
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-sensors6-all
    PACKAGE_DEST ignition-sensors-all
    TARGETS_ORIG ignition-sensors6-all
    TARGETS_DEST ignition-sensors-all
    NAMESPACE_ORIG ignition-sensors6
    NAMESPACE_DEST ignition-sensors
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-rendering6
    PACKAGE_DEST ignition-rendering
    TARGETS_ORIG ignition-rendering6
    TARGETS_DEST ignition-rendering
    NAMESPACE_ORIG ignition-rendering6
    NAMESPACE_DEST ignition-rendering
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-gazebo6-rendering
    PACKAGE_DEST ignition-gazebo-rendering
    TARGETS_ORIG ignition-gazebo6-rendering
    TARGETS_DEST ignition-gazebo-rendering
    NAMESPACE_ORIG ignition-gazebo6
    NAMESPACE_DEST ignition-gazebo
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-physics5
    PACKAGE_DEST ignition-physics
    TARGETS_ORIG ignition-physics5
    TARGETS_DEST ignition-physics
    NAMESPACE_ORIG ignition-physics5
    NAMESPACE_DEST ignition-physics
    REQUIRED TRUE
    )
