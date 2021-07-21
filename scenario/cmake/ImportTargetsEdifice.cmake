include(AliasImportedTarget)

# https://ignitionrobotics.org/docs/edifice/install#edifice-libraries

alias_imported_target(
    PACKAGE_ORIG sdformat11
    PACKAGE_DEST sdformat
    TARGETS_ORIG sdformat11
    TARGETS_DEST sdformat
    NAMESPACE_ORIG sdformat11
    NAMESPACE_DEST sdformat
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-gazebo5
    PACKAGE_DEST ignition-gazebo
    TARGETS_ORIG ignition-gazebo5 core
    TARGETS_DEST ignition-gazebo  core
    NAMESPACE_ORIG ignition-gazebo5
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
    PACKAGE_ORIG ignition-sensors5-all
    PACKAGE_DEST ignition-sensors-all
    TARGETS_ORIG ignition-sensors5-all
    TARGETS_DEST ignition-sensors-all
    NAMESPACE_ORIG ignition-sensors5
    NAMESPACE_DEST ignition-sensors
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-rendering5
    PACKAGE_DEST ignition-rendering
    TARGETS_ORIG ignition-rendering5
    TARGETS_DEST ignition-rendering
    NAMESPACE_ORIG ignition-rendering5
    NAMESPACE_DEST ignition-rendering
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-gazebo5-rendering
    PACKAGE_DEST ignition-gazebo-rendering
    TARGETS_ORIG ignition-gazebo5-rendering
    TARGETS_DEST ignition-gazebo-rendering
    NAMESPACE_ORIG ignition-gazebo5
    NAMESPACE_DEST ignition-gazebo
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG ignition-physics4
    PACKAGE_DEST ignition-physics
    TARGETS_ORIG ignition-physics4
    TARGETS_DEST ignition-physics
    NAMESPACE_ORIG ignition-physics4
    NAMESPACE_DEST ignition-physics
    REQUIRED TRUE
    )
