include(AliasImportedTarget)

# https://ignitionrobotics.org/docs/dome/install#dome-libraries

alias_imported_target(
    PACKAGE sdformat10
    TARGETS_ORIG sdformat10
    TARGETS_DEST sdformat
    NAMESPACE_ORIG sdformat10
    NAMESPACE_DEST sdformat
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE ignition-gazebo4
    TARGETS_ORIG ignition-gazebo4 core
    TARGETS_DEST ignition-gazebo  core
    NAMESPACE_ORIG ignition-gazebo4
    NAMESPACE_DEST ignition-gazebo
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE ignition-common3
    TARGETS_ORIG ignition-common3
    TARGETS_DEST ignition-common
    NAMESPACE_ORIG ignition-common3
    NAMESPACE_DEST ignition-common
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE ignition-sensors4-all
    TARGETS_ORIG ignition-sensors4-all
    TARGETS_DEST ignition-sensors-all
    NAMESPACE_ORIG ignition-sensors4
    NAMESPACE_DEST ignition-sensors
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE ignition-rendering4
    TARGETS_ORIG ignition-rendering4
    TARGETS_DEST ignition-rendering
    NAMESPACE_ORIG ignition-rendering4
    NAMESPACE_DEST ignition-rendering
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE ignition-gazebo4-rendering
    TARGETS_ORIG ignition-gazebo4-rendering
    TARGETS_DEST ignition-gazebo-rendering
    NAMESPACE_ORIG ignition-gazebo4
    NAMESPACE_DEST ignition-gazebo
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE ignition-physics3
    TARGETS_ORIG ignition-physics3
    TARGETS_DEST ignition-physics
    NAMESPACE_ORIG ignition-physics3
    NAMESPACE_DEST ignition-physics
    REQUIRED TRUE
    )
