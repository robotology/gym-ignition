include(AliasImportedTarget)

# https://ignitionrobotics.org/docs/fortress/install#fortress-libraries

alias_imported_target(
    PACKAGE_ORIG sdformat13
    PACKAGE_DEST sdformat
    TARGETS_ORIG sdformat13
    TARGETS_DEST sdformat
    NAMESPACE_ORIG sdformat13
    NAMESPACE_DEST sdformat
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG gz-sim7
    PACKAGE_DEST gz-sim
    TARGETS_ORIG gz-sim7 core
    TARGETS_DEST gz-sim  core
    NAMESPACE_ORIG gz-sim7
    NAMESPACE_DEST gz-sim
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG gz-common5
    PACKAGE_DEST gz-common
    TARGETS_ORIG gz-common5
    TARGETS_DEST gz-common
    NAMESPACE_ORIG gz-common5
    NAMESPACE_DEST gz-common
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG gz-sensors7-all
    PACKAGE_DEST gz-sensors-all
    TARGETS_ORIG gz-sensors7-all
    TARGETS_DEST gz-sensors-all
    NAMESPACE_ORIG gz-sensors7
    NAMESPACE_DEST gz-sensors7
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG gz-rendering7
    PACKAGE_DEST gz-rendering
    TARGETS_ORIG gz-rendering7
    TARGETS_DEST gz-rendering
    NAMESPACE_ORIG gz-rendering7
    NAMESPACE_DEST gz-rendering
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG gz-sim7-rendering
    PACKAGE_DEST gz-sim-rendering
    TARGETS_ORIG gz-sim7-rendering
    TARGETS_DEST gz-sim-rendering
    NAMESPACE_ORIG gz-sim7
    NAMESPACE_DEST gz-sim
    REQUIRED TRUE
    )

alias_imported_target(
    PACKAGE_ORIG gz-physics6
    PACKAGE_DEST gz-physics
    TARGETS_ORIG gz-physics6
    TARGETS_DEST gz-physics
    NAMESPACE_ORIG gz-physics6
    NAMESPACE_DEST gz-physics
    REQUIRED TRUE
    )
