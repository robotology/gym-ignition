# ===========
# WORLD FILES
# ===========

install(
    DIRECTORY worlds
    DESTINATION ${CMAKE_INSTALL_DATADIR}/gympp/gazebo)

# ==========
# SDF MODELS
# ==========

install(
    DIRECTORY CartPole
    DESTINATION ${CMAKE_INSTALL_DATADIR}/gympp/gazebo/models
    FILES_MATCHING PATTERN "*.sdf" PATTERN "*.config")

install(
    DIRECTORY iCubGazebo
    DESTINATION ${CMAKE_INSTALL_DATADIR}/gympp/gazebo/models
    FILES_MATCHING PATTERN "*.sdf" PATTERN "*.config")

install(
    DIRECTORY iCubGazeboV2_5
    DESTINATION ${CMAKE_INSTALL_DATADIR}/gympp/gazebo/models
    FILES_MATCHING PATTERN "*.sdf" PATTERN "*.config")
