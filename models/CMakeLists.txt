install(
    FILES CartPole/CartPole.sdf
          CartPole/CartPoleWorld.sdf
    DESTINATION ${CMAKE_INSTALL_DATADIR}/ignition/gym-ignition)

install(
    FILES icubIgnition/IcubIgnition.sdf
          icubIgnition/IcubWorld.sdf
    DESTINATION ${CMAKE_INSTALL_DATADIR}/ignition/gym-ignition)
