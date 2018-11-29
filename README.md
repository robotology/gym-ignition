# gym-ignition

`gym-ignition` is an Environment for the OpenAI Gym platform that interfaces with the new Open Robotics set of libraries [Ignition](https://ignitionrobotics.org/home).

After years of Gazebo development, Open Robotics started a big refactoring of its code. Ignition is the result of this work. Its main features are:

- Standalone framework
- Component-based architecture (math, physics, sensors, ...)
- Modern C++ / CMake
- Wraps ODE, Dart, and Bullet physics engines

#### Notes

1. The Environment has been created from scratch following the [official OpenAI Gym guide](https://github.com/openai/gym/tree/master/gym/envs#how-to-create-new-environments-for-gym)
