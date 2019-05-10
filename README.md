<p align="center">
<h1 align="center">gym-ignition</h1>
</p>

<p align="center">
<b><a href="https://github.com/robotology/gym-ignition#what">What</a></b>
•
<b><a href="https://github.com/robotology/gym-ignition#why">Why</a></b>
•
<b><a href="https://github.com/robotology/gym-ignition#how">How</a></b>
•
<b><a href="https://github.com/robotology/gym-ignition#setup">Setup</a></b>
</p>

<p align="center">
    <a href="https://travis-ci.com/robotology/gym-ignition">
    <img src="https://img.shields.io/travis/com/robotology/gym-ignition/master.svg?logo=travis&label=master" alt="Build Status (Linux/macOS) (master)" />
    </a>
    <a href="https://www.codacy.com/app/diegoferigo/gym-ignition?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=diegoferigo/gym-ignition&amp;utm_campaign=Badge_Grade">
    <img src="https://api.codacy.com/project/badge/Grade/899a7c8304e14ed9b2330eb309cdad15" alt="Codacy Badge" />
    </a>
    <a href="https://github.com/robotology/gym-ignition/blob/master/LICENSE">
    <img src="https://img.shields.io/badge/license-LGPL-19c2d8.svg" alt="License" />
    </a>
</p>

<p align="center">⚠️ Warning, <a href="https://en.wikipedia.org/wiki/Here_be_dragons">Here be Dragons</a> ⚠️</p>
<p align="center">You just reached a very unstable corner. The status of this project is pre-alpha, and is based on other projects in alpha status.<br/>Welcome, but mind the step. </p>

## What

`gym-ignition` provides the capability of creating **reproducible robotics environments** for reinforcement learning research. The project is composed of the following components:

| Component                                         | Description                                                  |
| ------------------------------------------------- | ------------------------------------------------------------ |
| [**`gympp`**](gympp/)                             | A C++ port of the OpenAI [Gym interfaces](https://github.com/openai/gym/tree/master/gym) |
| [**`ignition`**](ignition/)                       | A set of classes to interface `gympp` with the libraries that form new generation of Gazebo: [Ignition Robotics](http://ignitionrobotics.org) |
| [**`gym_ignition`**](gym_ignition/)               | A python package that wraps the environments implemented with `gympp` with the python OpenAI Gym interfaces |
| [**`plugins`**](plugins/) [**`models`**](models/) | Demo environments                                            |

## Why

Simulation has always been a key element for every robotic project, and this is confirmed by the impressive amount of available open-source and commercial simulators.
In the past few years, the interest of the robotics community in the most recent advances of reinforcement learning research has steadily grown.
However, the ecosystem of RL applied to robotics does not take yet the advantage of the tools with which roboticists are typically familiar.
Furthermore, most of them lack fundamental features required to effectively perform research on real robots since they focus mainly on the learning algorithm, not its actual usage on the target device.

The applicability of theoretical results on real devices (aka sim-to-real) is still an uncharted land, probably due to the numerous pitfalls that can be encountered when reinforcement learning is applied to real-world robots.
The aim of `gym-ignition` is to ease the roboticists life when dealing with the burden of prototyping, testing, and deploying such applications.

Particularly, the features we envision to achieve are the following:

- Create environments and robot models using [SDF descriptions](http://sdformat.org)
- Allow interfacing on a C++ level with simulated and real robots
- Expose a standard OpenAI Gym interface to python in order to exploit existing libraries that provide RL algorithms
- Guarantee a fully reproducible behavior

## How

Among the many, [Gazebo](http://gazebosim.org) became one of the most successful simulators, backed by its bond with the ROS middleware, and it is used by countless companies and research institutes.
Open Robotics is currently refactoring the entire code base of Gazebo which will be released in the version 11 of the simulator.
We decided to exploit the new [Ignition Robotics](http://ignitionrobotics.org) libraries, which provide the following features:

- Support of many physic engines out-of-the-box
- Possibility to integrate third-party physic engines
- Easy system-integration thanks to the possibility to use gazebo as a library
- Modular architecture
- Support of distributed simulations
- C++ utilities developed with a robotic mindset
- Well maintained and packaged

The environments created for Ignition Gazebo are wrapped in a C++ interface inspired by OpenAI Gym.
We also provide python bindings to map this C++ interface to the OpenAI Gym interface.
In this way, we can reuse all the amazing frameworks that the reinforcement learning and machine learning communities developed over these years while providing to the researchers of these topics a familiar interface.
Furthermore, by implementing the interfaces of `gympp` to communicate with a real robot (where typically C++ APIs are used), the learning python code does not have to change between simulation and real-time usage (still in `[WIP]`).

## Setup

Considering the alpha status of Ignition Robotics, there is no simple way yet to provide stable setup instructions.
We are working upon their latest development branches, and in some cases we have own forks to implement specific features.
If you are interested in collaborating, please reach out!

### Build and Install

After you have installed all the required ignition libraries, this project can be compiled and installed executing the following commands:

```sh
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=<installprefix> ..
cmake --build .
cmake --build . --target install
```

### Configure the environment

Ignition Robotics needs to find in the system four types of files. Depending on your install prefix, export the following environment varables:

| Environment Variable            | Value                                           | Description                                |
| ------------------------------- | ----------------------------------------------- | ------------------------------------------ |
| `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` | `<installprefix>/lib/gympp/plugins`             | Folder containing plugins                  |
| `IGN_GAZEBO_RESOURCE_PATH`      | `<installprefix>/share/gympp/gazebo/worlds`     | Folder containing `.world` files           |
| `SDF_PATH`                      | `<installprefix>/share/gympp/gazebo/models`     | Folders containing `sdf` models            |
| `IGN_FILE_PATH`                 | Location of mesh files - model dependent        | Folders searched to resolve `file://` URIs |

Furthermore, in order to use the python bindings you should export `PYTHONPATH=<installprefix>/lib/gympp/bindings`.

---

**Disclaimer:** `gym-ignition` is an independent project and is not related by any means to OpenAI and Open Robotics
