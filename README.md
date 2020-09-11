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
<b><a href="https://github.com/robotology/gym-ignition#demo">Demo</a></b>
•
<b><a href="https://github.com/robotology/gym-ignition#setup">Setup</a></b>
•
<b><a href="https://github.com/robotology/gym-ignition#Citation">Citation</a></b>
</p>

<div align="center">
<p><br/></p>
<table>
    <tbody>
         <tr>
            <td align="left">General</td>
            <td align="center">
                <a href="https://isocpp.org">
                <img src="https://img.shields.io/badge/standard-C++17-blue.svg?style=flat&logo=c%2B%2B" alt="C++ Standard" />
                </a>
                <a href="https://github.com/robotology/gym-ignition">
                <img src="https://img.shields.io/github/languages/code-size/robotology/gym-ignition.svg" alt="Size" />
                </a>
                <a href="https://github.com/robotology/gym-ignition/blob/master/LICENSE">
                <img src="https://img.shields.io/badge/license-LGPL-19c2d8.svg" alt="Size" />
                </a>
            </td>
        </tr> 
         <tr>
            <td align="left">CI/CD</td>
            <td align="center">
                <a href="https://github.com/robotology/gym-ignition/actions">
                <img src="https://github.com/robotology/gym-ignition/workflows/Docker%20Images/badge.svg" alt="Docker Images" />
                </a>
                <a href="https://github.com/robotology/gym-ignition/actions">
                <img src="https://github.com/robotology/gym-ignition/workflows/CI/badge.svg" alt="CI" />
                </a>
                <a href="https://github.com/robotology/gym-ignition/actions">
                <img src="https://github.com/robotology/gym-ignition/workflows/PyPI%20CD/badge.svg" alt="PyPI CD" />
                </a>
                <a href="https://www.codacy.com/app/diegoferigo/gym-ignition?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=diegoferigo/gym-ignition&amp;utm_campaign=Badge_Grade">
                <img src="https://api.codacy.com/project/badge/Grade/899a7c8304e14ed9b2330eb309cdad15" alt="Codacy Badge" />
                </a>
            </td>
        </tr>   
        <tr>
            <td align="left"><code>gym-ignition</code></td>
            <td align="center">
                <a href="https://pypi.org/project/gym-ignition/">
                <img src="https://img.shields.io/pypi/v/gym-ignition.svg" />
                </a>
                <a href="https://pypi.org/project/gym-ignition/">
                <img src="https://img.shields.io/pypi/pyversions/gym-ignition.svg" />
                </a>
                <a href="https://pypi.org/project/gym-ignition/">
                <img src="https://img.shields.io/pypi/status/gym-ignition.svg" />
                </a>
                <a href="https://pypi.org/project/gym-ignition/">
                <img src="https://img.shields.io/pypi/format/gym-ignition.svg" />
                </a>
                <a href="https://pypi.org/project/gym-ignition/">
                <img src="https://img.shields.io/pypi/l/gym-ignition.svg" />
                </a>
            </td>
        </tr>
    </tbody>
</table>
<p><br/></p>
</div>

## What

Gym-Ignition is a framework to create **reproducible robotics environments** for reinforcement learning research.

The project is composed of the following components:

| Component                   | Description                                                  |
| --------------------------- | ------------------------------------------------------------ |
| `ScenarI/O`                 | *Scene Interfaces for Robot Input / Output* is a set of C++ interfaces to abstract the interfacing with simulated and real robots. |
| `Gazebo ScenarI/O`          | Ignition Gazebo bindings that expose the ScenarI/O interfaces. |
| `gym-ignition`              | Python package that provides the tooling to create OpenAI Gym environments for robot learning. |
| `gym-ignition-environments` | Demo environments created with `gym-ignition` and [gym-ignition-models](https://github.com/dic-iit/gym-ignition-models). |
| `Gympp`                     | An _experimental_ C++ port of the OpenAI [Gym interfaces](https://github.com/openai/gym/tree/master/gym), used to create pure C++ environments. |

*Disclaimer: we do not provide support for experimental components.*

## Why

Refer to the [Citation](#citation) for the extended rationale behind this project.

`TL;DR`

We designed Gym-Ignition driven by the following reasons:

- Advances in RL research are pushed by the development of more complex environments, and vice versa.
- There's no standard framework in the robotics community for creating simulated robotic environments.
- Environments that can be transferred from simulation to reality with minimal changes are yet to be seen.
- Alternative solutions are not developed by roboticists for roboticist, and therefore they do not use familiar tools.
- Existing robotics environments are typically difficult to adapt to run on different physics engines and different robots.
- Only few solutions offer realistic rendering capabilities.

## How

This project interfaces with the new generation of the [Gazebo](http://gazebosim.org) simulator, called [Ignition Gazebo](https://ignitionrobotics.org/libs/gazebo).
It is part of the new [Ignition Robotics](http://ignitionrobotics.org) suite developed by [Open Robotics](https://www.openrobotics.org/).

Ignition Robotics is currently under heavy development.
Though, it already offers enough functionalities that fit this project's aims:

- Simulator-as-a-library
- New modular architecture
- C++ utilities developed with a robotic mindset
- New abstractions of physics engines and rendering engines that exploit runtime plugins
- Full support of [DART](https://github.com/dartsim/dart) and coming support of [bullet3](https://github.com/bulletphysics/bullet3)
- Support of distributed simulations
- Well maintained and packaged
- [Ignition Fuel](https://app.ignitionrobotics.org/dashboard) database to download models and worlds

Our **Gazebo ScenarI/O** component provides Python bindings of the simulator that are comparable 
to other popular solutions like [pybullet](https://github.com/bulletphysics/bullet3) and [mujoco-py](https://github.com/openai/mujoco-py).

### Features

At the time of writing, Gym-Ignition offers the following features:

- Environments compatible with [OpenAI Gym](https://github.com/openai/gym/)
- Worlds and models are [SDF descriptions](http://sdformat.org)
- Reproducibility guarantees
- Accelerated and multiprocess execution
- Environments are a combination of three elements:
  - **Task**: the logic of the decision-making problem. It defines how to process the given action, how to calculate the reward, how to structure the observation, and how to reset the environment. Task objects are robot-independent and runtime-independent.
  - **World**: unified interface to access world resources.
  - **Runtime**: implements the actual step of the environment. Simulated runtimes step the simulator, real-time runtimes deal with real-time execution constraints. A Task object can be executed by any runtime without any change.
- Experimental support to create C++ environments

## Demo

We provide a docker image that shows few rollouts executed by a random policy on a cartpole model:

```sh
docker pull diegoferigo/gym-ignition:latest
pip3 install rocker

# Intel GPU
rocker --devices /dev/dri --x11 diegoferigo/gym-ignition /github/examples/python/launch_cartpole.py

# Nvidia GPU
rocker --x11 --nvidia diegoferigo/gym-ignition /github/examples/python/launch_cartpole.py
```

## Setup

Visit the [Installation](https://robotology.github.io/gym-ignition/devel/installation/intro.html) section of our website for updated installation instructions.
Make sure to select the correct branch since the installation could differ depending on the branch you're interested.

## Citation

```
@INPROCEEDINGS{ferigo2020gymignition,
    title={Gym-Ignition: Reproducible Robotic Simulations for Reinforcement Learning},
    author={D. {Ferigo} and S. {Traversaro} and G. {Metta} and D. {Pucci}},
    booktitle={2020 IEEE/SICE International Symposium on System Integration (SII)},
    year={2020},
    pages={885-890},
    doi={10.1109/SII46433.2020.9025951}
} 
```

---

**Disclaimer:** Gym-Ignition is an independent project and is not related by any means to OpenAI and Open Robotics.