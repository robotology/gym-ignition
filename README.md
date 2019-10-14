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

<p align="center">
    <a href="https://github.com/robotology/gym-ignition/actions">
    <img src="https://github.com/robotology/gym-ignition/workflows/.github/workflows/docker.yml/badge.svg" alt="CI Docker Image" />
    </a>
    <a href="https://github.com/robotology/gym-ignition/actions">
    <img src="https://github.com/robotology/gym-ignition/workflows/.github/workflows/ci.yml/badge.svg" alt="Continuous Integration" />
    </a>
    <a href="https://github.com/robotology/gym-ignition/actions">
    <img src="https://github.com/robotology/gym-ignition/workflows/.github/workflows/pypi.yml/badge.svg" alt="PyPI Release" />
    </a>
    <a href="https://www.codacy.com/app/diegoferigo/gym-ignition?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=diegoferigo/gym-ignition&amp;utm_campaign=Badge_Grade">
    <img src="https://api.codacy.com/project/badge/Grade/899a7c8304e14ed9b2330eb309cdad15" alt="Codacy Badge" />
    </a>
    <a href="https://github.com/robotology/gym-ignition/blob/master/LICENSE">
    <img src="https://img.shields.io/badge/license-LGPL-19c2d8.svg" alt="License" />
    </a>
</p>

<p align="center">
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
</p>

<p align="center">
    <a href="https://colab.research.google.com/github/robotology/gym-ignition/blob/master/examples/colab/RandomPolicy.ipynb">
    <img src="https://colab.research.google.com/assets/colab-badge.svg" alt="Open In Colab"/>
    </a>
</p>

<p align="center">⚠️ Warning, <a href="https://en.wikipedia.org/wiki/Here_be_dragons">Here be Dragons</a> ⚠️</p>
<p align="center">You just reached a very unstable corner.<br/>Welcome, but mind your step. </p>

## What

Gym-Ignition provides the capability of creating **reproducible robotics environments** for reinforcement learning research.

The project is composed of the following components:

| Component                                     | Description                                                  |
| --------------------------------------------- | ------------------------------------------------------------ |
| [**`ignition`**](ignition/)                   | A set of classes to interface `gympp` with the new Ignition Gazebo simulator, part of the [Ignition Robotics](http://ignitionrobotics.org) suite. |
| [**`plugins`**](plugins/)                     | Ignition Gazebo plugins.                                     |
| [**`gym_ignition`**](gym_ignition/)           | Python package for creating OpenAI Gym environments. Environments can be implemented either in C++ using `gympp` or in Python using the SWIG binded classes of the `ignition` component. |
| [**`gym_ignition_data`**](gym_ignition_data/) | SDF and URDF models and Gazebo worlds.                       |
| [**`gympp`**](gympp/)                         | An _experimental_ C++ port of the OpenAI [Gym interfaces](https://github.com/openai/gym/tree/master/gym), used to create pure C++ environments. |

## Why

Refer to the [Citation](#citation) for the extended rationale behind this project.

`TL;DR`

We designed Gym-Ignition driven by the following reasons:

- Advances in RL research are pushed by the development of more complex environments, and vice versa.
- There's no standard framework in the robotics community for creating simulated robotic environments.
- Environments that can be transferred from simulation to reality with minimal changes do not exist.
- Alternative solutions are not developed by roboticists for roboticist, and therefore they do not use familiar tools.
- Existing robotics environments are typically difficult to adapt to run on different physics engines and different robotic platforms.
- Only few solutions offer realistic rendering capabilities.

## How

This project interfaces with the new generation of the [Gazebo](http://gazebosim.org) simulator, called [Ignition Gazebo](https://ignitionrobotics.org/libs/gazebo). It is part of the new [Ignition Robotics](http://ignitionrobotics.org) suite developed by [Open Robotics](https://www.openrobotics.org/).

Ignition Robotics is currently under heavy development and is not yet stable. Though, it already offers enough functionalities for this project's aims:

- Simulator-as-a-library
- New modular architecture
- C++ utilities developed with a robotic mindset
- New abstractions of physics engines and rendering engines that exploit runtime plugins
- Full support of [DART](https://github.com/dartsim/dart) and coming support of [bullet3](https://github.com/bulletphysics/bullet3)
- Support of distributed simulations
- Well maintained and packaged
- [Ignition Fuel](https://app.ignitionrobotics.org/dashboard) database to download models and worlds

### Features

At the time of writing, Gym-Ignition offers the following features:

- Environments compatible with [OpenAI Gym](https://github.com/openai/gym/)
- Worlds and models are [SDF descriptions](http://sdformat.org)
- Reproducibility guarantees
- Accelerated and multiprocess execution
- Environments are a combination of three elements:
  - **Task**: the logic of the decision-making problem. It defines how to process the given action, how to calculate the reward, how to structure the observation, and how to reset the environment. Task objects are robot-independent and runtime-independent.
  - **Robot**: unified interface to access to robot resources. It is used to gather data and send commands to either simulated or real robot in a seamless way.
  - **Runtime**: implements the actual step of the environment. Simulated runtimes step the simulator, real-time runtimes deal with real-time execution constraints. A Task object can be executed by any runtime without any change.
- Experimental support to create C++ environments

## Demo

We provide two different methods to test Gym-Ignition without the need to install it locally:

1. **Colab notebook**: [![](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/robotology/gym-ignition/blob/master/examples/colab/RandomPolicy.ipynb) and run the example
1. **Docker image**:
   ```sh
   docker pull diegoferigo/gym-ignition:latest
   pip install rocker
   
   # Intel GPU
   rocker --x11 diegoferigo/gym-ignition ./github/examples/python/launch_cartpole.py
   
   # Nvidia GPU
   rocker --x11 --nvidia diegoferigo/gym-ignition ./github/examples/python/launch_cartpole.py
   ```

## Setup

The setup instructions expect a **Ubuntu** distribution with at least **Python 3.6**. Gym-Ignition is compatible also with other distributions (and, also, other OSs) under the assumption that the Ignition Robotics suite can be installed either from repos or source. Though, to keep the instruction simple, we only report the steps for the Ubuntu distro.

The process is different whether you're an _user_ that wants to create environments using Gym-Ignition or you are a _developer_ that wants to edit the Python and C++ code.

### User setup

1. Install the Ignition Robotics suite following the [official documentation](https://ignitionrobotics.org/docs/latest/install)
1. Install Gym-Ignition with `pip install gym-ignition` (preferably in a [virtual environment](https://docs.python.org/3.6/tutorial/venv.html))

After these steps, you should be able to execute the example [`launch_cartpole.py`](examples/python/launch_cartpole.py).

### Developer setup

1. Install [SWIG](https://github.com/swig/swig) with `apt install swig`
1. Install all the Ignition Robotics suite except `ignition-gazebo2` following the [official documentation](https://ignitionrobotics.org/docs/latest/install)
1. Install `ign-gazebo` from our [temporary fork](https://github.com/diegoferigo/ign-gazebo)
1. Clone this repository
1. Build and install the CMake project
   ```sh
   mkdir build
   cd build
   cmake -DCMAKE_INSTALL_PREFIX=<installprefix> ..
   cmake --build .
   cmake --build . --target install
   ```
1. Install the Python package in [editable mode](https://pip.pypa.io/en/stable/reference/pip_install/#editable-installs):
   ```sh
   pip3 install -e .
   ```
1. Export the following environment variable:
   ```sh
   # C++ bindings
   export PYTHONPATH=<installprefix>/lib/python3.6/site-packages
   ```

After these steps, you should be able to execute the example [`launch_cartpole.py`](examples/python/launch_cartpole.py).

### Unstable builds

Gym-Ignition still doesn't have a steady release cycle strategy. This project targets mainly research, and its development is very active. In order to quickly deliver new features, we do our best to have a fast release cycle.

Though, if you find interesting [PRs](https://github.com/robotology/gym-ignition/pulls) that are not yet included in the [most recent release](https://github.com/robotology/gym-ignition/releases), you can get the most recent version as follows:

1. **User installation**: install [`gym-ignition-nightly`](https://pypi.org/project/gym-ignition-nightly/). Be sure that the last release [includes a wheel](https://pypi.org/project/gym-ignition-nightly/#files).
1. **Developer installation**: check-out the `devel` branch after cloning the repository.

## Citation

```
@inproceedings{ferigoGymIgnition2019,
  title = {Gym-Ignition: Reproducible Robotic Simulations for Reinforcement Learning},
  author = {Ferigo, Diego and Traversaro, Silvio and Pucci, Daniele},
  booktitle = {RSS 2019 Workshop on Closing the Reality Gap in Sim2real Transfer for Robotic Manipulation},
  year = {2019},
}
```

[`paper.pdf`](https://sim2real.github.io/assets/papers/ferigo.pdf)

---

**Disclaimer:** Gym-Ignition is an independent project and is not related by any means to OpenAI and Open Robotics.