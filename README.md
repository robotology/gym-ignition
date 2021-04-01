<p align="center">
<h1 align="center">gym-ignition</h1>
</p>

<p align="center">
<b><a href="https://github.com/robotology/gym-ignition#description">Description</a></b>
•
<b><a href="https://github.com/robotology/gym-ignition#setup">Setup</a></b>
•
<b><a href="https://github.com/robotology/gym-ignition#citation">Citation</a></b>
•
<b><a href="https://robotology.github.io/gym-ignition/master/index.html">Website</a></b>
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
                <img src="https://github.com/robotology/gym-ignition/workflows/CI/CD/badge.svg" alt="CICD" />
                </a>
                <a href="https://github.com/robotology/gym-ignition/actions">
                <img src="https://github.com/robotology/gym-ignition/workflows/Docker%20Images/badge.svg" alt="Docker Images" />
                </a>
                <a href="https://www.codacy.com/gh/robotology/gym-ignition/dashboard?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=robotology/gym-ignition&amp;utm_campaign=Badge_Grade">
                <img src="https://api.codacy.com/project/badge/Grade/5536b05f8be94483b64ee883e7170a39" alt="Codacy Badge" />
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

## Description

**gym-ignition** is a framework to create **reproducible robotics environments** for reinforcement learning research.

The project consists of the following components:

- [**`ScenarI/O`**](cpp/scenario/core): 
  *Scene Interfaces for Robot Input / Output* is a C++ abstraction layer to interact with simulated and real robots.
- [**`Gazebo ScenarI/O`**](cpp/scenario/gazebo): 
  Implementation of the ScenarI/O interfaces to interact with the [Ignition Gazebo](https://ignitionrobotics.org) simulator. 
  We provide Python bindings with functionalities comparable to popular alternatives like 
  [pybullet](https://github.com/bulletphysics/bullet3) and [mujoco-py](https://github.com/openai/mujoco-py).
- [**`gym_ignition`**](python/gym_ignition): 
  A Python package with the tooling to create OpenAI Gym environments for robot learning. 
  It provides abstractions like `Task` and `Runtime` to help developing environments that can be executed transparently 
  on all the ScenarI/O implementations (different simulators, real robots, ...).
  The package also contains resources for inverse kinematics and multi-body dynamics supporting floating-based robots
  based on the [iDynTree](https://github.com/robotology/idyntree) library.
- [**`gym_ignition_environments`**](python/gym_ignition_environments):
  Demo environments created with `gym_ignition` and [`gym-ignition-models`](https://github.com/dic-iit/gym-ignition-models) 
  that show the recommended structure.
  
This project provides the complete implementation of ScenarI/O for the Ignition Gazebo simulator.
We expose all the physics engines supported by Ignition Gazebo.
Currently, the default and only physics engine is [DART](https://github.com/dartsim/dart).

We are currently working on backends based on robotic middleware to transparently execute the environments developed 
with `gym_ignition` on real robots.

If you're interested to know the reasons why we started developing gym-ignition and why we selected Ignition Gazebo for
our simulations, visit the _Motivations_ section of the [website](https://robotology.github.io/gym-ignition). 

## Setup

1. Install the Ignition suite following the [official instructions](https://ignitionrobotics.org/docs/edifice).
1. Execute `pip install gym-ignition`, preferably in a virtual environment.

**Note**: `gym-ignition` currently only supports the latest version of the ignition suite. For more information on supported versions please refer to the [Support Policy](https://robotology.github.io/gym-ignition/master/installation/support_policy.html).


Then, for some simple examples, visit the _Getting Started_ section of the [website](https://robotology.github.io/gym-ignition).

You can decide to install only the C++ resources if you are not interested in using Python.
We also offer a constantly updated pre-release channel with the last development updates.
You can find all the details about the different types of installations we support in the [website](https://robotology.github.io/gym-ignition).

||||
|:---:|:---:|:---:|
| ![](https://user-images.githubusercontent.com/469199/99262383-321fb200-281e-11eb-89cc-cc31f590daa3.png) | ![](https://user-images.githubusercontent.com/469199/99263111-0cdf7380-281f-11eb-9cfe-338b2aae0503.png) | ![](https://user-images.githubusercontent.com/469199/99262746-9e021a80-281e-11eb-9df1-d70134b0801a.png) |

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
