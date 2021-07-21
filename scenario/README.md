# ScenarIO

[![C++ standard](https://img.shields.io/badge/standard-C++17-blue.svg?style=flat&logo=c%2B%2B)](https://isocpp.org)
[![Size](https://img.shields.io/github/languages/code-size/robotology/gym-ignition.svg)][scenario]
[![Lines](https://img.shields.io/tokei/lines/github/robotology/gym-ignition)][gym-ignition]
[![Python CI/CD](https://github.com/robotology/gym-ignition/workflows/CI/CD/badge.svg)](https://github.com/robotology/gym-ignition/actions)

[![Version](https://img.shields.io/pypi/v/scenario.svg)][pypi]
[![Python versions](https://img.shields.io/pypi/pyversions/scenario.svg)][pypi]
[![Status](https://img.shields.io/pypi/status/scenario.svg)][pypi]
[![Format](https://img.shields.io/pypi/format/scenario.svg)][pypi]
[![License](https://img.shields.io/pypi/l/scenario.svg)][pypi]

[pypi]: https://pypi.org/project/scenario/
[gym-ignition]: https://github.com/robotology/gym-ignition
[scenario]: https://github.com/robotology/gym-ignition/tree/master/scenario

**SCEN**e interf**A**ces for **R**obot **I**nput / **O**utput.

||||
|:---:|:---:|:---:|
| ![][pendulum] | ![][panda] | ![][icub] |

[icub]: https://user-images.githubusercontent.com/469199/99262746-9e021a80-281e-11eb-9df1-d70134b0801a.png
[panda]: https://user-images.githubusercontent.com/469199/99263111-0cdf7380-281f-11eb-9cfe-338b2aae0503.png
[pendulum]: https://user-images.githubusercontent.com/469199/99262383-321fb200-281e-11eb-89cc-cc31f590daa3.png

## Description

**ScenarIO** is a C++ abstraction layer to interact with simulated and real robots.

It mainly provides the following 
[C++ interfaces](https://github.com/robotology/gym-ignition/tree/master/scenario/core/include/scenario/core):

- `scenario::core::World`
- `scenario::core::Model`
- `scenario::core::Link`
- `scenario::core::Joint`

These interfaces can be implemented to operate on different scenarios, 
including robots operating on either simulated worlds or in real-time.

ScenarIO currently fully implements **Gazebo ScenarIO**, 
a simulated back-end that interacts with [Ignition Gazebo](https://ignitionrobotics.org).
The result allows stepping the simulator programmatically, ensuring a fully reproducible behaviour.
It relates closely to other projects like
[pybullet](https://github.com/bulletphysics/bullet3) and [mujoco-py](https://github.com/openai/mujoco-py).

A real-time backend that interacts with the [YARP](https://github.com/robotology/yarp) middleware is under development.

ScenarIO can be used either from C++ ([APIs](https://robotology.github.io/gym-ignition/master/breathe/core.html)) 
or from Python ([APIs](https://robotology.github.io/gym-ignition/master/apidoc/scenario/scenario.bindings.html)).

If you're interested to know the reasons why we started developing ScenarIO and why we selected Ignition Gazebo 
for our simulations, visit the _Motivations_ section of the 
[website][website].

## Installation

ScenarIO only supports a single distribution of the Ignition suite.
Visit our [Support Policy](https://robotology.github.io/gym-ignition/master/installation/support_policy.html)
to check the distribution currently supported.

Then, install the supported Ignition suite following the 
[official instructions](https://ignitionrobotics.org/docs/edifice).

### Python

Execute, preferably in a [virtual environment](https://docs.python.org/3.8/tutorial/venv.html):

```bash
pip install scenario
```

### C++

You can either clone and install the standalone project:

```cmake
git clone https://github.com/robotology/gym-ignition
cd gym-ignition/scenario
cmake -S . -B build/
cmake --build build/ --target install
```

or include it in your CMake project with
[`FetchContent`](https://cmake.org/cmake/help/latest/module/FetchContent.html).

## Usage

You can find some examples that show the usage of ScenarIO in the _Getting Started_ section of the
[website][website].

## Contributing

Please visit the _Limitations_ section of the [website][website] and check the 
[`good first issue`](https://github.com/robotology/gym-ignition/issues?q=is%3Aissue+is%3Aopen+label%3A%22good+first+issue%22)
and
[`help wanted`](https://github.com/robotology/gym-ignition/issues?q=is%3Aissue+is%3Aopen+label%3A%22help+wanted%22)
issues.

You can visit our community forum hosted in [GitHub Discussions](https://github.com/robotology/gym-ignition/discussions).
Even without coding skills, replying user's questions is a great way of contributing.
If you use ScenarIO in your application and want to show it off, visit the 
[Show and tell](https://github.com/robotology/gym-ignition/discussions/categories/show-and-tell) section!

Pull requests are welcome.

For major changes, please open a [discussion](https://github.com/robotology/gym-ignition/discussions)
first to propose what you would like to change.

## Citation

```bibtex
@INPROCEEDINGS{ferigo2020gymignition,
    title={Gym-Ignition: Reproducible Robotic Simulations for Reinforcement Learning},
    author={D. {Ferigo} and S. {Traversaro} and G. {Metta} and D. {Pucci}},
    booktitle={2020 IEEE/SICE International Symposium on System Integration (SII)},
    year={2020},
    pages={885-890},
    doi={10.1109/SII46433.2020.9025951}
} 
```

## License

[LGPL v2.1](https://choosealicense.com/licenses/lgpl-2.1/) or any later version.

We vendor some resources from the Ignition code base.
For this reason, Gazebo ScenarIO is double-licensed with the 
[Apache License](https://choosealicense.com/licenses/apache-2.0/).

[website]: https://robotology.github.io/gym-ignition
