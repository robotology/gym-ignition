<p align="center">
<h1 align="center">gym-ignition</h1>
</p>

<div align="center">
<table>
    <tbody>
         <tr>
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
</div>

||||
|:---:|:---:|:---:|
| ![][pendulum] | ![][panda] | ![][icub] |

[icub]: https://user-images.githubusercontent.com/469199/99262746-9e021a80-281e-11eb-9df1-d70134b0801a.png
[panda]: https://user-images.githubusercontent.com/469199/99263111-0cdf7380-281f-11eb-9cfe-338b2aae0503.png
[pendulum]: https://user-images.githubusercontent.com/469199/99262383-321fb200-281e-11eb-89cc-cc31f590daa3.png

## Description

**gym-ignition** is a framework to create **reproducible robotics environments** for reinforcement learning research.

It is based on the [ScenarIO](scenario/) project which provides the low-level APIs to interface with the Ignition Gazebo simulator.
By default, RL environments share a lot of boilerplate code, e.g. for initializing the simulator or structuring the classes
to expose the `gym.Env` interface.
Gym-ignition provides the [`Task`](python/gym_ignition/base/task.py) and [`Runtime`](python/gym_ignition/base/runtime.py)
abstractions that help you focusing on the development of the decision-making logic rather than engineering.
It includes [randomizers](python/gym_ignition/randomizers) to simplify the implementation of domain randomization
of models, physics, and tasks.
Gym-ignition also provides powerful dynamics algorithms compatible with both fixed-base and floating-based robots by
exploiting [robotology/idyntree](https://github.com/robotology/idyntree/) and exposing
[high-level functionalities](python/gym_ignition/rbd/idyntree).

Gym-ignition does not provide out-of-the-box environments ready to be used.
Rather, its aim is simplifying and streamlining their development.
Nonetheless, for illustrative purpose, it includes canonical examples in the
[`gym_ignition_environments`](python/gym_ignition_environments) package.

Visit the [website][website] for more information about the project.

[website]: https://robotology.github.io/gym-ignition

## Installation

1. First, follow the installation instructions of [ScenarIO](scenario/).
2. `pip install gym-ignition`, preferably in a [virtual environment](https://docs.python.org/3.8/tutorial/venv.html).

## Contributing

You can visit our community forum hosted in [GitHub Discussions](https://github.com/robotology/gym-ignition/discussions).
Even without coding skills, replying user's questions is a great way of contributing.
If you use gym-ignition in your application and want to show it off, visit the
[Show and tell](https://github.com/robotology/gym-ignition/discussions/categories/show-and-tell) section!
You can advertise there your environments created with gym-ignition.

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

---

**Disclaimer:** Gym-ignition is an independent project and is not related by any means to OpenAI and Open Robotics.
