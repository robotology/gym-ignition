.. _installation_intro:

Introduction
============

**gym-ignition** is an hybrid C++ and Python project.
In this instructions you will find how to install the project whether you are a downstream *user* or a gym-ignition *developer*.
You will also be asked to choose one of the channels we provide: *stable* and *nightly*.
Depending on the desired setup, gym-ignition could require to find in the system a number of system dependencies and development tools.

The code we developed supports all of the major operating systems.
However, not all our dependencies are yet multi-platform.
Particularly, the simulator we've chosen, Ignition Gazebo, officially supports only GNU/Linux and macOS.
There's nothing that prevents running it on Windows, but at the time of writing obtaining all the dependencies is still quite complicated.

In this initial stage of development, **gym-ignition officially supports only the Ubuntu distribution**.

.. important::
    Our policy is to support the most recent LTS distribution, currently Ubuntu 18.04 Bionic, until the first minor release of the next LTS will be released.
    At the time of writing, we're waiting the release of Ubuntu Focal 20.04.1 before switching to the new LTS.

.. note::
    External contributions to add the support of other platforms are most welcome.

.. admonition:: Fun fact

    In the same spirit of `ubuntu/+bug/1 <https://bugs.launchpad.net/ubuntu/+bug/1>`_, we have our own :issue:`1`.

.. _installation_intro_cpp:

C++
***

gym-ignition requires a modern compiler with C++17 support:

- gcc 8
- clang 6

They aren't the default compiler on the current Ubuntu 18.04 distribution, but they can be easily installed from the official repositories.

.. _installation_intro_python:

Python
******

Our policy regarding the minimum supported Python version follows our Ubuntu policy.
We provide official support of the default Python version of the targeted Ubuntu distribution.
At the time of writing, Python 3.6 is the default version on Ubuntu 18.04.
As soon as we update to Ubuntu 20.04, we will bump the Python version to 3.7.

gym-ignition can be installed either from sources or from PyPI packages (see below for more details).
We provide ``sdist`` packages that can be installed on all systems that meet the dependencies requirements, and ``wheel`` packages for the supported Python version.

You can find `gym-ignition <https://pypi.org/project/gym-ignition/>`_ in the Python Package Index.
Stable and pre-release packages are mapped respectively to our Stable and Nightly channels.

.. warning::
    Our ``wheel`` packages are compiled against the supported version of the Ignition Robotics suite.
    Make sure to install the right release for the channel you selected.

.. _installation_intro_ignition_robotics:

Ignition Robotics
*****************

The main dependency of gym-ignition is the `Ignition Robotics <https://ignitionrobotics.org>`_ suite.
Our policy is the following:

- The **Stable** channel (the ``master`` branch) always targets a released and packaged version of Ignition Robotics.
  At the time of writing is `Citadel <https://ignitionrobotics.org/docs/all/releases>`_.
- The **Nightly** channel (the ``devel`` branch) could target either a unreleased version of Ignition Robotics or custom branches of our forks.
  The suite has to be installed from sources.

The nightly channel could be merged into the stable channel only when it is fully functional on a released version of Ignition Robotics.
We try to follow the quickest release cycle that comply with this only constraint.

.. _installation_intro_users_and_developers:

Users and Developers
********************

gym-ignition is a meta-environment framework.
The aim is not providing robotic environments ready to be used, but providing a framework to create such environments.

**Users** of gym-ignition are all those people that are interested in using the framework to create their own environments.
Most likely, you're one of our users.

**Developers** of gym-ignition are all those people that are interested in contributing to the gym-ignition project.
A developer installation provides a simplified setup for development, VCS integration, and debugging.
The installation process of the C++ and Python components is separated. You can omit the installation of the Python package, if you're interested in using only our C++ resources.

.. note::
    *User* and *Developer* installations can select either the **Stable** or the **Nightly** channel.
