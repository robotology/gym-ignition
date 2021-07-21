.. _installation_developer:

Developer Installation
======================

This installation type is intended for those that want to contribute to the development of the project.
Compared to :ref:`Stable <installation_stable>` and :ref:`Nightly <installation_nightly>`,
this installation type provides a simplified setup for development, VCS integration, and debugging.

Depending on whether you want to target the ``Stable`` or ``Nightly`` channels,
you have to clone respectively the ``master`` or ``devel`` branch.
Check our :ref:`support policy <support_policy>` to select the right distribution of Ignition to install.

Dependencies
************

The developer installation requires finding in the system other dependencies not required by the other installation types.
In those cases, we rely on setuptools to download, install, and find all the necessary third-party dependencies.

1. **iDynTree**: ``gym_ignition`` provides helper classes to manipulate the kinematics and the dynamics of rigid-bodies.
   Among the many existing solutions, we selected the algorithms implemented in `iDynTree <https://github.com/robotology/idyntree/>`_.

   Follow the `official installation instructions <https://github.com/robotology/idyntree/#installation>`__ and make sure
   that you also enable and install the `Python bindings <https://github.com/robotology/idyntree/#bindings>`__.

   You can verify that the installation succeeded and your system is properly configured if you can ``import idyntree.bindings`` in a Python interpreter.

C++ Project
***********

From the root of the repository and from the branch you are interested, you can configure, compile, and install
the CMake project as follows:

.. code-block:: bash

    cd scenario/
    cmake -S . -B build
    cmake --build build/
    cmake --build build/ --target install

.. note::

    The default install prefix of the CMake project is ``/usr/local``.
    If you want to use a different folder, pass ``-DCMAKE_INSTALL_PREFIX=/new/install/prefix`` to the first ``cmake`` command.

.. attention::

    The SWIG bindings are installed in the `site-packages <https://docs.python.org/3/install/#how-installation-works>`_
    folder of the active Python interpreter.
    If you have an active virtual environment, it will be automatically detected.
    We rely on CMake's logic for detecting Python,
    visit `FindPython3 <https://cmake.org/cmake/help/v3.16/module/FindPython3.html>`_ for more details.

.. include:: virtual_environment.rst

Editable Python Installation
****************************

Install the Python package in `editable mode <https://pip.pypa.io/en/stable/reference/pip_install/#editable-installs>`_.
From the root of the repository:

.. code-block:: bash

    pip install -e scenario/
    pip install -e .

The editable installation only symlinks the resources of the repository into the active Python installation.
It allows to develop directly operating on the files of the repository and use the updated package without requiring
to install it again.

.. note::

    The ``scenario`` editable installation is just a placeholder.
    It is necessary to prevent the editable installation of ``gym-ignition`` to override the resources installed by
    the manual CMake execution.
    Otherwise, the ``scenario`` package from PyPI would be pulled, resulting with a wrong version.

.. include:: system_configuration.rst
