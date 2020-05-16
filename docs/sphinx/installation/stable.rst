.. _installation_stable:

Stable Channel
==============

The stable channel is the easiest way to setup your system.
No compilation from sources is necessary (optional dependencies excluded).

.. hint::
    At the time of writing, **Stable** is quite behind **Nightly**.
    We recently rolled into ``devel`` a major refactoring and we warmly recommend its usage.
    We also temporarily set ``devel`` as main branch.
    The price to pay is a more complex installation, but it will pay off saving a downstream refactoring as soon as we merge it into **Stable**.

.. _installation_stable_common:

Common steps
************

Ignition Robotics
^^^^^^^^^^^^^^^^^

The stable channel depends on Ignition Robotics **Citadel**.
Install the suite following the upstream `binary installation <https://ignitionrobotics.org/docs/citadel/install_ubuntu>`_ instructions.

Virtual Environment (optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This step is optional but highly recommended.
Visit the `virtual environments <https://docs.python.org/3.6/tutorial/venv.html>`_ documentation for more details.

.. code-block:: bash

   sudo apt install virtualenv
   virtualenv -p python3.6 $HOME/venv
   source $HOME/venv/bin/activate

Note that the activation is temporary and it is valid only in the same terminal.
Make sure to execute the next steps in a terminal where the virtual environment is active.

.. _installation_stable_user:

User Installation
*****************

Install the `gym-ignition <https://pypi.org/project/gym-ignition/>`_ package from PyPI:

.. code-block:: bash

   pip install gym-ignition

.. _installation_stable_developer:

Developer Installation
**********************

Make sure that the default compiler of your system meets the :ref:`requirements <installation_intro_cpp>`.

.. tip::
    You can temporarily activate in the current terminal a different compiler by exporting the ``CC`` and ``CXX`` environment variables.
    For example, the following will activate gcc 8:

    .. code-block:: bash

        export CC=gcc-8
        export CXX=g++-8

From the root of the repository, configure, compile, and install the CMake project as follows:

.. code-block:: bash

    mkdir build
    cd build
    cmake ..
    cmake --build .
    cmake --build . --target install

.. note::
    The default install prefix of the CMake project is ``/usr/local``.
    If you want to use a different folder, pass ``-DCMAKE_INSTALL_PREFIX=/new/install/prefix`` to the first ``cmake`` command.

.. attention::
    The SWIG bindings are installed in the `site-packages <https://docs.python.org/3/install/#how-installation-works>`_ folder of the active Python interpreter.
    If you have an active virtual environment, it will be automatically detected.
    Visit `FindPython3 <https://cmake.org/cmake/help/v3.12/module/FindPython3.html>`_ for more details.

Finally, install the Python package.
From the root of the repository:

.. code-block:: bash

    pip install .

.. tip::
    In case of problems with the installation instructions, try to have a look to the `Dockerfile <https://github.com/robotology/gym-ignition/blob/devel/.docker/Dockerfile.cicd-master>`__ we use in our CI/CD pipeline.
    If tests are passing, they will contain all the commands to create a working system that can run the entire software stack.
    If you found some mistake in the instructions above, please open an issue or, better, submit a PR!