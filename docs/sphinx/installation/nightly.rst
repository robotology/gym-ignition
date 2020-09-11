.. _installation_nightly:

Nightly Channel
===============

The nightly channel contains the most recent updates of the project.
As described in the :ref:`introduction <installation_intro_ignition_robotics>`,
this channel requires building many dependencies from sources.

Make sure that the default compiler of your system meets the :ref:`requirements <installation_intro_cpp>`.

.. tip::
    You can temporarily activate in the current terminal a different compiler by exporting the ``CC`` and ``CXX`` environment variables.
    For example, the following will activate gcc 8:

    .. code-block:: bash

        export CC=gcc-8
        export CXX=g++-8

.. _installation_nightly_common:

Common steps
************

SWIG
^^^^

The nightly channel depends on SWIG 4.
Execute the following to install it in the default ``/usr/local`` prefix:

.. code-block:: bash

    sudo apt-get update
    sudo apt-get install -y --no-install-recommends \
            autotools-dev automake bison libpcre3-dev
    cd /tmp
    git clone --depth 1 -b rel-4.0.1 https://github.com/swig/swig.git
    cd swig
    sh autogen.sh
    ./configure
    make
    sudo make install

.. hint::
    If you're already on Ubuntu 20.04 Focal, SWIG 4 could be installed from the official repositories.

Ignition Robotics
^^^^^^^^^^^^^^^^^

The nightly channel depends on either unreleased branches of the Ignition Robotics suite or, temporarily, on our forks.

The installation of Ignition Robotics follows the upstream `sources installation <https://ignitionrobotics.org/docs/citadel/install_ubuntu_src>`_.
The main difference is the list of source repositories.
Make sure to use our `tags.yaml <https://github.com/robotology/gym-ignition/blob/devel/.docker/tags.yaml>`_ when `getting the sources <https://ignitionrobotics.org/docs/citadel/install_ubuntu_src#getting-the-sources>`_:

.. code-block:: bash

    wget https://raw.githubusercontent.com/robotology/gym-ignition/devel/.docker/tags.yaml
    vcs import < tags.yaml

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

.. _installation_nightly_user:

User Installation
*****************

Install the pre-release version of the `gym-ignition <https://pypi.org/project/gym-ignition/>`_ package from PyPI:

.. code-block:: bash

   pip install --pre gym-ignition

.. _installation_nightly_developer:

Developer Installation
**********************

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

Finally, install the Python package in `editable mode <https://pip.pypa.io/en/stable/reference/pip_install/#editable-installs>`_.
From the root of the repository:

.. code-block:: bash

    pip install -e .

.. warning::

    The packages of the nightly channel are built from our Continuous Delivery pipeline.
    While uploading ``sdist`` packages always succeeds being no more than a compressed version of git repository, ``bdist_wheel`` packages could be occasionally missing.

    If you're unlucky and you're trying to install the most recent release in this situation, ``pip`` will try to generate the wheel from the ``sdist`` package.
    Everything will happen under the hood, the installation instruction do not differ.
    In case of problems, you can back-up to search in PyPI the latest version packaged as a wheel and tell ``pip`` to install it by passing the precise version.

.. tip::
    In case of problems with the installation instructions, try to have a look to the `Dockerfile <https://github.com/robotology/gym-ignition/blob/devel/.docker/Dockerfile.cicd-devel>`__ we use in our CI/CD pipeline.
    If tests are passing, they will contain all the commands to create a working system that can run the entire software stack.
    If you found some mistake in the instructions above, please open an issue or, better, submit a PR!