.. _installation_stable:

Stable
======

The stable channel is the easiest way to setup your system.
As described in the :ref:`support policy <support_policy>`, this channel allows installing Ignition from binary packages.

We publish updated stable packages after any tagged release of the ``master`` branch.

.. include:: virtual_environment.rst

PyPI Package
************

We provide two different packages for ScenarIO and gym-ignition.

If you are interested in the ScenarIO package,
install the `scenario <https://pypi.org/project/scenario/>`_ package from PyPI:

.. code-block:: bash

   pip install scenario

Instead, if you are interested in gym-ignition,
install the `gym-ignition <https://pypi.org/project/gym-ignition/>`_ package from PyPI:

.. code-block:: bash

   pip install gym-ignition

It will download and install also ``scenario`` since it depends on it.
