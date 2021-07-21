.. _installation_nightly:

Nightly
=======

The nightly channel contains the most recent updates of the project.
As described in the :ref:`support policy <support_policy>`, this channel requires building Ignition from sources.

We publish updated nightly packages after any pull request merged in the ``devel`` branch.

.. include:: virtual_environment.rst

PyPI Package
************

We provide two different packages for ScenarIO and gym-ignition.

If you are interested in the ScenarIO package,
install the `scenario <https://pypi.org/project/scenario/>`_ package from PyPI:

.. code-block:: bash

   pip install --pre scenario

Instead, if you are interested in gym-ignition,
install the `gym-ignition <https://pypi.org/project/gym-ignition/>`_ package from PyPI:

.. code-block:: bash

   pip install --pre scenario gym-ignition

Note that in this case, specifying also the ``scenario`` dependency is necessary,
otherwise ``pip`` will pull the stable package from PyPI.

.. include:: system_configuration.rst
