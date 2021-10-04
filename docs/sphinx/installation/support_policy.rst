.. _support_policy:

Support policy
==============

**gym-ignition** is an hybrid C++ and Python project and it requires finding in the system updated compile and runtime
dependencies, depending on the installation type you select.

The project mostly supports all the major operating systems.
However, we are currently using and testing only GNU/Linux systems.
We do not yet provide official support to other operating systems.

The table below recaps the project requirements of the :ref:`Stable <installation_stable>` and :ref:`Nightly <installation_nightly>` channels:

+-------------+-----------------+--------+----------------------+----------+------------+---------+
| Channel     |       C++       | Python |      Ignition        |  Ubuntu  | macOS [*]_ | Windows |
+=============+=================+========+======================+==========+============+=========+
| **Stable**  | >= gcc8, clang6 | >= 3.8 | `Fortress`_ (binary) | >= 20.04 |     No     |    No   |
+-------------+-----------------+--------+----------------------+----------+------------+---------+
| **Nightly** | >= gcc8, clang6 | >= 3.8 | `Fortress`_ (source) | >= 20.04 |     No     |    No   |
+-------------+-----------------+--------+----------------------+----------+------------+---------+

.. _`Fortress`: https://ignitionrobotics.org/docs/fortress/install

.. [*] Ignition officially supports macOS and also ``gym-ignition`` could be installed on this platform.
       However, we do not currently test this configuration and we cannot guarantee support.

.. important::

    Our policy is to support Ubuntu from the most recent LTS distribution, currently Ubuntu 20.04 Focal.
    We typically switch to a new LTS when the first minor release ``YY.MM.1`` is released.

    The Python and compilers policies follow a similar approach, we try to keep them updated as much as
    possible following what the supported LTS distribution includes.

.. note::

    External contributions to extend the support and provide feedback about other platforms are most welcome.

.. admonition:: Fun fact

    In the same spirit of `ubuntu/+bug/1 <https://bugs.launchpad.net/ubuntu/+bug/1>`_, we have our own :issue:`1`.
