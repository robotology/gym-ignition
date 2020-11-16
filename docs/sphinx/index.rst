.. _what_is_gym_ignition:

What is gym-ignition?
=====================

**gym-ignition is a framework to create reproducible robotics environments for reinforcement learning research.**

The aims of the project are the following:

- Provide unified APIs for interfacing with both simulated and real robots.
- Implement the simulation backend interfacing with the Ignition Gazebo simulator.
- Enable a seamless switch of all the physics engines supported by Ignition Gazebo.
- Guarantee the reproducibility and the scalability of the simulations by using the simulator as a library, without
  relying on any network transport.
- Simplify the development of OpenAI Gym environments for robot learning research.

**gym-ignition** targets both *control* and *robot learning* research domains:

- Researchers in robotics and control can simulate their robots with familiar tools like Gazebo and URDF,
  without the need to rely on any middleware.
- Researchers in robot learning can quickly develop new robotic environments that can scale to hundreds of parallel instances.

To know more about why we started developing gym-ignition, why we selected Ignition Gazebo for our simulations,
and what are our long-term goals, visit the :ref:`Motivations <motivations>` page.

We are building an entire ecosystem around gym-ignition, if you're interested have a look to the other projects:

.. list-table::
    :header-rows: 1
    :align: center

    * - ScenarI/O and ``gym_ignition``
      - Robot Models
      - Ignition Plugins
    * - `robotology/gym-ignition <https://github.com/robotology/gym-ignition>`_
      - `robotology/gym-ignition-models <https://github.com/robotology/gym-ignition-models>`_
      - `dic-iit/gazebo-scenario-plugins <https://github.com/dic-iit/gazebo-scenario-plugins>`_

.. list-table::

   * - |pendulum_swing|
     - |panda_grasping|
     - |icub_stepping|

.. |icub_stepping| image:: https://user-images.githubusercontent.com/469199/99262746-9e021a80-281e-11eb-9df1-d70134b0801a.png
.. |panda_grasping| image:: https://user-images.githubusercontent.com/469199/99263111-0cdf7380-281f-11eb-9cfe-338b2aae0503.png
.. |pendulum_swing| image:: https://user-images.githubusercontent.com/469199/99262383-321fb200-281e-11eb-89cc-cc31f590daa3.png

.. toctree::
   :hidden:
   :maxdepth: 1
   :caption: Motivations

   motivations/why_gym_ignition

.. toctree::
   :hidden:
   :maxdepth: 1
   :caption: Installation

   installation/support_policy
   installation/stable
   installation/nightly
   installation/developer

.. toctree::
   :hidden:
   :maxdepth: 1
   :caption: Getting Started

   getting_started/scenario
   getting_started/manipulation
   getting_started/gym-ignition

.. toctree::
   :hidden:
   :maxdepth: 2
   :caption: ScenarI/O C++ API:

   breathe/core
   breathe/gazebo

.. toctree::
   :hidden:
   :maxdepth: 2
   :caption: Python Packages

   apidoc/scenario/scenario.bindings
   apidoc/gym-ignition/gym_ignition
   apidoc/gym-ignition-environments/gym_ignition_environments

.. toctree::
   :hidden:
   :maxdepth: 2
   :caption: Information

   info/faq
   info/limitations
