.. _scenario_and_gym_ignition:

ScenarIO and gym-ignition
=========================

This project targets both *control* and *robot learning* research domains:

- Researchers in robotics and control can simulate their robots with familiar tools like Gazebo and URDF/SDF,
  without the need to rely on any middleware.
- Researchers in robot learning can quickly develop new robotic environments that can scale to hundreds of parallel instances.

We provide two related subprojects to each of these categories:

1. **ScenarIO** provides APIs to interface with the robots.
2. **gym-ignition** helps structuring environments compatible with OpenAI Gym,
   while minimizing boilerplate code and providing common rigid-body dynamics utilities.

Check the sections :ref:`What is ScenarIO <what_is_scenario>` and
:ref:`What is gym-ignition <what_is_gym_ignition>` for more details,
and visit :ref:`Motivations <motivations>` for an extended overview.

For a quick practical introduction, visit the :ref:`Getting Started <getting_started_scenario>` page.

If you use this project for your research, please check the FAQ about :ref:`how to give credit <faq_citation>`.

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
   :caption: What

   what/what_is_scenario
   what/what_is_gym_ignition

.. toctree::
   :hidden:
   :maxdepth: 1
   :caption: Why

   why/motivations
   why/why_scenario
   why/why_ignition_gazebo
   why/why_gym_ignition

.. toctree::
   :hidden:
   :maxdepth: 1
   :caption: Installation (How)

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
   :caption: ScenarIO C++ API:

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
