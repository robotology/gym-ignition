.. _what_is_scenario:

What is ScenarIO
================

**ScenarIO** is a C++ abstraction layer to interact with simulated and real robots.

It mainly provides the following
`C++ interfaces <https://github.com/robotology/gym-ignition/tree/master/scenario/core/include/scenario/core>`_:

- :cpp:class:`scenario::core::World`
- :cpp:class:`scenario::core::Model`
- :cpp:class:`scenario::core::Link`
- :cpp:class:`scenario::core::Joint`

These interfaces can be implemented to operate on different scenarios,
including robots operating on either simulated worlds or in real-time.

ScenarIO currently fully implements **Gazebo ScenarIO** (APIs),
a simulated back-end that interacts with `Ignition Gazebo <https://ignitionrobotics.org>`_.
The result allows stepping the simulator programmatically, ensuring a fully reproducible behaviour.
It relates closely to other projects like
`pybullet <https://github.com/bulletphysics/bullet3>`_ and `mujoco-py <https://github.com/openai/mujoco-py>`_.

A real-time backend that interacts with the `YARP <https://github.com/robotology/yarp>`_ middleware is under development.

ScenarIO can be used either from C++ (:ref:`Core APIs <scenario_core>`, :ref:`Gazebo APIs <scenario_gazebo>`)
or from Python (:py:mod:`~scenario.bindings.core`, :py:mod:`~scenario.bindings.gazebo`).

If you're interested to know the reasons why we started developing ScenarIO and why we selected Ignition Gazebo
for our simulations, visit the :ref:`Motivations <motivations>` section.
