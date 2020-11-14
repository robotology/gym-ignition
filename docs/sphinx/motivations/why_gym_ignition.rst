.. _motivations:

In this section we recap the motivations behind gym-ignition.
Choosing the right framework for your research is often challenging and we hope to provide a broader look that could
help deciding whether it meets your needs or not.

The development of the framework is evolving quickly, and the architecture and motivations described in the
`reference publication <https://github.com/robotology/gym-ignition#Citation>`_ are no longer accurate.
This section provides a constantly updated description aligned with the most recent development news.

.. _why_scenario:

Why ScenarIO
============

*SCENe interfAces for Robot Input/Output* is an abstraction layer to interface with robots.
It exposes APIs to interact with a scene, providing a :cpp:class:`~scenario::core::World` that can return
:cpp:class:`~scenario::core::Model` objects, from which you can gather measurements and send commands.
The relevant APIs of ScenarIO are documented in the :ref:`Scenario Core <scenario_core>` section.

Many simulators already provide an abstraction of different physics engines.
They expose a unified interface that, after selecting the desired back-end, maps its unified methods to those of the
underlying physics engine. The aim of ScenarIO is extending this idea also to real-time robots.

The simulation backend of ScenarIO communicates with a simulator that itself abstracts physics engines.
This is powerful because, in this way, ScenarIO is independent from the details of the underlying physics engine.
Any current and future physics engine supported by the simulator is compatible with ScenarIO without requiring any
change from our side.

Regarding real-time robots, the APIs of ScenarIO can be implemented exploiting middlewares like ROS or YARP.
At the time of writing there are no official real-time backends, stay tuned for further updates.

Once both the simulation and real-time backends are in place, you can then write code to control your robot just once,
it will interact either with the simulated or the real robot depending on the ScenarIO backend you enabled.

.. note::

   The ScenarIO interface is flexible and generic.
   Let's say you already have built your functionality with the backends we provide, and you are not happy from the performance of the simulator we used.
   You can implement your own simulation backend and run it alongside those we provide.
   The same applies to the real-time backend, in case your robot uses a custom middleware or SDK.

.. tip:

   So far, we always referred to the C++ abstraction layer provided by ScenarIO.
   The interface / implementation pattern is implemented with classic inheritance and polymorphism.
   Having such unified interface simplifies the process to expose it to other languages.
   Thanks to SWIG, we officially provide Python bindings of ScenarIO, so that you can prototype your applications even faster!

.. _why_ignition_gazebo:

Why Ignition Gazebo
===================

In this section we want to go a bit deeper in the motivations that led us to select Ignition Gazebo as target solution for the simulation back-end of ScenarIO.

To begin, a bit of history. The `Gazebo <https://gazebosim.org>`_ simulator that the entire robotic community has used for
over a decade is different than Ignition Gazebo.
We will refer to the old simulator as Gazebo Classic.
Ignition Gazebo is the new generation of the simulator.
It takes all the lesson learned by the development of Gazebo Classic and provides a more modular and extensible framework for robotic simulations.
The monolithic structure of Gazebo Classic has been broken in standalone libraries, obtaining a *suite* called `Ignition <https://ignitionrobotics.org>`_.
Ignition Gazebo is just one of the libraries [*]_ that compose the suite.
When we started the development of gym-ignition, Ignition Gazebo was "stable" enough to start using it.
The clear advantage is support: Gazebo Classic will just receive bug fixing, all the development effort now shifted towards the new Ignition Gazebo.
The price to pay, instead, is that Ignition Gazebo has not yet reached feature parity with Gazebo Classic, even though
the gap is getting filled quickly.

.. [*] Yes, you read correctly: *library*. Ignition Gazebo is a library.
       A `ignition::gazebo::Server` object can be instantiated and it can be stepped programmatically from your code.
       This type of architecture became quite popular recently because it gives you full control of the simulation.
       Ignition Gazebo, therefore, became a solution similar to the well known alternatives like ``pybullet`` and ``mujoco-py``.

We have been and currently are Gazebo Classic users, as many other robotics research labs.
Over time, we became familiar with the tools and methods of Gazebo Classic and built a lot of code and applications that depend on it.
Unfortunately, despite someone has shown attempts, Gazebo Classic is not suitable for the large-scale simulations that are
typical in modern reinforcement learning architectures.
Ignition Gazebo offered us a viable solution for this use case that allows us to exploit the know-how we gained with Gazebo Classic.

The two main features that drove us towards the adoption of Ignition Gazebo are the following:

1. **Physics engines are loaded as plugin libraries and Ignition Gazebo operates on an API-stable interface.**
   This architecture allows everyone to implement a new physics engine backend and run simulations exploiting all the other
   components of the Ignition suite (rendering, systems, ...).
   While today only `DART <https://github.com/dartsim/dart>`_ is officially supported, we believe this is one of the best
   attempts to obtain in the long run a framework that allows to switch physics engines with minimal effort.
   For reinforcement learning research, it could bring domain randomization to the next level.
2. **Simulations can be stepped programmatically without relying on network transport, guaranteeing full reproducibility.**
   Reproducible simulations are paramount whether you are prototyping a new robot controller or you are running
   large-scale simulations for robot learning.
   Most of the client-server architectures cannot guarantee reproducibility since asynchronous network transports could
   provide different results depending on the load of your system.
   An effective solution is using the simulator as a library and stepping it programmatically from your code.
   Gazebo ScenarIO provides APIs to perform these kind of simulations with Ignition Gazebo.

There are a bunch of other nice features we didn't cover in this section.
Not all of them are currently exposed to ScenarIO Gazebo, please open a feature request if you have any suggestion or,
even better, fire up a pull request!

To summarize, these are the features that motivated us to choose Ignition Gazebo:

- Simulator developed for robotics
- Simulator-as-a-library structure
- Abstraction of different physics engines and rendering engines
- Modular software architecture
- Powerful and constantly improving SDF model description
- Well maintained, packaged, and widely tested
- Large big database of objects to create worlds: `Ignition Fuel <https://app.ignitionrobotics.org/dashboard>`_
- Long term vision and support

.. note::

   Ignition Gazebo is the target simulator of the new `DARPA Subterranean Challenge <https://subtchallenge.com>`_.
   Have a look to their simulation results to understand what you can expect from using Ignition Gazebo.

.. _why_gym_ignition:

Why gym-ignition
================

In the previous sections we described why we developed ScenarIO and why we used Ignition Gazebo to implement its simulation back-end.
While we mentioned few advantages for the robot learning domain, ScenarIO remains a general C++ library that can be used for generic robotic applications.

The reinforcement learning community, in the past years, converged towards Python for the development of the environments
containing the decision-making logic.
`OpenAI Gym <https://gym.openai.com>`_ became the reference interface to provide a clear separation between agents and environments.
A widespread interface is powerful because if you implement an environment that exposes the ``gym.Env`` interface, you can then use
all the countless frameworks provided by the community to train a policy selecting your favourite algorithm.

The Python package ``gym_ignition`` enables you to create robotic environments compatible with OpenAI Gym.
Being based on ScenarIO, it enables to develop environments that can run not only on different physics engines,
but also on real robots.

You can think of ``gym_ignition`` as a way to help you structuring your environment.
If you know how `pytorch-lightning <https://github.com/PyTorchLightning/pytorch-lightning>`_ relates to PyTorch,
the same applies to the interaction between gym-ignition and ScenarIO.
Thanks to the :py:class:`~gym_ignition.base.task.Task` and :py:class:`~gym_ignition.base.runtime.Runtime` interfaces,
``gym_ignition`` abstracts away all the unnecessary boilerplate that otherwise you have to copy and paste between environments.

For example, :py:class:`~gym_ignition.runtimes.gazebo_runtime.GazeboRuntime` provides all boilerplate code to take
your implementation of a :py:class:`~gym_ignition.base.task.Task` and simulate it with Ignition Gazebo.

Furthermore, we provide useful classes with functionalities that are commonly required by robotic environments, like
Inverse Kinematic (:py:class:`~gym_ignition.rbd.idyntree.inverse_kinematics_nlp.InverseKinematicsNLP`)
and multibody dynamics algorithms (:py:class:`~gym_ignition.rbd.idyntree.kindyncomputations.KinDynComputations`)
with full support of floating-base systems.

.. note::

   Developing environments for robot learning in C++ is a valid choice and the community has shown different examples.
   ScenarIO can be used to develop C++ environments as well, however we find more useful using Python since it allows
   a faster prototyping.

.. note::

   To the best of our knowledge, the first package that implemented a structure that abstracts the task, the robot, and
   the simulator is `openai_ros <http://wiki.ros.org/openai_ros>`_.
   We have been inspired by its structure in the early stage of development, and the current interfaces implemented in
   gym-ignition are an evolution of the original architecture.
