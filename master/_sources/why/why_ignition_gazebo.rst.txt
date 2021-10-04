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
       A ``ignition::gazebo::Server`` object can be instantiated and it can be stepped programmatically from your code.
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
