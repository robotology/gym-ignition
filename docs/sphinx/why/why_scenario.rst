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
   Let's say you already have built your functionality with the backends we provide,
   and you are not happy from the performance of the simulator we used.
   You can implement your own simulation backend and run it alongside those we provide.
   The same applies to the real-time backend, in case your robot uses a custom middleware or SDK.

.. tip::

   So far, we always referred to the C++ abstraction layer provided by ScenarIO.
   The interface / implementation pattern is implemented with classic inheritance and polymorphism.
   Having such unified interface simplifies the process to expose it to other languages.
   Thanks to SWIG, we officially provide Python bindings of ScenarIO,
   so that you can prototype your applications even faster!
