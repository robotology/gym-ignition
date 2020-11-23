Limitations
===========

Sensors support
---------------

Ignition Gazebo supports a wide variety of `sensors <https://ignitionrobotics.org/docs/citadel/comparison#sensors>`_,
like cameras, lidars, ...
However, ScenarI/O and consequently gym-ignition are not yet able to extract data from sensors.
Follow :issue:`199` if you're interested in sensors support.

Performance
-----------

When operating on a single model, DART provides good enough performance and accuracy for most of the use-cases.
However, we noticed that it does not perform well when many models are inserted in the simulated world,
especially in contact-rich scenarios.

If your aim is performing simulations for robot learning, we recommend running multiple instances of the simulator,
each of them operating on a world containing a single robot.

If your aim is simulating a big world where the controlled model can move inside the scene, exploiting a new feature
of Ignition Gazebo called `levels <https://github.com/ignitionrobotics/ign-gazebo/blob/ign-gazebo4/tutorials/levels.md>`_
could be the proper solution. This feature is exploited by the big worlds of `subt <https://subtchallenge.com>`_.

Instead, if your world has many models and the usage of levels does not apply to your use-case, you can try to switch
the physics backend to an engine that handles better your simulation.

.. note::

   At the time of writing only DART is officially supported.
   There is some recent activity to implement the bullet physics engine, but this back-and is not yet ready.
   As last resort, you can implement a new physics backend following the
   `instructions <https://github.com/ignitionrobotics/ign-physics/blob/ign-physics3/tutorials/03_physics_plugins.md>`_.
