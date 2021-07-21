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
