FAQ
===

.. _faq_citation:

How to give credit?
-------------------

If you use **ScenarIO** or **gym-ignition** for your research,
please cite the following reference:

.. code-block:: bibtex
   :caption: BibTeX entry

   @INPROCEEDINGS{ferigo2020gymignition,
     title={Gym-Ignition: Reproducible Robotic Simulations for Reinforcement Learning},
     author={D. {Ferigo} and S. {Traversaro} and G. {Metta} and D. {Pucci}},
     booktitle={2020 IEEE/SICE International Symposium on System Integration (SII)},
     year={2020},
     pages={885-890},
     doi={10.1109/SII46433.2020.9025951}
   }

Interaction with Tensorflow
---------------------------

If your Python application imports both ``scenario`` and ``tensorflow``,
you might experience segfaults with no error messages.
Likely the problem is the `protobuf <https://github.com/protocolbuffers/protobuf>`_ library.
In fact, both Tensorflow and Ignition Gazebo link agains protobuf, but while Gazebo uses the
default version of your OS, Tensorflow vendors a more recent version.
If you import ``scenario`` before ``tensorflow``, the system protobuf is loaded, and
Tensorflow will segfault.

The only workaround we found is importing Tensorflow first:

>>> import tensorflow
>>> import scenario.bindings.gazebo

Ogre2 and OpenGL
----------------

On GNU/Linux distributions that ship an old OpenGL version, the GUI could fail to open printing
error like *Unable to create the rendering window*.
The reason is that Ignition Gazebo has `ogre-next <https://github.com/OGRECave/ogre-next>`_
(also known as ogre2) as default rendering engine, and it requires OpenGL greater than 3.3.
You can find more details `here <https://github.com/ignitionrobotics/docs/blob/master/fortress/troubleshooting.md#unable-to-create-the-rendering-window>`_.

The workaround we recommend is modifying the file ``~/.ignition/gazebo/gui.config`` as follows:

.. code-block:: diff

   --- .ignition/gazebo/gui.config 2020-06-04 14:41:33.471804733 +0200
   +++ .ignition/gazebo/gui.config 2020-06-04 14:42:47.826475035 +0200
   @@ -30,7 +30,7 @@
        <property type='bool' key='showTitleBar'>false</property>
        <property type='string' key='state'>docked</property>
      </ignition-gui>
   -  <engine>ogre2</engine>
   +  <engine>ogre</engine>
      <scene>scene</scene>
      <ambient_light>0.4 0.4 0.4</ambient_light>
      <background_color>0.8 0.8 0.8</background_color>

After this modification, world SDF files that do no specifically ask ``ogre2`` will use
``ogre`` as default rendering engine which works also with older OpenGL versions.

Default SDF world
-----------------

If not specified differently, the :cpp:class:`~scenario::gazebo::GazeboSimulator`
class uses a default world file that is completely empty, without even the ground plane.
You can load a custom SDF world using :cpp:func:`~scenario::gazebo::GazeboSimulator::insertWorldFromSDF()` and then
extracting it from the simulator with :cpp:func:`~scenario::gazebo::GazeboSimulator::getWorld()` using its name.

.. code-block:: xml
   :caption: ``empty.world``: use this file as starting point for your custom world

   <?xml version="1.0" ?>
   <sdf version="1.7">
       <world name="default">
           <physics default="true" type="ignored">
           </physics>
           <light type="directional" name="sun">
               <cast_shadows>true</cast_shadows>
               <pose>0 0 10 0 0 0</pose>
               <diffuse>0.8 0.8 0.8 1</diffuse>
               <specular>0.2 0.2 0.2 1</specular>
               <attenuation>
                   <range>1000</range>
                   <constant>0.9</constant>
                   <linear>0.01</linear>
                   <quadratic>0.001</quadratic>
               </attenuation>
               <direction>-0.5 0.1 -0.9</direction>
           </light>
       </world>
   </sdf>

.. note::

   If you don't specify any GUI configuration, the default ``~/.ignition/gazebo/gui.config`` is used.
   This is the preferred approach since it's easier to maintain and keep world files updated.
   You can find more information in the upstream `default.sdf <https://github.com/ignitionrobotics/ign-gazebo/blob/master/examples/worlds/default.sdf>`_.

.. tip::

   You don't need to add the physics plugin in the world file. You can use
   :cpp:func:`scenario::gazebo::World::setPhysicsEngine()` from your code.
   You can also load other plugins during runtime using
   :cpp:func:`scenario::gazebo::World::insertWorldPlugin()` and
   :cpp:func:`scenario::gazebo::Model::insertModelPlugin()`.
