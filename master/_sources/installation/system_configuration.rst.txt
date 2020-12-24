System Configuration
********************

This section applies only to the installations that require building Ignition from sources.

If you installed Ignition from sources, you likely used ``colcon`` and therefore the entire suite was installed in a custom folder called workspace.
The workspace contains all the shared libraries and executables of Ignition, including the plugins loaded during runtime.
Since we cannot know in advance where you created the workspace, ``gym-ignition`` is not able to find the physics plugins.

After you enabled the workspace by sourcing its bash script, you may need to also export the following variable:

.. code-block:: bash

   export IGN_GAZEBO_PHYSICS_ENGINE_PATH=${IGN_GAZEBO_PHYSICS_ENGINE_PATH}:${COLCON_PREFIX_PATH}/lib/ign-physics-3/engine-plugins/

Make sure to select the folder corresponding to the correct version of ign-physics, otherwise the wrong plugins are found.
