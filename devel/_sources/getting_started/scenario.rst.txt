.. _getting_started_scenario:

ScenarIO
========

In this getting started section we show how to use the Gazebo ScenarIO library to simulate a pendulum system.
We will use the models of the ground plane and the pendulum stored in the repository
`gym_ignition_models <https://github.com/robotology/gym-ignition-models>`_.

The final outcome of this section is shown in the following GIF:

.. figure:: https://user-images.githubusercontent.com/469199/99263551-a9097a80-281f-11eb-98de-714c69385b06.png
   :align: center

.. tip::
    We fully support `Ignition Fuel <https://app.ignitionrobotics.org/dashboard>`_, a constantly enlarging database of SDF models.
    You can use :py:meth:`~scenario.bindings.gazebo.get_model_file_from_fuel` to download any model of the database:

    .. code-block:: python

      from scenario import gazebo as scenario_gazebo

      model_name = "Electrical Box"

      model_file = scenario_gazebo.get_model_file_from_fuel(
          uri=f"https://fuel.ignitionrobotics.org/openrobotics/models/{model_name}")

.. _getting_started_scenario_python:

Python
******

.. tabs::
  .. group-tab:: example.py

    .. code-block:: python

      import time
      import gym_ignition_models
      from scenario import gazebo as scenario_gazebo

      # Create the simulator
      gazebo = scenario_gazebo.GazeboSimulator(step_size=0.001,
                                               rtf=1.0,
                                               steps_per_run=1)

      # Initialize the simulator
      gazebo.initialize()

      # Get the default world and insert the ground plane
      world = gazebo.get_world()
      world.insert_model(gym_ignition_models.get_model_file("ground_plane"))

      # Select the physics engine
      world.set_physics_engine(scenario_gazebo.PhysicsEngine_dart)

      # Open the GUI
      gazebo.gui()
      time.sleep(3)
      gazebo.run(paused=True)

      # Insert a pendulum
      world.insert_model(gym_ignition_models.get_model_file("pendulum"))
      gazebo.run(paused=True)
      time.sleep(3)

      # Get the pendulum model
      pendulum = world.get_model("pendulum")

      # Reset the pole position
      pendulum.get_joint("pivot").to_gazebo().reset_position(0.01)
      gazebo.run(paused=True)
      time.sleep(3)

      # Simulate 30 seconds
      for _ in range(int(30.0 / gazebo.step_size())):
          gazebo.run()

      # Close the simulator
      time.sleep(5)
      gazebo.close()

.. _getting_started_scenario_cpp:

C++
***

.. tabs::
  .. group-tab:: example.cpp

    .. code-block:: cpp

      #include <scenario/gazebo/GazeboSimulator.h>
      #include <scenario/gazebo/Joint.h>
      #include <scenario/gazebo/Model.h>
      #include <scenario/gazebo/World.h>

      #include <chrono>
      #include <string>
      #include <thread>

      int main(int argc, char* argv[])
      {
          // Create the simulator
          auto gazebo = scenario::gazebo::GazeboSimulator(
              /*stepSize=*/0.001, /*rtf=*/1.0, /*stepsPerRun=*/1);

          // Initialize the simulator
          gazebo.initialize();

          // Get the default world
          auto world = gazebo.getWorld();

          // Insert the ground plane
          const std::string groundPlaneSDF = "ground_plane.sdf";
          world->insertModel(groundPlaneSDF);

          // Select the physics engine
          world->setPhysicsEngine(scenario::gazebo::PhysicsEngine::Dart);

          // Open the GUI
          gazebo.gui();
          std::this_thread::sleep_for(std::chrono::seconds(3));
          gazebo.run(/*paused=*/true);

          // Insert a pendulum
          const std::string pendulumURDF = "pendulum.urdf";
          world->insertModel(/*modelFile=*/pendulumURDF);
          gazebo.run(/*paused=*/true);

          // Get the pendulum
          auto pendulum = world->getModel(/*modelName=*/"pendulum");

          // Reset the pole position
          auto pivot = pendulum->getJoint("pivot");
          auto pivotGazebo = std::static_pointer_cast<scenario::gazebo::Joint>(pivot);
          pivotGazebo->resetPosition(0.001);

          // Simulate 30 seconds
          for (size_t i = 0; i < 30.0 / gazebo.stepSize(); ++i) {
              gazebo.run();
          }

          // Close the simulator
          std::this_thread::sleep_for(std::chrono::seconds(3));
          gazebo.close();

          return 0;
      }

  .. group-tab:: CMakeLists.txt

    .. code-block:: cmake

      cmake_minimum_required(VERSION 3.16)
      project(ExampleWithScenario VERSION 1.0)

      set(CMAKE_CXX_STANDARD 17)
      set(CMAKE_CXX_STANDARD_REQUIRED ON)

      find_package(Scenario COMPONENTS Gazebo REQUIRED)

      add_executable(ExampleWithScenario example.cpp)

      target_link_libraries(ExampleWithScenario PRIVATE
          ScenarioGazebo::ScenarioGazebo
          ScenarioGazebo::GazeboSimulator)

.. note::

    The environment should be properly configured to find the plugins and the models.
    Use ``IGN_GAZEBO_SYSTEM_PLUGIN_PATH`` for the plugins and ``IGN_GAZEBO_RESOURCE_PATH`` for the models and meshes.
