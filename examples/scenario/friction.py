# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from typing import List
from scenario import core as scenario_core
from scenario import gazebo as scenario_gazebo
from gym_ignition.utils.scenario import init_gazebo_sim
import tempfile
import time

###############################
# Experiment Configuration
###############################

# Render scene flag
GUI = True

# Set the verbosity
scenario_gazebo.set_verbosity(scenario_gazebo.Verbosity_warning)

num_cubes = 7   # Number of cubes
cubes_colors = ((1., 0., 0., 1.),   # Cubes color codes
                (0., 1., 0., 1.),
                (0., 0., 1., 1.),
                (1., 1., 0., 1.),
                (1., 0., 1., 1.),
                (0., 1., 1., 1.),
                (1., 1., 1., 1.))

cubes_coulomb_frictions = (0., 0.5, 1., 3., 100., 0.5, 0.5)     # Cubes Coulomb friction coefficients
slip_coefficients = (0., 0., 0., 0., 0., 0., 1.)                # Cubes slip coefficients
applied_force_magnitudes = (20., 20., 20., 20., 20., 20., 20.)  # Magnitudes of applied forces
force_duration = 0.2                                            # Duration of applied forces


# Helper function returning the URDF of a cube
# TODO: Use @dataclass approach with urdf() method
def get_cube_urdf(mass: float = 5.0,
                  edge: float = 0.2,
                  color: tuple = (1, 1, 1, 1),
                  coulomb_friction: float = 50.0,
                  slip: float = 0.0) -> str:

    i = 1 / 12 * mass * (edge ** 2 + edge ** 2)
    cube_urdf = f"""
    <robot name="cube_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
        <!-- ====== -->
        <!-- COLORS -->
        <!-- ====== -->
        <material name="custom">
            <color rgba="{color[0]} {color[1]} {color[2]} {color[3]}"/>
        </material>
        <gazebo reference="cube">
            <visual>
              <material>
                <diffuse>{color[0]} {color[1]} {color[2]} {color[3]}</diffuse>
              </material>
            </visual>
            <collision>
                <surface>
                  <friction>
                    <ode>
                        <mu>{coulomb_friction}</mu>
                        <mu2>{coulomb_friction}</mu2>
                        <slip1>{slip}</slip1>
                        <slip2>{slip}</slip2>
                    </ode>
                  </friction>
                </surface>
            </collision>
        </gazebo>
        <!-- ===== -->
        <!-- LINKS -->
        <!-- ===== -->
        <link name="cube">
            <inertial>
              <origin rpy="0 0 0" xyz="0 0 0"/>
              <mass value="{mass}"/>
              <inertia ixx="{i}" ixy="0" ixz="0" iyy="{i}" iyz="0" izz="{i}"/>
            </inertial>
            <visual>
              <geometry>
                <box size="{edge} {edge} {edge}"/>
              </geometry>
              <origin rpy="0 0 0" xyz="0 0 0"/>
                <material name="custom">
                    <color rgba="{color[0]} {color[1]} {color[2]} {color[3]}"/>
                </material>
            </visual>
            <collision>
              <geometry>
                <box size="{edge} {edge} {edge}"/>
              </geometry>
              <origin rpy="0 0 0" xyz="0 0 0"/>
              <surface>
                <friction>
                    <ode>
                        <mu>{coulomb_friction}</mu>
                        <mu2>{coulomb_friction}</mu2>
                        <slip1>{slip}</slip1>
                        <slip2>{slip}</slip2>
                    </ode>
                </friction>
              </surface>
            </collision>
        </link>
    </robot>"""
    return cube_urdf


# Get the default simulator and the default empty world
gazebo, world = init_gazebo_sim()

# Insert cube models specifying the pose and the model name
for i in range(num_cubes):
    model_name = "cube_" + str(i)
    model_pose = scenario_core.Pose([0., 3.0 - i, 0.25], [1., 0, 0, 0])

    # Write the cube URDF to a temporary file
    handle, model_file = tempfile.mkstemp()
    with open(handle, 'w') as f:
        f.write(get_cube_urdf(mass=0.5,
                              edge=0.5,
                              color=cubes_colors[i],
                              coulomb_friction=cubes_coulomb_frictions[i],
                              slip=slip_coefficients[i]))

    world.insert_model(model_file,
                       model_pose,
                       model_name)

if GUI:
    gazebo.gui()
    time.sleep(5)

# Print names of models in the world
print("Models currently inserted in the world:", world.model_names())

# Get the ground plane model and link
ground_model: scenario_core.Model = world.get_model("ground_plane")
ground_link: scenario_core.Link = ground_model.get_link("link")

# Get the cubes models and links
cube_models: List[scenario_core.Model] = []
cube_links: List[scenario_core.Link] = []
for i in range(num_cubes):
    cube_models.append(world.get_model(model_name="cube_" + str(i)))
    cube_links.append(cube_models[i].get_link("cube"))

if GUI:
    gazebo.run()
    time.sleep(5)

# Apply forces
for i in range(num_cubes):
    force = [applied_force_magnitudes[i], 0., 0.]
    cube_links[i].apply_world_force(force, force_duration)

# Execute simulation
for i in range(10000):
    gazebo.run()

# Print traveled distances along X
print("Traveled distance (x):")
for i in range(num_cubes):
    print("Cube #" + str(i) + ": " + '%.2f' % cube_links[i].position()[0])
print()

# Print linear velocities along X
print("Linear velocity(x):")
for i in range(num_cubes):
    print("Cube #" + str(i) + ": " + '%.2f' % cube_links[i].body_linear_velocity()[0])

if GUI:
    time.sleep(5)
