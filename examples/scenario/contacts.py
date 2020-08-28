# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from scenario import core as scenario_core
from scenario import gazebo as scenario_gazebo

from gym_ignition.utils.scenario import init_gazebo_sim
import tempfile
import time

# Configuration
GUI = True


# Function returning the URDF of a cube
# TODO: Use @dataclass approach with urdf() method
def get_cube_urdf(mass: float = 5.0, edge: float = 0.2) -> str:

    i = 1 / 12 * mass * (edge ** 2 + edge ** 2)
    cube_urdf = f"""
    <robot name="cube_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
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
            </visual>
            <collision>
              <geometry>
                <box size="{edge} {edge} {edge}"/>
              </geometry>
              <origin rpy="0 0 0" xyz="0 0 0"/>
            </collision>
        </link>
    </robot>"""
    return cube_urdf


# Set the verbosity
scenario_gazebo.set_verbosity(scenario_gazebo.Verbosity_warning)

# Get the default simulator and the default empty world
gazebo, world = init_gazebo_sim()

# Insert a cube model specifying the pose and the model name
model_name = "my_cube"
default_pose = scenario_core.Pose([0, 0, 10.], [1., 0, 0, 0])

# Write the cube URDF to a temporary file
handle, model_file = tempfile.mkstemp()
with open(handle, 'w') as f:
    f.write(get_cube_urdf(mass=100.0, edge=2))

world.insert_model(model_file,
                   default_pose,
                   model_name)

if GUI:
    gazebo.gui()
    time.sleep(5)

# Print names of models in the world
print("Models currently inserted in the world:", world.model_names())

# Get the cube  model
cube: scenario_core.Model = world.get_model(model_name=model_name)
print("cube.contacts_enabled():", cube.contacts_enabled())

# Get the cube link
cube_link: scenario_core.Link = cube.get_link("cube")
print("cube_link.contacts_enabled():", cube_link.contacts_enabled())

# The cube was inserted floating in the air at 1m height.
# There should be no contacts.
gazebo.run(paused=True)
print("cube_link.in_contact()", cube_link.in_contact())
print("Number of active contacts on the cube:", len(cube.contacts()))

# Check cube height
print("Initial cube height:", cube.base_position()[2])

# Make the cubes fall for 50ms
while not cube_link.in_contact():
    gazebo.run()
    print("Falling cube height:", cube.base_position()[2])

print("Contact detected!")

# Readout and print contacts information
contacts = cube_link.contacts()

print("Links in contact: [ ", contacts[0].body_a, " | ", contacts[0].body_b, " ]")

print("Number of contact points with the ground:", len(contacts[0].points))

for i in range(len(contacts[0].points)):
    print("Contact # ", i,  "point position:", contacts[0].points[i].position)  # Contact point position
    print("Contact # ", i,  "force:", contacts[0].points[i].force)              # Contact forces

# Compute and print sum of reaction forces on the cube
total_force = 0
for i in range(len(contacts[0].points)):
    total_force += contacts[0].points[i].force[2]

print("Total vertical reaction force at contact instant:", total_force, "N")
print("Cube mass:", cube_link.mass(), "Kg")

# Wait for the cube to stop
for i in range(100):
    gazebo.run()

# Recompute total reaction force
contacts = cube_link.contacts()     # Readout again the contact data
total_force = 0
for i in range(len(contacts[0].points)):
    total_force += contacts[0].points[i].force[2]

print("Total vertical reaction force at steady-state:", total_force, "N")
print("Cube mass:", cube_link.mass(), "Kg")

if GUI:
    time.sleep(5)
