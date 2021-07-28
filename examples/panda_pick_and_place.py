import enum
import time
from functools import partial
from typing import List

import gym_ignition
import gym_ignition_environments
import numpy as np
from gym_ignition.context.gazebo import controllers
from gym_ignition.rbd import conversions
from gym_ignition.rbd.idyntree import inverse_kinematics_nlp
from scipy.spatial.transform import Rotation as R

from scenario import core as scenario_core
from scenario import gazebo as scenario_gazebo

# Configure verbosity
scenario_gazebo.set_verbosity(scenario_gazebo.Verbosity_error)

# Configure numpy output
np.set_printoptions(precision=4, suppress=True)


def add_panda_controller(
    panda: gym_ignition_environments.models.panda.Panda, controller_period: float
) -> None:

    # Set the controller period
    assert panda.set_controller_period(period=controller_period)

    # Increase the max effort of the fingers
    panda.get_joint(
        joint_name="panda_finger_joint1"
    ).to_gazebo().set_max_generalized_force(max_force=500.0)
    panda.get_joint(
        joint_name="panda_finger_joint2"
    ).to_gazebo().set_max_generalized_force(max_force=500.0)

    # Insert the ComputedTorqueFixedBase controller
    assert panda.to_gazebo().insert_model_plugin(
        *controllers.ComputedTorqueFixedBase(
            kp=[100.0] * (panda.dofs() - 2) + [10000.0] * 2,
            ki=[0.0] * panda.dofs(),
            kd=[17.5] * (panda.dofs() - 2) + [100.0] * 2,
            urdf=panda.get_model_file(),
            joints=panda.joint_names(),
        ).args()
    )

    # Initialize the controller to the current state
    assert panda.set_joint_position_targets(panda.joint_positions())
    assert panda.set_joint_velocity_targets(panda.joint_velocities())
    assert panda.set_joint_acceleration_targets(panda.joint_accelerations())


def get_panda_ik(
    panda: gym_ignition_environments.models.panda.Panda, optimized_joints: List[str]
) -> inverse_kinematics_nlp.InverseKinematicsNLP:

    # Create IK
    ik = inverse_kinematics_nlp.InverseKinematicsNLP(
        urdf_filename=panda.get_model_file(),
        considered_joints=optimized_joints,
        joint_serialization=panda.joint_names(),
    )

    # Initialize IK
    ik.initialize(
        verbosity=1,
        floating_base=False,
        cost_tolerance=1e-8,
        constraints_tolerance=1e-8,
        base_frame=panda.base_frame(),
    )

    # Set the current configuration
    ik.set_current_robot_configuration(
        base_position=np.array(panda.base_position()),
        base_quaternion=np.array(panda.base_orientation()),
        joint_configuration=np.array(panda.joint_positions()),
    )

    # Add the cartesian target of the end effector
    end_effector = "end_effector_frame"
    ik.add_target(
        frame_name=end_effector,
        target_type=inverse_kinematics_nlp.TargetType.POSE,
        as_constraint=False,
    )

    return ik


def insert_bucket(world: scenario_gazebo.World) -> scenario_gazebo.Model:

    # Insert objects from Fuel
    uri = lambda org, name: f"https://fuel.ignitionrobotics.org/{org}/models/{name}"

    # Download the cube SDF file
    bucket_sdf = scenario_gazebo.get_model_file_from_fuel(
        uri=uri(
            org="GoogleResearch",
            name="Threshold_Basket_Natural_Finish_Fabric_Liner_Small",
        ),
        use_cache=False,
    )

    # Assign a custom name to the model
    model_name = "bucket"

    # Insert the model
    assert world.insert_model(
        bucket_sdf, scenario_core.Pose([0.68, 0, 1.02], [1.0, 0, 0, 1]), model_name
    )

    # Return the model
    return world.get_model(model_name=model_name)


def insert_table(world: scenario_gazebo.World) -> scenario_gazebo.Model:

    # Insert objects from Fuel
    uri = lambda org, name: f"https://fuel.ignitionrobotics.org/{org}/models/{name}"

    # Download the cube SDF file
    bucket_sdf = scenario_gazebo.get_model_file_from_fuel(
        uri=uri(org="OpenRobotics", name="Table"), use_cache=False
    )

    # Assign a custom name to the model
    model_name = "table"

    # Insert the model
    assert world.insert_model(bucket_sdf, scenario_core.Pose_identity(), model_name)

    # Return the model
    return world.get_model(model_name=model_name)


def insert_cube_in_operating_area(
    world: scenario_gazebo.World,
) -> scenario_gazebo.Model:

    # Insert objects from Fuel
    uri = lambda org, name: f"https://fuel.ignitionrobotics.org/{org}/models/{name}"

    # Download the cube SDF file
    cube_sdf = scenario_gazebo.get_model_file_from_fuel(
        uri=uri(org="openrobotics", name="wood cube 5cm"), use_cache=False
    )

    # Sample a random position
    random_position = np.random.uniform(low=[0.2, -0.3, 1.01], high=[0.4, 0.3, 1.01])

    # Get a unique name
    model_name = gym_ignition.utils.scenario.get_unique_model_name(
        world=world, model_name="cube"
    )

    # Insert the model
    assert world.insert_model(
        cube_sdf, scenario_core.Pose(random_position, [1.0, 0, 0, 0]), model_name
    )

    # Return the model
    return world.get_model(model_name=model_name)


def solve_ik(
    target_position: np.ndarray,
    target_orientation: np.ndarray,
    ik: inverse_kinematics_nlp.InverseKinematicsNLP,
) -> np.ndarray:

    quat_xyzw = R.from_euler(seq="y", angles=90, degrees=True).as_quat()

    ik.update_transform_target(
        target_name=ik.get_active_target_names()[0],
        position=target_position,
        quaternion=conversions.Quaternion.to_wxyz(xyzw=quat_xyzw),
    )

    # Run the IK
    ik.solve()

    return ik.get_reduced_solution().joint_configuration


def end_effector_reached(
    position: np.array,
    end_effector_link: scenario_core.Link,
    max_error_pos: float = 0.01,
    max_error_vel: float = 0.5,
    mask: np.ndarray = np.array([1.0, 1.0, 1.0]),
) -> bool:

    masked_target = mask * position
    masked_current = mask * np.array(end_effector_link.position())

    return (
        np.linalg.norm(masked_current - masked_target) < max_error_pos
        and np.linalg.norm(end_effector_link.world_linear_velocity()) < max_error_vel
    )


def get_unload_position(bucket: scenario_core.Model) -> np.ndarray:

    return bucket.base_position() + np.array([0, 0, 0.3])


class FingersAction(enum.Enum):

    OPEN = enum.auto()
    CLOSE = enum.auto()


def move_fingers(
    panda: gym_ignition_environments.models.panda.Panda, action: FingersAction
) -> None:

    # Get the joints of the fingers
    finger1 = panda.get_joint(joint_name="panda_finger_joint1")
    finger2 = panda.get_joint(joint_name="panda_finger_joint2")

    if action is FingersAction.OPEN:
        finger1.set_position_target(position=finger1.position_limit().max)
        finger2.set_position_target(position=finger2.position_limit().max)

    if action is FingersAction.CLOSE:
        finger1.set_position_target(position=finger1.position_limit().min)
        finger2.set_position_target(position=finger2.position_limit().min)


# ====================
# INITIALIZE THE WORLD
# ====================

# Get the simulator and the world
gazebo, world = gym_ignition.utils.scenario.init_gazebo_sim(
    step_size=0.001, real_time_factor=2.0, steps_per_run=1
)

# Open the GUI
gazebo.gui()
time.sleep(3)
gazebo.run(paused=True)

# Insert the Panda manipulator
panda = gym_ignition_environments.models.panda.Panda(
    world=world, position=[-0.1, 0, 1.0]
)

# Enable contacts only for the finger links
panda.get_link("panda_leftfinger").to_gazebo().enable_contact_detection(True)
panda.get_link("panda_rightfinger").to_gazebo().enable_contact_detection(True)

# Process model insertion in the simulation
gazebo.run(paused=True)

# Monkey patch the class with finger helpers
panda.open_fingers = partial(move_fingers, panda=panda, action=FingersAction.OPEN)
panda.close_fingers = partial(move_fingers, panda=panda, action=FingersAction.CLOSE)

# Add a custom joint controller to the panda
add_panda_controller(panda=panda, controller_period=gazebo.step_size())

# Populate the world
table = insert_table(world=world)
bucket = insert_bucket(world=world)
gazebo.run(paused=True)

# Create and configure IK for the panda
ik_joints = [
    j.name() for j in panda.joints() if j.type is not scenario_core.JointType_fixed
]
ik = get_panda_ik(panda=panda, optimized_joints=ik_joints)

# Get some manipulator links
finger_left = panda.get_link(link_name="panda_leftfinger")
finger_right = panda.get_link(link_name="panda_rightfinger")
end_effector_frame = panda.get_link(link_name="end_effector_frame")

while True:

    # Insert a new cube
    cube = insert_cube_in_operating_area(world=world)
    gazebo.run(paused=True)

    # =========================
    # PHASE 1: Go over the cube
    # =========================

    print("Hovering")

    # Position over the cube
    position_over_cube = np.array(cube.base_position()) + np.array([0, 0, 0.4])

    # Get the joint configuration that brings the EE over the cube
    over_joint_configuration = solve_ik(
        target_position=position_over_cube,
        target_orientation=np.array(cube.base_orientation()),
        ik=ik,
    )

    # Set the joint references
    assert panda.set_joint_position_targets(over_joint_configuration, ik_joints)

    # Open the fingers
    panda.open_fingers()

    # Run the simulation until the EE reached the desired position
    while not end_effector_reached(
        position=position_over_cube,
        end_effector_link=end_effector_frame,
        max_error_pos=0.05,
        max_error_vel=0.5,
    ):
        gazebo.run()

    # Wait a bit more
    [gazebo.run() for _ in range(500)]

    # =======================
    # PHASE 2: Reach the cube
    # =======================

    print("Reaching")

    # Get the joint configuration that brings the EE to the cube
    over_joint_configuration = solve_ik(
        target_position=np.array(cube.base_position()) + np.array([0, 0, 0.04]),
        target_orientation=np.array(cube.base_orientation()),
        ik=ik,
    )

    # Set the joint references
    assert panda.set_joint_position_targets(over_joint_configuration, ik_joints)
    panda.open_fingers()

    # Run the simulation until the EE reached the desired position
    while not end_effector_reached(
        position=np.array(cube.base_position()) + np.array([0, 0, 0.04]),
        end_effector_link=end_effector_frame,
    ):

        gazebo.run()

    # Wait a bit more
    [gazebo.run() for _ in range(500)]

    # =======================
    # PHASE 3: Grasp the cube
    # =======================

    print("Grasping")

    # Close the fingers
    panda.close_fingers()

    # Detect a graps reading the contact wrenches of the finger links
    while not (
        np.linalg.norm(finger_left.contact_wrench()) >= 50.0
        and np.linalg.norm(finger_right.contact_wrench()) >= 50.0
    ):
        gazebo.run()

    # =============
    # PHASE 4: Lift
    # =============

    print("Lifting")

    # Position over the cube
    position_over_cube = np.array(cube.base_position()) + np.array([0, 0, 0.4])

    # Get the joint configuration that brings the EE over the cube
    over_joint_configuration = solve_ik(
        target_position=position_over_cube,
        target_orientation=np.array(cube.base_orientation()),
        ik=ik,
    )

    # Set the joint references
    assert panda.set_joint_position_targets(over_joint_configuration, ik_joints)

    # Run the simulation until the EE reached the desired position
    while not end_effector_reached(
        position=position_over_cube,
        end_effector_link=end_effector_frame,
        max_error_pos=0.1,
        max_error_vel=0.5,
    ):
        gazebo.run()

    # Wait a bit more
    [gazebo.run() for _ in range(500)]

    # =====================================
    # PHASE 5: Place the cube in the bucket
    # =====================================

    print("Dropping")

    # Get the joint configuration that brings the EE over the bucket
    unload_joint_configuration = solve_ik(
        target_position=get_unload_position(bucket=bucket),
        target_orientation=np.array([0, 1.0, 0, 0]),
        ik=ik,
    )

    # Set the joint references
    assert panda.set_joint_position_targets(unload_joint_configuration, ik_joints)

    # Run the simulation until the EE reached the desired position
    while not end_effector_reached(
        position=get_unload_position(bucket=bucket)
        + np.random.uniform(low=-0.05, high=0.05, size=3),
        end_effector_link=end_effector_frame,
        max_error_pos=0.01,
        max_error_vel=0.1,
        mask=np.array([1, 1, 0]),
    ):

        gazebo.run()

    # Open the fingers
    panda.open_fingers()

    # Wait that both fingers are in not contact (with the cube)
    while finger_left.in_contact() or finger_right.in_contact():
        gazebo.run()

    # Wait a bit more
    [gazebo.run() for _ in range(500)]

    # Remove the cube
    world.remove_model(model_name=cube.name())

# It is always a good practice to close the simulator.
# In this case it is not required since above there is an infinite loop.
# gazebo.close()
