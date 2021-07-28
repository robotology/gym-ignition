# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest

pytestmark = pytest.mark.gym_ignition

from typing import Tuple

import numpy as np
from gym_ignition.context.gazebo import controllers
from gym_ignition.rbd.idyntree import inverse_kinematics_nlp
from gym_ignition_environments import models

from scenario import gazebo as scenario_gazebo

from ..common.utils import default_world_fixture as default_world

# Set the verbosity
scenario_gazebo.set_verbosity(scenario_gazebo.Verbosity_debug)


@pytest.mark.parametrize("default_world", [(1.0 / 1_000, 1.0, 1)], indirect=True)
def test_inverse_kinematics(
    default_world: Tuple[scenario_gazebo.GazeboSimulator, scenario_gazebo.World]
):

    # Get the simulator and the world
    gazebo, world = default_world

    # Get the robot
    panda = models.panda.Panda(world=world)
    gazebo.run(paused=True)

    # Control all the joints
    controlled_joints = panda.joint_names()

    # Set the controller period
    assert panda.set_controller_period(gazebo.step_size())
    assert panda.controller_period() == gazebo.step_size()

    # Insert the ComputedTorqueFixedBase controller
    assert panda.to_gazebo().insert_model_plugin(
        *controllers.ComputedTorqueFixedBase(
            kp=[20.0] * panda.dofs(),
            ki=[0.0] * panda.dofs(),
            kd=[5.0] * panda.dofs(),
            urdf=panda.get_model_file(),
            joints=controlled_joints,
        ).args()
    )

    # Create the IK object
    ik = inverse_kinematics_nlp.InverseKinematicsNLP(
        urdf_filename=panda.get_model_file(),
        considered_joints=controlled_joints,
        joint_serialization=panda.joint_names(),
    )

    # Initialize IK
    ik.initialize(verbosity=1, floating_base=False, cost_tolerance=1e-8)

    # There should be no active targets
    assert len(ik.get_active_target_names()) == 0

    # Add the cartesian target of the end effector
    end_effector = "end_effector_frame"
    ik.add_target(
        frame_name=end_effector,
        target_type=inverse_kinematics_nlp.TargetType.POSE,
        as_constraint=False,
    )

    assert set(ik.get_active_target_names()) == {end_effector}
    assert (
        ik.get_target_data(target_name=end_effector).type
        == inverse_kinematics_nlp.TargetType.POSE
    )

    # Desired EE position
    target_ee_position = np.array([0.5, -0.3, 1.0])
    target_ee_quaternion = np.array([0.0, 1.0, 0.0, 0.0])

    # Update the target transform
    ik.update_transform_target(
        target_name=end_effector,
        position=target_ee_position,
        quaternion=target_ee_quaternion,
    )

    # Check that the target transform was stored
    assert isinstance(
        ik.get_target_data(target_name=end_effector).data,
        inverse_kinematics_nlp.TransformTargetData,
    )
    assert ik.get_target_data(target_name=end_effector).data.position == pytest.approx(
        target_ee_position
    )
    assert ik.get_target_data(
        target_name=end_effector
    ).data.quaternion == pytest.approx(target_ee_quaternion)

    # Set the current configuration
    ik.set_current_robot_configuration(
        base_position=np.array(panda.base_position()),
        base_quaternion=np.array(panda.base_orientation()),
        joint_configuration=np.array(panda.joint_positions()),
    )

    # Solve the IK problem
    ik.solve()

    # Get the full solution
    ik_solution = ik.get_full_solution()

    # Validate the solution
    assert ik_solution.joint_configuration.size == len(controlled_joints)
    assert ik_solution.base_position == pytest.approx(np.array([0.0, 0, 0]))
    assert ik_solution.base_quaternion == pytest.approx(np.array([1.0, 0, 0, 0]))

    # Set the desired joint configuration
    assert panda.set_joint_position_targets(
        ik_solution.joint_configuration, controlled_joints
    )
    assert panda.set_joint_velocity_targets([0.0] * len(controlled_joints))
    assert panda.set_joint_acceleration_targets([0.0] * len(controlled_joints))

    for _ in range(5_000):
        gazebo.run()

    # Check that the desired joint configuration has been reached
    assert panda.joint_positions(controlled_joints) == pytest.approx(
        ik_solution.joint_configuration, abs=np.deg2rad(0.1)
    )

    # Get the end effector pose
    ee_position = panda.get_link(link_name=end_effector).position()
    ee_quaternion = panda.get_link(link_name=end_effector).orientation()

    assert ee_position == pytest.approx(target_ee_position, abs=0.005)
    assert ee_quaternion == pytest.approx(target_ee_quaternion, abs=0.005)
