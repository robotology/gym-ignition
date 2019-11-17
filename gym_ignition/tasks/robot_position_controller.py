# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
import gym
import time
import gym.logger as logger
from typing import NamedTuple
# import gympp_bindings as bindings
from gym_ignition.base import task
from gym_ignition.utils.typing import *
from gym_ignition.base.robot import feature_detector
from gym_ignition.base.robot import robot_abc, robot_joints, robot_initialstate
from gym_ignition.base.controllers import PositionController, PositionControllerReferences


class Config(NamedTuple):
    position: bool = True        # (1 x DoF) array
    velocity: bool = False       # (1 x DoF) array
    acceleration: bool = False   # (1 x DoF) array
    base_pose: bool = False      # (1 x 7) array: position and orientation quaternion
    base_velocity: bool = False  # (1 x 6) array: linear and angular velocity
    floating: bool = False       # Enable floating or fixed base controller


# Initialize the robot features required by the task
@feature_detector
class RobotFeatures(robot_abc.RobotABC,
                    robot_joints.RobotJoints,
                    robot_initialstate.RobotInitialState,
                    abc.ABC):
    pass


class RobotPositionController(task.Task, abc.ABC):
    """
    Environment that exposes a position-controlled robot with a Gym interface.

    The environment is configured with the ControlledQuantities named tuple.  #  TODO
    Depending on its configuration, the size of action and observation vectors change.

    The configuration of the action and observation spaces depends on the

    For fixed base robots, the action is an array with the joint position references.
    For floating base robots, the action is expanded to have in its front the
    """
    def __init__(self,
                 config: Config,
                 controller_cls: type,
                 controller_kwargs: Dict,
                 floating_base: bool = False,
                 controlled_joints: List[str] = None,
                 initial_joint_positions: np.ndarray = None,
                 **kwargs) -> None:

        logger.debug("Initializing parent Task class")
        super().__init__()

        # Store the requested robot features for this task
        self.robot_features = RobotFeatures

        # Attributes
        self._config = config  # TODO
        self._controller = None
        self._controller_cls = controller_cls
        self._controller_kwargs = controller_kwargs
        self._floating_base = floating_base
        self._controlled_joints = controlled_joints
        self._initial_joint_positions = initial_joint_positions

    def _array_to_namedtuple(self, array: np.ndarray) -> PositionControllerReferences:
        dofs = self.controller.nr_controlled_dofs
        start = 0
        end = dofs
        references = PositionControllerReferences(position=array[start:end])
        start = end

        if self._config.velocity:
            end = start + dofs
            references = references._replace(velocity=array[start:end])
            start = end
        else:
            references = references._replace(velocity=np.zeros_like(references.position))

        if self._config.acceleration:
            end = start + dofs
            references = references._replace(velocity=array[start:end])
            start = end
        else:
            references = references._replace(
                acceleration=np.zeros_like(references.position))

        if self._config.base_pose:
            end = start + 7
            references = references._replace(base_position=array[start:start+3],
                                             base_orientation=array[start+3:end])
            start = end

        if self._config.base_velocity:
            end = start + 6
            references = references._replace(base_position=array[start:start+3],
                                             base_orientation=array[start+3:end])

        assert references.valid(), "References are not valid"
        return references

    @property
    def controller(self) -> PositionController:
        if self._controller is not None:
            return self._controller

        logger.debug("Creating the controller object")

        def pop_kwargs(kw: str, kwargs: dict, default=None, required: bool = False):
            if kw in kwargs:
                value = kwargs[kw]
                kwargs.pop(kw)
                return value
            else:
                if required:
                    raise Exception(f"Failed to find kwarg {kw}")

                return default

        dt = pop_kwargs("dt", self._controller_kwargs)
        robot = pop_kwargs("robot", self._controller_kwargs, self.robot)
        urdf = pop_kwargs("urdf", self._controller_kwargs, self.robot.model_file)

        # Create the controller
        self._controller = self._controller_cls(dt=dt,
                                                robot=robot,
                                                urdf=urdf,
                                                controlled_joints=self._controlled_joints,
                                                **self._controller_kwargs)

        logger.debug("Controller object created")
        return self._controller

    def create_spaces(self) -> Tuple[ActionSpace, ObservationSpace]:
        if not self.robot:
            raise Exception("The robot class is not yet ready")

        # Get joints limits
        joints_limit_min = []
        joints_limit_max = []

        if not self._controlled_joints:
            self._controlled_joints = self.robot.joint_names()

        for joint_name in self._controlled_joints:
            q_min, q_max = self.robot.joint_position_limits(joint_name)
            joints_limit_min.append(q_min)
            joints_limit_max.append(q_max)

        # TODO: increase slightly the limits?
        joints_limit_min = np.array(joints_limit_min)
        joints_limit_max = np.array(joints_limit_max)

        # Create the observation space
        observation_space = gym.spaces.Box(low=joints_limit_min,
                                           high=joints_limit_max,
                                           dtype=np.float32)

        # Create the action space
        action_space = gym.spaces.Box(low=np.array(joints_limit_min),
                                      high=np.array(joints_limit_max),
                                      dtype=np.float32)

        return action_space, observation_space

    def set_action(self, action: Action) -> bool:
        assert action.size == self.controller.nr_controlled_dofs, \
            "%r (%s) invalid" % (action, type(action))

        # TODO
        # assert self.action_space.contains(action), \
        #     "%r (%s) invalid" % (action, type(action))

        # Check the controller was initialized
        assert self.controller is not None, "Joint controller was not initialized"

        # Check that the action is compatible with the robot
        # assert action.size == robot.dofs(),
        #     "Action size does not match the number or "robot joints"

        # Get the joint torques from the controller
        # now = time.time()

        now = time.time()
        refs = self._array_to_namedtuple(action)
        print(f"arr2nt: {time.time() - now}")

        ok_references = self.controller.set_control_references(references=refs)
        assert ok_references  # TODO

        now = time.time()
        torques = self.controller.step()
        print(f"contr step: {time.time() - now}")

        # CPP
        # pos_refs = bindings.PositionReferences(self.controller.nr_controlled_dofs)
        # pos_refs.position = refs.position.tolist()
        # ok_references2 = self._controllercpp.setReferences(pos_refs)
        # assert ok_references2  # TODO
        # now = time.time()
        # _ = self.controller.step()
        # print(f"contr cpp step: {time.time() - now}")

        # print("=== Controller = {}".format(time.time() - now))
        ok_torques = True

        # Get the robot object
        robot = self.robot

        # Apply the joint torques
        for idx, joint_name in enumerate(self.controller.controlled_joints):
            ok_torques = ok_torques and robot.set_joint_force(joint_name, torques[idx])

        assert ok_torques, "Failed to set torques"

        return ok_torques

    def get_observation(self) -> Observation:
        # Create the observation object
        observation = Observation(self.robot.joint_positions())

        # Return the observation
        return observation

    def get_reward(self) -> Reward:
        # Initialize the reward
        reward = 0.0
        return Reward(reward)

    def is_done(self) -> bool:
        return False

    def reset_task(self) -> bool:
        # ====================
        # INITIALIZE THE ROBOT
        # ====================

        # Switch to fixed base if the robot is floating base
        if self.robot.is_floating_base() and not self._floating_base:
            ok_floating = self.robot.set_as_floating_base(False)
            assert ok_floating, "Failed to set the robot as fixed base"

            pos, orient = self.robot.base_pose()
            ok_base = self.robot.reset_base_pose(position=pos, orientation=orient)
            assert ok_base, "Failed to reset the base pose"

        # Switch to floating base if the robot is fixed base
        if not self.robot.is_floating_base() and self._floating_base:
            ok_floating = self.robot.set_as_floating_base(True)
            assert ok_floating, "Failed to set the robot as floating base"

            pos, orient = self.robot.base_pose()
            ok_base = self.robot.reset_base_pose(position=pos, orientation=orient)
            assert ok_base, "Failed to reset the base pose"

        # =====================
        # INITIALIZE CONTROLLER
        # =====================

        if self._controller is not None:
            ok_terminate = self.controller.terminate()
            self._controller = None
            assert ok_terminate, "Failed to terminate the controller"

        # Initialize the controller
        ok_initialize = self.controller.initialize()
        assert ok_initialize, "Failed to initialize the controller"

        # ======================
        # INITIALIZE ROBOT STATE
        # ======================

        # TODO: Allow initializing also the velocity and base?

        # Set initial position to 0 as default
        # if self.initial_joint_positions is None:
        #     self.initial_joint_positions = self.robot.initial_joint_positions()

        if self._initial_joint_positions is not None:
            for idx, name in enumerate(self.robot.joint_names()):
                self.robot.set_joint_position(name, self._initial_joint_positions[idx])

        # Validate the initial joint positions
        # assert self.initial_joint_positions.size == robot.dofs(), \
        #     "The size of the initial position array does not match the robot's DoFs"

        # Get joint names
        # joint_names = robot.joint_names()

        # # Reset the joints
        # for idx, pos in enumerate(self.initial_joint_positions):
        #     # TODO: velocity
        #     robot.reset_joint(joint_names[idx], pos, velocity=0.0)

        # # Store the initial base position and orientation
        # # TODO: this works only on simulated robots. This task though should work also
        # #  on real-time robots.
        # if self._initial_base_position is None or self._initial_base_orientation is None:
        #     # Store the base pose
        #     self._initial_base_position, self._initial_base_orientation = \
        #         robot.base_pose()
        #
        #     # In the fixed-base case, lift the robot
        #     if not self._floating_base:
        #         self._initial_base_position *= 1.2
        #
        # # Set the base pose
        # base_ok = robot.reset_base_pose(position=self._initial_base_position,
        #                                 orientation=self._initial_base_orientation,
        #                                 floating=self._floating_base)
        # assert base_ok, "Failed to lift the base of the robot"

        return True
