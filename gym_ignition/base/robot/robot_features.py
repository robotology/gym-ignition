# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
from typing import Type
from gym_ignition.base import robot


class RobotFeatures(robot.robot_abc.RobotABC,
                    robot.robot_baseframe.RobotBaseFrame,
                    robot.robot_joints.RobotJoints,
                    robot.robot_links.RobotLinks,
                    robot.robot_contacts.RobotContacts,
                    robot.robot_initialstate.RobotInitialState,
                    abc.ABC):
    pass


def feature_detector(cls_to_patch):
    """
    Feature class decorator for feature detection.

    Usage:

    ```
    @feature_detector
    class RobotFeatures(robot.robot_base.RobotBase, robot.robot_joints.RobotJoints):
       pass

    class MyRobot(robot.robot_base.RobotBase)
        [...]

    my_robot = MyRobot()

    if not RobotFeatures.has_all_feature(my_robot):
        raise Exception
    ```

    Args:
        cls_to_patch: The feature class

    Returns:
        The feature class patched with an additional `has_all_features` method.
    """
    # Define the function that will become a class method
    def has_all_features(cls: Type, obj: object) -> None:
        import inspect
        # Get the list of the wanted features to check.
        # Remove the first entry since it is the 'cls' class.
        wanted_features = inspect.getmro(cls)[1:]

        # Check that all the features are present
        for feature in wanted_features:
            if not isinstance(obj, feature):
                raise Exception("Missing feature: '{}'".format(feature))

    if not hasattr(cls_to_patch, 'has_all_features'):
        setattr(cls_to_patch, 'has_all_features', classmethod(has_all_features))

    return cls_to_patch
