import abc
import numpy as np
from gym_ignition import rbd
from typing import Tuple, Union
import idyntree.bindings as idt


class FromNumPy(abc.ABC):

    @staticmethod
    def to_idyntree_dyn_vector(array: np.ndarray) -> idt.VectorDynSize:

        return idt.VectorDynSize_FromPython(array)

    @staticmethod
    def to_idyntree_fixed_vector(array: np.ndarray):

        size = array.size
        supported_sizes = [3, 4, 6]

        if size not in supported_sizes:
            raise ValueError(array)

        if size == 3:
            idyntree_vector = idt.Vector3()
        elif size == 4:
            idyntree_vector = idt.Vector4()
        elif size == 6:
            idyntree_vector = idt.Vector6()
        else:
            raise RuntimeError

        return idyntree_vector.FromPython(array)

    @staticmethod
    def to_idyntree_position(position: np.ndarray) -> idt.Position:

        if position.size != 3:
            raise ValueError("The position array must have 3 elements")

        return idt.Position(position[0], position[1], position[2])

    @staticmethod
    def to_idyntree_rotation(quaternion: np.ndarray) -> idt.Rotation:

        if quaternion.size != 4:
            raise ValueError("The quaternion array must have 4 elements")

        quat = idt.Vector4_FromPython(quaternion)

        R = idt.Rotation()
        R.fromQuaternion(quat)

        return R

    @staticmethod
    def to_idyntree_transform(position: np.ndarray,
                              quaternion: np.ndarray = None,
                              rotation: np.ndarray = None) -> idt.Transform:

        if quaternion is None and rotation is None:
            raise ValueError("You must pass either a quaternion or a rotation")

        if quaternion is not None and rotation is not None:
            raise ValueError("You must pass either a quaternion or a rotation")

        if rotation is not None:
            quaternion = rbd.conversions.Quaternion.from_matrix(matrix=rotation)

        p = FromNumPy.to_idyntree_position(position=position)
        R = FromNumPy.to_idyntree_rotation(quaternion=quaternion)

        H = idt.Transform()
        H.setPosition(p)
        H.setRotation(R)

        return H

    @staticmethod
    def to_idyntree_twist(linear_velocity: np.ndarray,
                          angular_velocity: np.ndarray) -> idt.Twist:

        if linear_velocity.size != 3:
            raise ValueError("The linear velocity must have 3 elements")

        if angular_velocity.size != 3:
            raise ValueError("The angular velocity must have 3 elements")

        twist_numpy = np.concatenate((linear_velocity, angular_velocity))

        twist = idt.Twist_FromPython(twist_numpy)

        return twist


class ToNumPy(abc.ABC):

    @staticmethod
    def from_idyntree_vector(vector) -> np.ndarray:

        input_types = (
            idt.Vector3,
            idt.Vector4,
            idt.Vector6,
            idt.VectorDynSize,
            idt.Matrix3x3,
            idt.Matrix4x4,
            idt.Matrix6x6,
            idt.MatrixDynSize,
        )

        if not isinstance(vector, input_types):
            raise ValueError(vector)

        return np.array(vector.toNumPy())

    @staticmethod
    def from_idyntree_transform(transform: idt.Transform, split: bool = False) \
            -> Union[Tuple[np.ndarray, np.ndarray], np.ndarray]:

        if not isinstance(transform, idt.Transform):
            raise ValueError(transform)

        rotation = transform.getRotation()
        position = transform.getPosition()

        if split:
            return position.toNumPy(), rotation.toNumPy()

        else:
            H = np.zeros(shape=[4, 4])
            H[0:3, 3] = position.toNumPy()
            H[0:3, 0:3] = rotation.toNumPy()

            return H
