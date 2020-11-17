import abc
import numpy as np
from typing import Tuple
from scipy.spatial.transform import Rotation


class Transform(abc.ABC):

    @staticmethod
    def from_position_and_quaternion(position: np.ndarray,
                                     quaternion: np.ndarray) -> np.ndarray:

        if quaternion.size != 4:
            raise ValueError("Quaternion array must have 4 elements")

        rotation = Quaternion.to_rotation(quaternion)
        transform = Transform.from_position_and_rotation(position=position,
                                                         rotation=rotation)

        return transform

    @staticmethod
    def from_position_and_rotation(position: np.ndarray,
                                   rotation: np.ndarray) -> np.ndarray:

        if position.size != 3:
            raise ValueError("Position array must have 3 elements")

        if rotation.shape != (3, 3):
            raise ValueError("Rotation must be a square 3x3 matrix")

        transform = np.eye(4)
        transform[0:3, 3] = position
        transform[0:3, 0:3] = rotation

        return transform

    @staticmethod
    def to_position_and_rotation(transform: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:

        if transform.shape != (4, 4):
            raise ValueError("Transform must be a 4x4 matrix")

        position = transform[0:3, 3]
        rotation = transform[0:3, 0:3]

        return position, rotation

    @staticmethod
    def to_position_and_quaternion(transform: np.ndarray) \
        -> Tuple[np.ndarray, np.ndarray]:

        p, R = Transform.to_position_and_rotation(transform=transform)
        return p, Quaternion.from_matrix(matrix=R)


class Quaternion(abc.ABC):

    @staticmethod
    def to_wxyz(xyzw: np.ndarray) -> np.ndarray:

        if xyzw.shape != (4,):
            raise ValueError(xyzw)

        return xyzw[[3, 0, 1, 2]]

    @staticmethod
    def to_xyzw(wxyz: np.ndarray) -> np.ndarray:

        if wxyz.shape != (4,):
            raise ValueError(wxyz)

        return wxyz[[1, 2, 3, 0]]

    @staticmethod
    def to_rotation(quaternion: np.ndarray) -> np.ndarray:

        if quaternion.shape != (4,):
            raise ValueError(quaternion)

        xyzw = Quaternion.to_xyzw(quaternion)

        return Rotation.from_quat(xyzw).as_matrix()

    @staticmethod
    def from_matrix(matrix: np.ndarray) -> np.ndarray:

        if matrix.shape != (3, 3):
            raise ValueError(matrix)

        quaternion_xyzw = Rotation.from_matrix(matrix).as_quat()
        quaternion_wxyz = Quaternion.to_wxyz(quaternion_xyzw)

        return quaternion_wxyz
