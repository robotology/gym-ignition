# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import numpy as np


def wedge(vector3: np.ndarray) -> np.ndarray:
    """
    Convert a 3D vector to a skew-symmetric matrix.

    Args:
        vector3: The 3D vector defining the matrix coefficients.

    Returns:
        The skew-symmetric matrix whose elements are created from the input vector.

    Note:
        The wedge operator can be useful to compute the cross product of 3D vectors:
        :math:`v_1 \\times v_2 = v_1^\\wedge v_2`.
    """

    if vector3.size != 3:
        raise ValueError(vector3)

    s = np.zeros(shape=(3, 3))
    s[1, 0] = vector3[2]
    s[0, 1] = -vector3[2]
    s[0, 2] = vector3[1]
    s[2, 0] = -vector3[1]
    s[2, 1] = vector3[0]
    s[1, 2] = -vector3[0]

    return s


def vee(matrix3x3: np.ndarray) -> np.ndarray:
    """
    Convert a 3x3 matrix to a 3D vector with the components of its skew-symmetric part.

    Args:
        matrix3x3: The input 3x3 matrix. If present, its symmetric part is removed.

    Returns:
        The 3D vector defining the skew-symmetric matrix.

    Note:
        This is the inverse operator of :py:func:`wedge`.
    """

    if matrix3x3.shape != (3, 3):
        raise ValueError(matrix3x3)

    skew_symmetric = extract_skew(matrix3x3)

    return np.array([skew_symmetric[2, 1], skew_symmetric[0, 2], skew_symmetric[1, 0]])


def extract_skew(matrix: np.ndarray) -> np.ndarray:
    """
    Extract the skew-symmetric part of a square matrix.

    Args:
        matrix: A square matrix.

    Returns:
        The skew-symmetric part of the input matrix.
    """

    if matrix.shape[0] != matrix.shape[1]:
        raise ValueError(matrix)

    return 0.5 * (matrix - matrix.T)


def extract_symm(matrix: np.ndarray) -> np.ndarray:
    """
    Extract the symmetric part of a square matrix.

    Args:
        matrix: A square matrix.

    Returns:
        The symmetric part of the input matrix.
    """

    if matrix.shape[0] != matrix.shape[1]:
        raise ValueError(matrix)

    return 0.5 * (matrix + matrix.T)
