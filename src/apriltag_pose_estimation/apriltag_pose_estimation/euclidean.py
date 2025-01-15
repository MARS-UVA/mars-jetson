from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import numpy.typing as npt
from scipy.linalg import expm, logm


__all__ = ['Pose', 'Twist']


@dataclass(frozen=True)
class Pose:
    """
    An object representing the pose of an object in 3D Euclidean space (an element of SE(3)).
    """
    rotation_matrix: npt.NDArray[np.float64]
    """A 3x3 float64 matrix which is the rotation component of the pose."""
    translation_vector: npt.NDArray[np.float64]
    """A float64 vector which is the translation component of the pose."""

    error: Optional[float] = None
    """An error associated with the pose, if any."""

    def __post_init__(self):
        if self.rotation_matrix.shape != (3, 3):
            raise ValueError(f'expected shape (3, 3) for rotation matrix, got {self.rotation_matrix.shape}')
        if self.translation_vector.shape != (3, 1):
            raise ValueError(f'expected shape (3, 1) for translation vector, got {self.translation_vector.shape}')

    @classmethod
    def from_vectors(cls, rotation_vector: npt.NDArray[np.float64], translation_vector: npt.NDArray[np.float64]):
        """
        Returns a new Pose object constructed from a rotation vector and translation vector.
        :param rotation_vector: A vector representing the rotation component of the pose in Euler angles, in order of
                                [roll, pitch, yaw] in radians (shape must be either 3 or 3x1).
        :param translation_vector: A vector representing the translation component of the pose in order of
                                   [x, y, z] (shape must be either 3 or 3x1).
        :return: A Pose created from the given vectors.
        """
        rotation_matrix, _ = cv2.Rodrigues(rotation_vector.astype(np.float64))
        return cls(rotation_matrix=rotation_matrix, translation_vector=translation_vector.astype(np.float64).reshape(-1, 1))

    @classmethod
    def from_matrix(cls, matrix: npt.NDArray[np.float64]):
        """
        Returns a new Pose object constructed from the given matrix representing an element of SE(3).
        :param matrix: The matrix, an element of SE(3).
        :return: A new Pose object constructed from the given matrix.
        """
        if matrix.shape != (4, 4):
            raise ValueError(f'matrix must be shape (4, 4), got {matrix.shape}')
        matrix = matrix.astype(np.float64)
        return cls(rotation_matrix=matrix[:3, :3], translation_vector=matrix[:3, 3].reshape(-1, 1))

    @property
    def rotation_vector(self) -> npt.NDArray[np.float64]:
        """
        A vector (3x1 matrix) representing the rotation of the pose in the order of [roll, pitch, yaw] in radians.
        """
        result, _ = cv2.Rodrigues(self.rotation_matrix)
        return result

    def get_matrix(self) -> npt.NDArray[np.float64]:
        """
        Returns a 4x4 homogenous matrix representation of the pose.

        The resulting matrix is an element of SE(3) and is a transformation from the frame of the posed object to the
        reference frame.
        :return: The matrix representation of the pose.
        """
        return np.block([[self.rotation_matrix, self.translation_vector],
                         [np.zeros((1, 3), dtype=np.float64), 1]])

    def log(self) -> 'Twist':
        """
        Returns the image of the logarithmic map applied to this pose.

        This is the twist of a particle which moved from the origin to this pose with constant angular and linear
        velocity for one seconds.

        :return: The image of the logarithmic map applied to this pose.
        """
        return Twist.from_matrix(logm(self.get_matrix()))


@dataclass(frozen=True)
class Twist:
    """
    An object representing the combined angular and linear velocity of an object in 3D Euclidean space (an element of
    se(3)).
    """
    angular_velocity_matrix: npt.NDArray[np.float64]
    """A 3x3 float64 antisymmetric matrix which is the angular velocity component of the twist."""
    linear_velocity_vector: npt.NDArray[np.float64]
    """A float64 vector which is the linear velocity component of the twist."""

    def __post_init__(self):
        if self.angular_velocity_matrix.shape != (3, 3):
            raise ValueError(f'expected shape (3, 3) for angular velocity matrix, got {self.angular_velocity_matrix.shape}')
        if self.linear_velocity_vector.shape != (3, 1):
            raise ValueError(f'expected shape (3, 1) for linear velocity vector, got {self.linear_velocity_vector.shape}')

    @classmethod
    def from_matrix(cls, matrix: npt.NDArray[np.float64]) -> 'Twist':
        """
        Returns a new Twist object constructed from the given matrix representing an element of se(3).
        :param matrix: The matrix, an element of se(3).
        :return: A new Twist object constructed from the given matrix.
        """
        matrix = matrix.astype(np.float64)
        return Twist(matrix[:3, :3], matrix[:3, 3].reshape(-1, 1))

    @classmethod
    def from_vectors(cls, angular_velocity_vector: npt.NDArray[np.float64], linear_velocity_vector: npt.NDArray[np.float64]):
        """
        Returns a new Twist object constructed from an angular velocity vector and a linear velocity vector.
        :param angular_velocity_vector: A vector representing the angular velocity component of the twist in Euler angle
                                        velocities, in order of [roll, pitch, yaw] in radians (shape must be either 3
                                        or 3x1).
        :param linear_velocity_vector: A vector representing the linear velocity component of the twist in order of
                                       [x, y, z] (shape must be either 3 or 3x1).
        :return: A Twist created from the given vectors.
        """
        angular_velocity_vector = angular_velocity_vector.reshape(-1)
        return Twist(angular_velocity_matrix=np.array([[0, -angular_velocity_vector[2], angular_velocity_vector[1]],
                                                       [angular_velocity_vector[2], 0, -angular_velocity_vector[0]],
                                                       [-angular_velocity_vector[1], angular_velocity_vector[0], 0]],
                                                      dtype=np.float64),
                     linear_velocity_vector=linear_velocity_vector.astype(np.float64).reshape(-1, 1))

    @property
    def angular_velocity_vector(self) -> npt.NDArray[np.float64]:
        """
        A 3x1 vector representing the angular velocity component of the twist in Euler angle velocities, in order of
        [roll, pitch, yaw] in radians.
        """
        return np.array([self.angular_velocity_matrix[2, 1], self.angular_velocity_matrix[0, 2], self.angular_velocity_matrix[1, 0]],
                        dtype=np.float64).reshape(-1, 1)

    def get_matrix(self) -> npt.NDArray[np.float64]:
        """
        Returns a 4x4 matrix representation of the twist.

        The resulting matrix is an element of se(3).
        :return: The matrix representation of the twist.
        """
        return np.block([[self.angular_velocity_matrix, self.angular_velocity_vector],
                         [np.zeros((1, 4), dtype=np.float64)]])

    def __add__(self, other: 'Twist') -> 'Twist':
        if not isinstance(other, Twist):
            return NotImplemented
        return Twist(angular_velocity_matrix=self.angular_velocity_matrix + other.angular_velocity_matrix,
                     linear_velocity_vector=self.linear_velocity_vector + other.linear_velocity_vector)

    def __sub__(self, other: 'Twist') -> 'Twist':
        if not isinstance(other, Twist):
            return NotImplemented
        return Twist(angular_velocity_matrix=self.angular_velocity_matrix - other.angular_velocity_matrix,
                     linear_velocity_vector=self.linear_velocity_vector - other.linear_velocity_vector)

    def __mul__(self, other: float) -> 'Twist':
        if not isinstance(other, int):
            return NotImplemented
        return Twist(angular_velocity_matrix=other * self.angular_velocity_matrix,
                     linear_velocity_vector=other * self.linear_velocity_vector)

    def __rmul__(self, other: float) -> 'Twist':
        if not isinstance(other, int):
            return NotImplemented
        return Twist(angular_velocity_matrix=other * self.angular_velocity_matrix,
                     linear_velocity_vector=other * self.linear_velocity_vector)

    def __truediv__(self, other: float) -> 'Twist':
        if not isinstance(other, int):
            return NotImplemented
        return Twist(angular_velocity_matrix=self.angular_velocity_matrix / other,
                     linear_velocity_vector=self.linear_velocity_vector / other)

    def exp(self) -> 'Pose':
        """
        Returns the image of the exponential map applied to this twist.

        This is the pose of a particle which started at the origin and moved at constant angular and linear velocity
        given by this twist for one unit of time.

        :return: The image of the exponential map applied to this twist.
        """
        return Pose.from_matrix(expm(self.get_matrix()))
