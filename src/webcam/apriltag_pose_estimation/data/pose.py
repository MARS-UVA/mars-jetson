from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import numpy.typing as npt


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
    """An error associated with the camera_pose, if any."""

    def __post_init__(self):
        if self.rotation_matrix.shape != (3, 3):
            raise ValueError(f'expected shape (3, 3) for rotation matrix, got {self.rotation_matrix.shape}')
        if self.translation_vector.shape != (3, 1):
            raise ValueError(f'expected shape (3, 1) for translation vector, got {self.translation_vector.shape}')

    @classmethod
    def make(cls,
             roll: float = 0, pitch: float = 0, yaw: float = 0,
             tx: float = 0, ty: float = 0, tz: float = 0) -> 'Pose':
        """
        Returns a new Pose object with the given parameters.
        :param roll: The roll (rotation about the x-axis) in radians.
        :param pitch: The pitch (rotation about the y-axis) in radians.
        :param yaw: The yaw (rotation about the z-axis) in radians.
        :param tx: The x-component of the translation.
        :param ty: The y-component of the translation.
        :param tz: The z-component of the translation.
        :return: A new Pose object with the given parameters.
        """
        return cls.from_vectors(np.array([roll, pitch, yaw], dtype=np.float64),
                                np.array([tx, ty, tz], dtype=np.float64))

    @classmethod
    def from_vectors(cls, rotation_vector: npt.NDArray[np.float64], translation_vector: npt.NDArray[np.float64]):
        """
        Returns a new Pose from a rotation vector and translation vector.
        :param rotation_vector: A vector representing the rotation component of the transformation in Euler angles, in
                                order of [roll, pitch, yaw] in radians (shape must be either 3 or 3x1).
        :param translation_vector: A vector representing the translation component of the transformation in order of
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
        A vector (3x1 matrix) representing the rotation of the transformation in the order of [roll, pitch, yaw] in
        radians.
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
