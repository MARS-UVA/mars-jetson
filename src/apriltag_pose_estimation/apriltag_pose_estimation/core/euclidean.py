from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import numpy.typing as npt
from scipy.linalg import expm, logm
from scipy.spatial.transform import Rotation


__all__ = ['Transform', 'Twist']


@dataclass(frozen=True)
class Transform:
    """
    An object representing a Euclidean transformation in 3D (an element of the Lie group E(3)).

    The group operation is assigned to the matrix multiplication operator ``@``.

    ``Transform`` objects may have an associated input space and output space, which are represented as strings. Before
    performing transformations, the output space of the first transformation is confirmed to match the input space of
    the second. This is designed to prevent erroneous operations. The checks are disabled in optimized mode.

    Client code should avoid calling the constructor directly since the class method constructors provide safer options.
    """
    matrix: npt.NDArray[np.float64]
    """
    A 4x4 matrix which represents the transformation.
    
    The 3x3 leading principal submatrix describes the rotation associated with this transformation. The 3x1 vector
    consisting of the first three rows of the last column describes the translation associated with the transformation.
    The bottom right entry is always 1, and the other entries in the last row are always 0.
    
    This representation was chosen because this is the matrix representation of the Lie group E(3).
    """

    input_space: Optional[str] = None
    """A string indicating the input space of this transformation."""
    output_space: Optional[str] = None
    """A string indicating the input space of this transformation."""

    error: Optional[float] = None
    """An error associated with the transformation, if any."""

    ambiguity: Optional[float] = None
    """
    An ambiguity associated with the transformation, if any.
    
    Ambiguity is calculated during localization as the ratio of reprojection errors from the best to second-best
    candidate transformations.
    """

    def __post_init__(self):
        if self.matrix.shape != (4, 4):
            raise ValueError('the transformation matrix must be 4x4')
        if __debug__:
            rotation_det = np.linalg.det(self.matrix[:-1, :-1])
            if not (np.isclose(rotation_det, 1) or np.isclose(rotation_det, -1)):
                raise ValueError('the rotation component of the matrix must have determinant +1 or -1')
            if not np.isclose(self.matrix[-1, :-1], 0).all():
                raise ValueError(f'the bottom row should be [0, 0, 0, 1] '
                                 f'(got [{", ".join(str(x) for x in self.matrix[-1])}]')

    @classmethod
    def identity(cls,
                 input_space: Optional[str] = None,
                 output_space: Optional[str] = None,
                 error: Optional[float] = None,
                 ambiguity: Optional[float] = None) -> 'Transform':
        """
        Creates a new ``Transform`` object that represents the identity transformation.
        :param input_space: The input space of the transformation (optional).
        :param output_space: The output space of the transformation (optional).
        :param error: The error associated with the transformation (optional).
        :param ambiguity: The ambiguity associated with the transformation (optional).
        :return: A new ``Transform`` object that represents the identity transformation.
        """
        return cls(matrix=np.eye(4),
                   input_space=input_space,
                   output_space=output_space,
                   error=error,
                   ambiguity=ambiguity)

    @classmethod
    def make(cls,
             rotation: Rotation,
             translation: npt.ArrayLike,
             input_space: Optional[str] = None,
             output_space: Optional[str] = None,
             error: Optional[float] = None,
             ambiguity: Optional[float] = None) -> 'Transform':
        """
        Creates a new ``Transform`` object.
        :param rotation: The rotation associated with the transformation.
        :param translation: The translation associated with the transformation, in order of [x, y, z].
        :param input_space: The input space of the transformation (optional).
        :param output_space: The output space of the transformation (optional).
        :param error: The error associated with the transformation (optional).
        :param ambiguity: The ambiguity associated with the transformation (optional).
        :return: A new ``Transform`` object.
        """
        return cls(matrix=np.block([[rotation.as_matrix(), np.array(translation, dtype=np.float64).reshape(-1, 1)],  # type: ignore
                                    [np.zeros(shape=(1, 3)), 1]]),
                   input_space=input_space,
                   output_space=output_space,
                   error=error,
                   ambiguity=ambiguity)

    @classmethod
    def from_opencv_vectors(cls,
                            rotation_vector: npt.NDArray[np.float64],
                            translation_vector: npt.NDArray[np.float64],
                            input_space: Optional[str] = None,
                            output_space: Optional[str] = None,
                            error: Optional[float] = None,
                            ambiguity: Optional[float] = None) -> 'Transform':
        """
        Returns a new Transform object constructed from a rotation vector and translation vector in the format used by
        OpenCV's solvePnP algorithm.
        :param rotation_vector: A vector representing the rotation component of the pose where the direction is the axis
                                of rotation and the norm is the magnitude of rotation in radians (shape must be either
                                3 or 3x1).
        :param translation_vector: A vector representing the translation component of the pose in order of
                                   [x, y, z] (shape must be either 3 or 3x1).

        :param input_space: The input space of the transformation (optional).
        :param output_space: The output space of the transformation (optional).
        :param error: The error associated with the transformation (optional).
        :param ambiguity: The ambiguity associated with the transformation (optional).
        :return: A Transform created from the given vectors.
        """
        return cls.make(rotation=Rotation.from_rotvec(rotation_vector.reshape(-1)),
                        translation=translation_vector.astype(np.float64),
                        input_space=input_space,
                        output_space=output_space,
                        error=error,
                        ambiguity=ambiguity)

    @classmethod
    def from_matrix(cls,
                    matrix: npt.NDArray[np.float64],
                    input_space: Optional[str] = None,
                    output_space: Optional[str] = None,
                    error: Optional[float] = None,
                    ambiguity: Optional[float] = None) -> 'Transform':
        """
        Returns a new Transform object constructed from the given matrix.

        The 3x3 leading principal submatrix describes the rotation associated with this transformation. The 3x1 vector
        consisting of the first three rows of the last column describes the translation associated with the
        transformation. The bottom right entry is always 1, and the other entries in the last row are always 0.

        :param matrix: The matrix which represents the transformation.
        :param input_space: The input space of the transformation (optional).
        :param output_space: The output space of the transformation (optional).
        :param error: The error associated with the transformation (optional).
        :param ambiguity: The ambiguity associated with the transformation (optional).
        :return: A new Transform object constructed from the given matrix.
        """
        return cls(matrix=matrix.astype(np.float64),
                   input_space=input_space,
                   output_space=output_space,
                   error=error,
                   ambiguity=ambiguity)

    def with_input_space(self, input_space: str) -> 'Transform':
        """Returns a new Transform object representing the same transformation but with the given input space."""
        return Transform(matrix=self.matrix,
                         input_space=input_space,
                         output_space=self.output_space,
                         error=self.error,
                         ambiguity=self.ambiguity)

    def with_output_space(self, output_space: str) -> 'Transform':
        """Returns a new Transform object representing the same transformation but with the given output space."""
        return Transform(matrix=self.matrix,
                         input_space=self.input_space,
                         output_space=output_space,
                         error=self.error,
                         ambiguity=self.ambiguity)

    def with_error(self, error: float) -> 'Transform':
        """Returns a new Transform object representing the same transformation but with the given error."""
        return Transform(matrix=self.matrix,
                         input_space=self.input_space,
                         output_space=self.output_space,
                         error=error,
                         ambiguity=self.ambiguity)

    def with_ambiguity(self, ambiguity: float) -> 'Transform':
        """Returns a new Transform object with the same position data but with the given ambiguity."""
        return Transform(matrix=self.matrix,
                         input_space=self.input_space,
                         output_space=self.output_space,
                         error=self.error,
                         ambiguity=ambiguity)

    @property
    def rotation(self) -> Rotation:
        """The rotation associated with the transformation."""
        return Rotation.from_matrix(self.matrix[:-1, :-1])

    @property
    def translation(self) -> npt.NDArray[np.float64]:
        """The translation associated with the transformation as a NumPy array of shape (3,)."""
        return self.matrix[:-1, -1]

    @property
    def opencv_rotation_vector(self) -> npt.NDArray[np.float64]:
        """
        A 3x1 matrix representing the rotation of the pose where the direction of the vector is the axis of rotation and
        the norm of the vector is the magnitude of rotation in radians.
        """
        result, _ = cv2.Rodrigues(self.matrix[:-1, :-1])
        return result

    @property
    def opencv_translation_vector(self) -> npt.NDArray[np.float64]:
        """
        A 3x1 matrix representing the translation associated with the transformation as a NumPy array of shape (3,).
        """
        return self.matrix[:-1, -1].reshape(-1, 1)

    def transform(self, points: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Transforms the given points using this transformation.
        :param points: A 3xN array of points.
        :return: The transformed points as a 3xN array.
        """
        if points.shape[0] != 3:
            raise ValueError(f'points must be 3 dimensional (got {points.shape[0]} dimensions)')
        homogeneous_points = np.ones(shape=(points.shape[0] + 1, points.shape[1]))
        homogeneous_points[:-1, :] = points
        return (self.matrix @ homogeneous_points)[:-1, :]

    def __matmul__(self, other: 'Transform') -> 'Transform':
        if not isinstance(other, Transform):
            return NotImplemented
        if __debug__:
            if (self.input_space is not None and other.output_space is not None
                    and other.output_space != self.input_space):
                raise ValueError(f'T2 @ T1: T1 output space ({other.output_space}) does not match '
                                 f'T2 input space ({self.input_space})')
        return Transform(matrix=self.matrix @ other.matrix,
                         input_space=other.input_space,
                         output_space=self.output_space)

    def inv(self) -> 'Transform':
        return Transform(matrix=np.linalg.inv(self.matrix),
                         input_space=self.output_space,
                         output_space=self.input_space,
                         error=self.error,
                         ambiguity=self.ambiguity)

    def log(self) -> 'Twist':
        """
        Returns the image of the logarithmic map applied to this transformation.

        This is the twist of a particle which moved from the origin of the input space to the pose represented by this
        transformation with constant angular and linear velocity for one second.

        :return: The image of the logarithmic map applied to this transformation.
        """
        return Twist.from_matrix(logm(self.matrix, disp=True))  # type: ignore


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
            raise ValueError(f'expected shape (3, 3) for angular velocity matrix, '
                             f'got {self.angular_velocity_matrix.shape}')
        if self.linear_velocity_vector.shape != (3, 1):
            raise ValueError(f'expected shape (3, 1) for linear velocity vector, '
                             f'got {self.linear_velocity_vector.shape}')

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
    def from_vectors(cls,
                     angular_velocity_vector: npt.NDArray[np.float64],
                     linear_velocity_vector: npt.NDArray[np.float64]):
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
        return np.array(
            [self.angular_velocity_matrix[2, 1], self.angular_velocity_matrix[0, 2], self.angular_velocity_matrix[1, 0]],
            dtype=np.float64).reshape(-1, 1)

    def get_matrix(self) -> npt.NDArray[np.float64]:
        """
        Returns a 4x4 matrix representation of the twist.

        The resulting matrix is an element of se(3).
        :return: The matrix representation of the twist.
        """
        return np.block([[self.angular_velocity_matrix, self.angular_velocity_vector],  # type: ignore
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

    def exp(self) -> 'Transform':
        """
        Returns the image of the exponential map applied to this twist.

        This is the pose of a particle which started at the origin and moved at constant angular and linear velocity
        given by this twist for one unit of time.

        :return: The image of the exponential map applied to this twist.
        """
        return Transform.from_matrix(expm(self.get_matrix()))
