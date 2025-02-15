import json
from collections.abc import Mapping
from typing import Dict, List, Optional, TextIO

import numpy as np
import numpy.typing as npt
from scipy.spatial.transform import Rotation

from .euclidean import Transform


__all__ = ['AprilTagField', 'load_field']


class AprilTagField(Mapping[int, Transform]):
    """
    A class whose instances store information about the AprilTags in the region in which the robot is operating.

    This class implements the :class:`Mapping` protocol, where the keys are tag IDs and the values are their poses in
    the world frame.
    """

    def __init__(self, tag_size: float, tag_positions: dict[int, Transform], tag_family: str = 'tag36h11'):
        """
        :param tag_size: The size of the AprilTags on the field in meters.
        :param tag_positions: A dictionary from the IDs of tags on the field to their poses in the world frame.
        :param tag_family: The AprilTag family of which the tags on the field are a part.
        """
        self.__input_space = (next(iter(tag_positions.values())).input_space
                              if all(pos.input_space is not None for pos in tag_positions.values())
                              else None)
        self.__output_space = (next(iter(tag_positions.values())).output_space
                               if all(pos.output_space is not None for pos in tag_positions.values())
                               else None)
        if __debug__ and tag_positions:
            if self.__input_space is not None and any(position.input_space != self.__input_space
                                                      for position in tag_positions.values()):
                raise ValueError('tag position input space mismatch')
            if self.__output_space is not None and any(position.output_space != self.__output_space
                                                       for position in tag_positions.values()):
                raise ValueError('tag position input space mismatch')

        self.__tag_size = tag_size
        self.__tag_positions = tag_positions
        self.__tag_family = tag_family
        self.__corners: Dict[int, npt.NDArray[np.float64]] = {}

        self.__calculate_corners()

    @property
    def tag_size(self) -> float:
        """The size of the AprilTags on the field in meters."""
        return self.__tag_size

    @property
    def tag_family(self) -> str:
        """The AprilTag family of which the tags on the field are a part."""
        return self.__tag_family

    @property
    def input_space(self) -> Optional[str]:
        """The input space of each tag position, if specified."""
        return self.__input_space

    @property
    def output_space(self) -> Optional[str]:
        """The output space of each tag position, if specified."""
        return self.__output_space

    def __getitem__(self, __key: int):
        return self.__tag_positions[__key]

    def get_corners(self, *tag_ids: int) -> npt.NDArray[np.float64]:
        """
        Returns corner points of the AprilTags with the given IDs in the world frame.

        The corner points are returned in a 4nx3 array in the same order the IDs were given.
        :param tag_ids: The IDs of the AprilTags for which corner points will be retrieved.
        :return: A 4nx3 array containing the corner points of the AprilTags.
        """
        if not tag_ids:
            return np.zeros(shape=(0, 4))
        return np.vstack([corners for tag_id, corners in self.__corners.items() if tag_id in tag_ids])

    def __len__(self):
        return len(self.__tag_positions)

    def __iter__(self):
        return iter(self.__tag_positions)

    def get_tag_ids(self) -> List[int]:
        """Returns a list of IDs corresponding to the AprilTags on this field in no particular order."""
        return list(self.__tag_positions.keys())

    def __calculate_corners(self) -> None:
        corner_points = np.array([
            [-1, +1, 0],
            [+1, +1, 0],
            [+1, -1, 0],
            [-1, -1, 0],
        ]) / 2 * self.tag_size
        self.__corners = {tag_id: pose.transform(corner_points.T).T for tag_id, pose in self.__tag_positions.items()}


def load_field(fp: TextIO) -> AprilTagField:
    """
    Loads an AprilTag field from a JSON file.

    The JSON should have a key called ``fiducials`` with a value that is the list of all the tags on the field, a
    ``tag_size`` key with the tag size in meters, and the ``tag_family`` key with the AprilTag family as a string.

    Each tag is a JSON object with an ID as an integer, an axis-magnitude rotation vector, and a translation vector.

    Example JSON::

        {
          "fiducials": [
            {
              "id": 0,
              "rotation_vector": [
                -1.2091995761561456,
                1.2091995761561452,
                -1.2091995761561458
              ],
              "translation_vector": [
                0,
                0.105,
                0.56
              ]
            },
            {
              "id": 1,
              "rotation_vector": [
                -1.5707963267948968,
                0,
                0
              ],
              "translation_vector": [
                0.21,
                -0.9,
                0.82
              ]
            },
            {
              "id": 2,
              "rotation_vector": [
                0,
                1.5707963267948963,
                0
              ],
              "translation_vector": [
                -0.018,
                -0.445,
                0.34
              ]
            }
          ],
          "tag_size": 0.080,
          "tag_family": "tagStandard41h12"
        }

    :param fp: A text file pointer to the JSON file.
    :return: An AprilTagField instance created from the data in the JSON file. The input space of each pose is
             "tag_optical", and the output space is "world".
    """
    field_dict = json.load(fp)
    return AprilTagField(tag_size=field_dict['tag_size'],
                         tag_family=field_dict['tag_family'],
                         tag_positions={tag_data['id']: Transform.make(rotation=Rotation.from_rotvec(tag_data['rotation_vector']),
                                                                       translation=tag_data['translation_vector'],
                                                                       input_space='tag_optical',
                                                                       output_space='world')
                                        for tag_data in field_dict['fiducials']})
