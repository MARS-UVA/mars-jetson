from collections.abc import Collection
from typing import List

import cv2
import numpy as np
import numpy.typing as npt

from ...core.camera import CameraParameters
from ...core.detection import AprilTagDetection
from ...core.euclidean import Transform
from ...core.field import AprilTagField


class PoseEstimationStrategyTestCase:

    def __init__(self,
                 id_: int,
                 actual_camera_pose: Transform,
                 apriltag_field: AprilTagField,
                 detected_apriltags: Collection[int],
                 camera_params: CameraParameters):
        self.__id = id_
        self.__actual_camera_pose = actual_camera_pose
        self.__apriltag_field = apriltag_field
        self.__detected_apriltags = detected_apriltags
        self.__camera_params = camera_params

        self.__detections = self.__create_detections()

    @property
    def id(self) -> int:
        return self.__id

    @property
    def actual_camera_pose(self) -> Transform:
        return self.__actual_camera_pose

    @property
    def apriltag_field(self) -> AprilTagField:
        return self.__apriltag_field

    @property
    def detected_apriltags(self) -> Collection[int]:
        return self.__detected_apriltags

    @property
    def camera_params(self) -> CameraParameters:
        return self.__camera_params

    @property
    def detections(self) -> List[AprilTagDetection]:
        return self.__detections

    def __str__(self) -> str:
        return (f'{type(self).__name__}(id_={self.__id}, '
                f'actual_camera_pose={self.__actual_camera_pose}, '
                f'apriltag_field={self.__apriltag_field}, '
                f'detected_apriltags={self.__detected_apriltags}, '
                f'camera_params={self.camera_params})')

    def __repr__(self) -> str:
        return (f'{type(self).__name__}(id_={self.__id!r}, '
                f'actual_camera_pose={self.__actual_camera_pose!r}, '
                f'apriltag_field={self.__apriltag_field!r}, '
                f'detected_apriltags={self.__detected_apriltags!r}, '
                f'camera_params={self.camera_params!r})')

    def __create_detections(self):
        return [AprilTagDetection(tag_id=tag_id,
                                  tag_family=self.__apriltag_field.tag_family,
                                  center=_project_points_onto_camera(self.__apriltag_field[tag_id].opencv_translation_vector,
                                                                     camera_params=self.__camera_params,
                                                                     camera_in_origin=self.__actual_camera_pose)[0],
                                  corners=_project_points_onto_camera(self.__apriltag_field.get_corners(tag_id),
                                                                      camera_params=self.__camera_params,
                                                                      camera_in_origin=self.__actual_camera_pose),
                                  decision_margin=60,
                                  hamming=0,
                                  tag_poses=None)
                for tag_id in self.__detected_apriltags]


def _project_points_onto_camera(points: npt.NDArray[np.float64],
                                camera_params: CameraParameters,
                                camera_in_origin: Transform) -> npt.NDArray[np.float64]:
    origin_in_camera = camera_in_origin.inv()
    image_points, _ = cv2.projectPoints(points,
                                        origin_in_camera.opencv_rotation_vector,
                                        origin_in_camera.opencv_translation_vector,
                                        camera_params.get_matrix(),
                                        camera_params.get_distortion_vector())
    return image_points[:, 0, :]
