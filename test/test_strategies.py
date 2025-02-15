import json
from collections.abc import Collection
from importlib.resources import files
from itertools import product
from typing import List

import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from apriltag_pose_estimation.apriltag.render import OverlayWriter, WHITE
from apriltag_pose_estimation.core import AprilTagField, CameraParameters, Transform, PnPMethod
from apriltag_pose_estimation.localization import PoseEstimationStrategy
from apriltag_pose_estimation.localization.strategies import (MultiTagPnPEstimationStrategy,
                                                              MultiTagSpecialEstimationStrategy,
                                                              LowestAmbiguityEstimationStrategy)
from apriltag_pose_estimation.localization.testcase import PoseEstimationStrategyTestCase

import test.resources


DEPSTECH_CAM_PARAMETERS = CameraParameters(fx=1329.143348,
                                           fy=1326.537785,
                                           cx=945.392392,
                                           cy=521.144703,
                                           k1=-0.348650,
                                           k2=0.098710,
                                           p1=-0.000157,
                                           p2=-0.001851,
                                           k3=0.000000)


LOGITECH_CAM_PARAMETERS = CameraParameters(fx=1394.6027293299926,
                                           fy=1394.6027293299926,
                                           cx=995.588675691456,
                                           cy=599.3212928484164,
                                           k1=0.11480806073904032,
                                           k2=-0.21946985653851792,
                                           p1=0.0012002116999769957,
                                           p2=0.008564577708855225,
                                           k3=0.11274677130853494)
"""Camera parameters for a Logitech C920 webcam."""


# Note: the AprilTag field was taken from Limelight's model of the FRC 2024 game field.
# https://downloads.limelightvision.io/models/frc2024.fmap
def get_apriltag_field() -> AprilTagField:
    with files(test.resources).joinpath('frc2024.json').open(mode='r') as fp:
        field_data = json.load(fp)
    axis_change = Transform.from_matrix(np.array([[0, -1, 0, 0],
                                                  [0, 0, -1, 0],
                                                  [1, 0, 0, 0],
                                                  [0, 0, 0, 1]]).T,
                                        input_space='tag_optical',
                                        output_space='tag_limelight')
    return AprilTagField(tag_size=0.1651,
                         tag_positions={fiducial['id']: Transform.from_matrix(np.array(fiducial['transform'])
                                                                              .reshape((4, 4)),
                                                                              input_space='tag_limelight',
                                                                              output_space='world') @ axis_change
                                        for fiducial in field_data['fiducials']},
                         tag_family='tag36h11')


def robot_to_camera_pose(tx: float, ty: float, rot: float, camera_on_robot: Transform) -> Transform:
    axis_change = Transform.from_matrix(np.array([[0, 0, 1, 0],
                                                  [-1, 0, 0, 0],
                                                  [0, -1, 0, 0],
                                                  [0, 0, 0, 1]]),
                                        input_space='camera_optical',
                                        output_space='camera_standard')
    robot_in_origin = Transform.make(rotation=Rotation.from_euler('xyz', angles=[0, 0, rot], degrees=True),
                                     translation=[tx, ty, 0],
                                     input_space='robot_standard',
                                     output_space='world')
    return robot_in_origin @ camera_on_robot @ axis_change


def get_cases() -> List[PoseEstimationStrategyTestCase]:
    case_id = 0

    def case(actual_camera_pose: Transform, detected_apriltags: Collection[int]) -> PoseEstimationStrategyTestCase:
        nonlocal case_id
        case = PoseEstimationStrategyTestCase(id_=case_id,
                                              actual_camera_pose=actual_camera_pose,
                                              apriltag_field=field,
                                              detected_apriltags=detected_apriltags,
                                              camera_params=DEPSTECH_CAM_PARAMETERS)
        image = np.zeros(shape=(1080, 1920, 3), dtype=np.uint8)
        overlay_writer = OverlayWriter(image, detections=case.detections, camera_params=case.camera_params,
                                       tag_size=case.apriltag_field.tag_size)
        overlay_writer.overlay_square(color=WHITE, show_corners=True)
        overlay_writer.overlay_label(color=WHITE)
        cv2.imwrite(f'case{case_id}.png', image)
        case_id += 1
        return case

    def special_case(actual_camera_pose: Transform,
                     apriltag_field: AprilTagField,
                     detected_apriltags: Collection[int],
                     camera_params: CameraParameters) -> PoseEstimationStrategyTestCase:
        nonlocal case_id
        case = PoseEstimationStrategyTestCase(id_=case_id,
                                              actual_camera_pose=actual_camera_pose,
                                              apriltag_field=apriltag_field,
                                              detected_apriltags=detected_apriltags,
                                              camera_params=camera_params)
        image = np.zeros(shape=(1080, 1920, 3), dtype=np.uint8)
        overlay_writer = OverlayWriter(image, detections=case.detections, camera_params=case.camera_params,
                                       tag_size=case.apriltag_field.tag_size)
        overlay_writer.overlay_square(show_corners=True)
        overlay_writer.overlay_label(color=WHITE)
        cv2.imwrite(f'case{case_id}.png', image)
        case_id += 1
        return case

    def camera_pose(tx: float, ty: float, rot: float):
        return robot_to_camera_pose(tx, ty, rot, camera_on_robot)

    field = get_apriltag_field()
    camera_on_robot = Transform.make(rotation=Rotation.from_rotvec([0, -30, 0], degrees=True),
                                     translation=[0.304, -0.127, 0.237],
                                     input_space='camera_standard',
                                     output_space='robot_standard')
    axis_change = Transform.from_matrix(np.array([[0, 0, 1, 0],
                                                  [-1, 0, 0, 0],
                                                  [0, -1, 0, 0],
                                                  [0, 0, 0, 1]]),
                                        input_space='world_optical',
                                        output_space='world')
    return [
        case(actual_camera_pose=camera_pose(3.5, 0, 5),
             detected_apriltags=[3, 4]),
        case(actual_camera_pose=camera_pose(6.5, 2, 0),
             detected_apriltags=[3, 4]),
        case(actual_camera_pose=camera_pose(6.5, 2.5, 90),
             detected_apriltags=[5]),
        case(actual_camera_pose=camera_pose(6.5, -2, -60),
             detected_apriltags=[1, 2]),
        special_case(actual_camera_pose=Transform.make(rotation=Rotation.from_rotvec([-90, 0, 0], degrees=True)
                                                                * Rotation.from_rotvec([0, -90, 0], degrees=True),
                                                       translation=[2.5, 0.8, -0.3],
                                                       input_space='camera_optical',
                                                       output_space='world_optical'),
                     apriltag_field=AprilTagField(tag_size=0.080,
                                                  tag_family='tagStandard41h12',
                                                  tag_positions={
                                                      0: axis_change @ Transform.identity(input_space='tag_optical',
                                                                                          output_space='world_optical'),
                                                      2: axis_change @ Transform.make(rotation=Rotation.identity(),
                                                                                      translation=[-0.835, 0, 0],
                                                                                      input_space='tag_optical',
                                                                                      output_space='world_optical'),
                                                   }),
                     detected_apriltags=[0, 2],
                     camera_params=LOGITECH_CAM_PARAMETERS)
    ]


cases = get_cases()


def strategy_tester(strategy: PoseEstimationStrategy, case: PoseEstimationStrategyTestCase) -> None:
    pose = strategy.estimate_pose(detections=case.detections, field=case.apriltag_field,
                                  camera_params=case.camera_params)
    assert pose is not None
    camera_in_origin = pose.inv()
    assert camera_in_origin.input_space == case.actual_camera_pose.input_space
    assert camera_in_origin.output_space == case.actual_camera_pose.output_space
    assert np.isclose(camera_in_origin.matrix, case.actual_camera_pose.matrix, atol=10 ** -2).all()


@pytest.mark.parametrize('pnp_method,case', list(product(
    iter(PnPMethod),
    cases
)))
def test_lowest_ambiguity_strategy(pnp_method: PnPMethod,
                                   case: PoseEstimationStrategyTestCase):
    strategy_tester(LowestAmbiguityEstimationStrategy(pnp_method=pnp_method), case)


@pytest.mark.parametrize('fallback_strategy,pnp_method,case', list(product(
    (LowestAmbiguityEstimationStrategy(pnp_method=method) for method in PnPMethod),
    [PnPMethod.ITERATIVE, PnPMethod.SQPNP],
    cases
)))
def test_multitag_pnp_strategy(fallback_strategy: PoseEstimationStrategy,
                               pnp_method: PnPMethod,
                               case: PoseEstimationStrategyTestCase):
    strategy_tester(MultiTagPnPEstimationStrategy(fallback_strategy=fallback_strategy, pnp_method=pnp_method), case)


@pytest.mark.parametrize('fallback_strategy,pnp_method,case', list(product(
    [LowestAmbiguityEstimationStrategy(pnp_method=method) for method in PnPMethod],
    iter(PnPMethod),
    cases
)))
def test_multitag_special_strategy(fallback_strategy: PoseEstimationStrategy,
                                   pnp_method: PnPMethod,
                                   case: PoseEstimationStrategyTestCase):
    strategy_tester(MultiTagSpecialEstimationStrategy(angle_producer=lambda: case.actual_camera_pose.rotation, fallback_strategy=fallback_strategy, pnp_method=pnp_method), case)
