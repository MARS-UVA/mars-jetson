import os
import warnings
from pathlib import Path
from typing import List

import cv2

import numpy as np
import numpy.typing as npt
from pupil_apriltags import Detector

from apriltag_pose_estimation import Pose, CameraParameters, AprilTagPoseEstimationStrategy
from apriltag_pose_estimation.strategies import (HomographyOrthogonalIterationStrategy, PerspectiveNPointStrategy,
                                                 PnPMethod)

from data import EvaluationCase


__all__ = ['CAMERA_PARAMETERS',
           'CASES',
           'CASE_PARETO_PATH',
           'DETECTOR_PARAMETERS',
           'OVERALL_PARETO_PATH',
           'RANK_SUMMARY_PATH',
           'RUNS_PER_CASE',
           'STRATEGIES',
           'TAG_SIZE']


def load_image(file: str | os.PathLike) -> npt.NDArray[np.uint8]:
    return cv2.cvtColor(cv2.imread(str(Path(file).absolute())), cv2.COLOR_BGR2GRAY)


CASE_PARETO_PATH = 'case_pareto.csv'
RANK_SUMMARY_PATH = 'rank_summary.csv'
OVERALL_PARETO_PATH = 'overall_pareto.csv'

STRATEGIES: List[AprilTagPoseEstimationStrategy] = [
    HomographyOrthogonalIterationStrategy(),
    PerspectiveNPointStrategy(PnPMethod.ITERATIVE),
    PerspectiveNPointStrategy(PnPMethod.AP3P),
    PerspectiveNPointStrategy(PnPMethod.IPPE),
    PerspectiveNPointStrategy(PnPMethod.SQPNP)
]

CAMERA_PARAMETERS = CameraParameters(fx=1329.143348,
                                     fy=1326.537785,
                                     cx=945.392392,
                                     cy=521.144703,
                                     k1=-0.348650,
                                     k2=0.098710,
                                     p1=-0.000157,
                                     p2=-0.001851,
                                     k3=0.000000)
TAG_SIZE = 0.150
DETECTOR_PARAMETERS = dict(
    families='tagStandard41h12',
    nthreads=2,
    quad_sigma=0,
    refine_edges=1,
    decode_sharpening=0.25
)

CASES = [
    EvaluationCase(
        name='example',
        image=load_image('../photo.jpg'),
        expected_tag_pose=Pose.from_matrix(np.identity(4))
    )
]

RUNS_PER_CASE = 10


def filter_valid_cases(cases: List[EvaluationCase]) -> List[EvaluationCase]:
    detector = Detector(**DETECTOR_PARAMETERS)
    valid_cases: List[EvaluationCase] = []
    for case in cases:
        if detector.detect(case.image):
            valid_cases.append(case)
        else:
            warnings.warn(f'case {case.name} discarded because an AprilTag could not be detected')
    return valid_cases


CASES = filter_valid_cases(CASES)
