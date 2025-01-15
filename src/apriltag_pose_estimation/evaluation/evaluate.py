import logging
import time
import tracemalloc
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from statistics import mean
from typing import List, Tuple, Dict, TypeAlias, Any

import numpy as np
import numpy.typing as npt

from apriltag_pose_estimation import AprilTagPoseEstimator, Pose, Twist, AprilTagPoseEstimationStrategy, \
    CameraParameters


logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class EvaluationCase:
    """A dataclass for cases against which pose estimation strategies are tested."""
    name: str
    """The name of the case."""
    image: npt.NDArray[np.uint8]
    """The image for the case. The image should contain an AprilTag which is detectable."""
    expected_tag_pose: Pose
    """The expected pose of the tag."""

    def __hash__(self):
        return hash(self.name)


@dataclass(frozen=True)
class EvaluationRun:
    """
    A dataclass whose instances store results from a single run of a strategy against a case.

    Use the :meth:`make` method to create instances of this class instead of constructing an instance directly.
    """
    time_seconds: float
    """The time it took to run the case, in seconds."""
    memory_bytes: int
    """The amount of memory used to run the case, in bytes."""
    calculated_tag_pose: Pose
    """The calculated pose of the AprilTag."""
    error: Twist
    """The error from the true pose, expressed as a twist."""
    error_magnitude: float
    """The magnitude of the error from the true pose."""

    @classmethod
    def make(cls, time_seconds: float, memory_bytes: int, calculated_tag_pose: Pose, expected_tag_pose: Pose) -> 'EvaluationRun':
        """
        Creates a new instance of ``EvaluationRun``.
        :param time_seconds: The time it took to run the case.
        :param memory_bytes: The amount of memory used to run the case.
        :param calculated_tag_pose: The calculated pose of the AprilTag.
        :param expected_tag_pose: The actual pose of the AprilTag.
        :return: A new instance of ``EvaluationRun``.
        """
        error, error_magnitude = get_error(expected_tag_pose, calculated_tag_pose)
        return EvaluationRun(time_seconds=time_seconds,
                             memory_bytes=memory_bytes,
                             calculated_tag_pose=calculated_tag_pose,
                             error=error,
                             error_magnitude=error_magnitude)


@dataclass(frozen=True)
class EvaluationResult:
    """
    A dataclass whose instances store results from every run of a strategy against a case.

    Use the :meth:`make` method to create instances of this class instead of constructing an instance directly.
    """
    average_time_seconds: float
    """The average time it took to run the case, in seconds."""
    average_memory_bytes: float
    """The average amount of memory used to run the case, in bytes."""
    average_error_magnitude: float
    """The average of the magnitude of the error from the true pose."""
    runs: List[EvaluationRun]
    """A list of all the runs of the strategy against a case."""

    @classmethod
    def make(cls, runs: Sequence[EvaluationRun]) -> 'EvaluationResult':
        """
        Creates a new instance of ``EvaluationResult``.
        :param runs: A sequence of all the runs of the strategy against a case.
        :return: A new instance of ``EvaluationResult``.
        """
        return EvaluationResult(average_time_seconds=mean(run.time_seconds for run in runs),
                                average_memory_bytes=mean(run.memory_bytes for run in runs),
                                average_error_magnitude=mean(run.error_magnitude for run in runs),
                                runs=list(runs))

    @staticmethod
    def summary_dict_factory(key_value_pairs: List[Tuple[str, Any]]) -> Dict[str, Any]:
        """
        Constructs a dictionary from key-value pairs.

        If the key is ``'runs'``, then it will not be included in the final dictionary.

        See :func:`dataclasses.as_dict` for more information.
        :param key_value_pairs: A list of key-value pairs.
        :return: A dictionary containing each of the key-value pairs, excluding the ``'runs'`` key.
        """
        return {key: value for key, value in key_value_pairs if key != 'runs'}


MultipleStrategyEvaluationResults: TypeAlias = Dict[AprilTagPoseEstimationStrategy, Dict[EvaluationCase, EvaluationResult]]


def evaluate_strategies(strategies: Iterable[AprilTagPoseEstimationStrategy],
                        cases: Iterable[EvaluationCase],
                        tag_size: float,
                        camera_params: CameraParameters,
                        runs_per_case: int = 10,
                        **estimator_params) -> MultipleStrategyEvaluationResults:
    """
    Evaluates the provided ``strategies`` against the provided ``cases``.
    :param strategies: An iterable containing all the strategies to evaluate.
    :param cases: An iterable containing all the cases against which the strategies are evaluated.
    :param tag_size: The size of the AprilTag for each of the cases.
    :param camera_params: The parameters of the camera used to capture the photo for each of the cases.
    :param runs_per_case: The number of times each case is run (default: 10).
    :param estimator_params: Parameters for an :class:`AprilTagPoseEstimator`.
    :return: A dictionary from strategies to a dictionary from cases to the results for the strategy and case.
    """
    results: MultipleStrategyEvaluationResults = {}
    for strategy in strategies:
        logger.info(f'Evaluating strategy {strategy.name}')
        estimator = AprilTagPoseEstimator(strategy=strategy,
                                          tag_size=tag_size,
                                          camera_params=camera_params,
                                          **estimator_params)
        results[strategy] = evaluate(estimator, cases, runs_per_case=runs_per_case)

    return results


def evaluate(estimator: AprilTagPoseEstimator,
             cases: Iterable[EvaluationCase],
             runs_per_case: int = 10) -> dict[EvaluationCase, EvaluationResult]:
    """
    Evaluates the given ``estimator`` against the provided ``cases``.

    :param estimator: The estimator to evaluate.
    :param cases: An iterable containing all the cases against which the estimator will be evaluated.
    :param runs_per_case: The number of times each case is run (default: 10).
    :return: A dictionary from cases to the results for the case.
    """
    results: dict[EvaluationCase, EvaluationResult] = {}
    for case in cases:
        logger.info(f'Running case {case.name}')
        runs: List[EvaluationRun] = []
        for _ in range(runs_per_case):
            tracemalloc.start()
            start = time.perf_counter()
            detection = estimator.estimate_tag_pose(case.image)[0]
            end = time.perf_counter()
            snapshot = tracemalloc.take_snapshot().filter_traces((
                tracemalloc.Filter(False, '<frozen importlib._bootstrap>'),
                tracemalloc.Filter(False, '<frozen importlib._bootstrap_external'),
                tracemalloc.Filter(False, '<unknown>'),
                tracemalloc.Filter(False, 'evaluate.py')
            ))
            tracemalloc.stop()
            runs.append(EvaluationRun.make(time_seconds=end - start,
                                           memory_bytes=sum(stat.size for stat in snapshot.statistics('filename')),
                                           calculated_tag_pose=detection.best_tag_pose,
                                           expected_tag_pose=case.expected_tag_pose))
        results[case] = EvaluationResult.make(runs)
    return results


def get_error(expected_pose: Pose, actual_pose: Pose) -> Tuple[Twist, float]:
    """
    Returns the error between the expected and actual poses.
    :param expected_pose: The expected pose.
    :param actual_pose: The actual pose.
    :return: A twist representing the error and the magnitude of the error.
    """
    error_pose_matrix = np.linalg.inv(expected_pose.get_matrix()) @ actual_pose.get_matrix()
    return Pose.from_matrix(error_pose_matrix).log(), float(np.log(np.linalg.norm(error_pose_matrix) ** 2 / 4))
