from dataclasses import asdict

import numpy as np
import pandas as pd

from apriltag_pose_estimation import Pose, Twist
from apriltag_pose_estimation.strategies import PerspectiveNPointStrategy, PnPMethod
from evaluate import EvaluationCase, EvaluationRun, EvaluationResult, MultipleStrategyEvaluationResults


def get_per_case_summary_dataframe(results: MultipleStrategyEvaluationResults) -> pd.DataFrame:
    """
    Returns a ``DataFrame`` containing the mean results for each case in each evaluation parameter for each strategy.
    :param results: The results which will be summarized.
    :return: A ``DataFrame`` summarizing the results.
    """
    df = pd.DataFrame(
        {'case': case.name, 'strategy': strategy.name, **asdict(result, dict_factory=EvaluationResult.summary_dict_factory)}
        for strategy, cases in results.items()
        for case, result in cases.items()
    )
    return df


def get_rank_summary_dataframe(results: MultipleStrategyEvaluationResults) -> pd.DataFrame:
    """
    Returns a ``DataFrame`` containing the mean rank across all cases in each evaluation parameter for each strategy.
    :param results: The results which will be summarized.
    :return: A ``DataFrame`` summarizing the results.
    """
    return (get_per_case_summary_dataframe(results)
            .set_index(['case', 'strategy'])
            .groupby('case', observed=True)
            .rank()
            .groupby('strategy', observed=True)
            .mean()
            .rename(columns=lambda old_name: f'mean_rank_{old_name}'))


def get_mock_data() -> MultipleStrategyEvaluationResults:
    """Returns mock evaluation data for testing purposes."""
    rng = np.random.default_rng()
    results: MultipleStrategyEvaluationResults = {}
    for strategy in (PerspectiveNPointStrategy(method=method) for method in PnPMethod):
        average_error = abs(float(rng.normal(0.5, 0.1)))
        cases = {}
        for case_index in range(50):
            case = EvaluationCase(name=f'case{case_index}',
                                  image=rng.integers(low=0, high=128, size=(480, 360)).astype(np.uint8),
                                  expected_tag_pose=Pose.from_matrix(np.identity(4)))
            average_time = rng.normal(0.01, 0.002)
            cases[case] = EvaluationResult.make([EvaluationRun.make(time_seconds=float(rng.normal(average_time, 0.002)),
                                                                    memory_bytes=int(rng.normal(65536, 128)),
                                                                    calculated_tag_pose=Twist.from_vectors(rng.normal(0, average_error, size=3),
                                                                                                           rng.normal(0, average_error, size=3)).exp(),
                                                                    expected_tag_pose=case.expected_tag_pose)
                                                 for _ in range(20)])
        results[strategy] = cases
    return results
