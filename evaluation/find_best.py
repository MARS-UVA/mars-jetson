import logging

import pandas as pd
from paretoset import paretoset

from cases import *
from data import get_per_case_summary_dataframe, get_rank_summary_dataframe
from evaluate import MultipleStrategyEvaluationResults, evaluate_strategies


logger = logging.getLogger(__name__)


def get_pareto_sets_for_cases(results: MultipleStrategyEvaluationResults) -> pd.DataFrame:
    """
    For each case in the results, finds the Pareto set of strategies.
    :param results: Results from :func:`evaluate_strategies`.
    :return: A :class:`DataFrame` containing the Pareto set of strategies for each case. The results for each strategy
             in the Pareto set are also included.
    """
    df = get_per_case_summary_dataframe(results)
    case_paretosets: pd.DataFrame = pd.DataFrame()
    for case in df['case'].unique():
        logger.info(f'Calculating pareto set for case {case}')
        case_df = df[df['case'] == case]
        case_paretosets = pd.concat([case_paretosets, case_df[paretoset(case_df.drop(columns=['case', 'strategy']))]])
    case_paretosets.set_index('case', inplace=True)
    return case_paretosets


def get_overall_pareto_set(results: MultipleStrategyEvaluationResults) -> pd.DataFrame:
    """
    Finds the Pareto set of strategies based on the mean of the ranks of each strategy in each evaluation parameter.
    :param results: Results from :func:`evaluate_strategies`.
    :return: A :class:`DataFrame` containing the Pareto set of strategies. The mean ranks of each strategy in each
             parameter are also included.
    """
    logger.info('Calculating overall pareto set')
    df = get_rank_summary_dataframe(results)
    return df[paretoset(df)]


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    data = evaluate_strategies(strategies=STRATEGIES,
                               cases=CASES,
                               tag_size=TAG_SIZE,
                               camera_params=CAMERA_PARAMETERS,
                               runs_per_case=RUNS_PER_CASE,
                               **DETECTOR_PARAMETERS)
    df_case_paretoset = get_pareto_sets_for_cases(data)
    df_case_paretoset.to_csv(CASE_PARETO_PATH)
    df_rank = get_rank_summary_dataframe(data)
    df_rank.to_csv(RANK_SUMMARY_PATH)
    df_paretoset = get_overall_pareto_set(data)
    df_paretoset.to_csv(OVERALL_PARETO_PATH)
