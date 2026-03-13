from collections import defaultdict
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Self

import numpy as np
import numpy.typing as npt
import pandas as pd
from scipy.spatial.transform import Rotation


EXPECTED_LOCATIONS = {
    1: [3.56, 1.84],
    2: [3.22, 2.31],
    3: [2.89, 2.68],
    4: [2.575, 2.96],
    5: [1.595, 3.42],
}


EXPECTED_YAWS = {
    i: np.atan2(-p[1], -p[0]) for i, p in EXPECTED_LOCATIONS.items()
}


@dataclass(frozen=True)
class Summary:
    method: str
    height: float
    angle_of_elevation: float
    index: int
    x_error: float
    y_error: float
    z_error: float
    distance_error: float
    distance_std: float
    angle_error: float
    angle_std: float

    @classmethod
    def of(cls,
           method: str,
           height: float,
           angle_of_elevation: float,
           index: int,
           translation_errors: npt.NDArray[np.float64],
           rotation_errors: Rotation) -> Self:
        mean_translation_error = translation_errors.mean(axis=1)

        distance_error = np.linalg.norm(translation_errors, axis=1)
        rotation_error = np.rad2deg(rotation_errors.magnitude())
        return cls(method=method,
                   height=height,
                   angle_of_elevation=angle_of_elevation,
                   index=index,
                   x_error=mean_translation_error[0],
                   y_error=mean_translation_error[1],
                   z_error=mean_translation_error[2],
                   distance_error=distance_error.mean(),
                   distance_std=distance_error.std(),
                   angle_error=rotation_error.mean(),
                   angle_std=rotation_error.std())


@dataclass(frozen=True)
class DatasetId:
    method: str
    height: float
    angle_of_elevation: float
    index: int


CONVERSION_ROTATION = Rotation.from_matrix(np.array(
    [[ 0,  0, +1],
     [-1,  0,  0],
     [ 0,  -1, 0],]
))


def analyze_errors(data_dir: Path) -> None:
    summaries: list[Summary] = []
    for data_file in data_dir.rglob('*.csv'):
        df = pd.read_csv(data_file, index_col='frame')
        translations = df[['x', 'y', 'z']]
        rotations = Rotation.from_euler('ZXZ', df[['zxz_1', 'zxz_2', 'zxz_3']], degrees=False)
        index_char, method = data_file.stem.split('_', maxsplit=1)
        index = int(index_char)
        height, angle_of_elevation = map(float, data_file.parent.name.split('_', maxsplit=1))
        expected_translation = np.array(EXPECTED_LOCATIONS[index] + [(height - 244) / 100], dtype=np.float64)
        expected_rotation = Rotation.from_euler(
            'ZY',
            [EXPECTED_YAWS[index], -np.deg2rad(angle_of_elevation)],
            degrees=False
        ) * CONVERSION_ROTATION
        translation_errors = (translations - expected_translation).astype(np.float64)
        rotation_errors = expected_rotation.inv() * rotations

        summaries.append(Summary.of(method=method,
                                    height=height,
                                    angle_of_elevation=angle_of_elevation,
                                    index=index,
                                    translation_errors=translation_errors,
                                    rotation_errors=rotation_errors))

    results = pd.DataFrame(map(asdict, summaries))
    results.to_csv('testing_results/errors.csv', index=False)


# def save_covariances(data_dir: Path) -> None:
#     for data_file in data_dir.rglob('*.csv'):
#         df = pd.read_csv(data_file, index_col='frame')
#         df.cov(numeric_only=True).to_html(data_file.with_stem(f'{data_file.stem}_covariance').with_suffix('.html'))


if __name__ == '__main__':
    target_dir = Path('testing_data')
    analyze_errors(target_dir)
    # save_covariances(target_dir)
