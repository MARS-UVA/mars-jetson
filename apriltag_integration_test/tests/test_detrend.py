import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from detrend import linear_detrend


class DetrendTests(unittest.TestCase):
    def test_known_bias_and_drift_leave_only_orthogonal_noise(self):
        time = np.arange(5.)
        noise = np.array([1., -2., 2., -2., 1.])
        error = 7 + .5*time + noise
        fit = linear_detrend(time, error)
        self.assertAlmostEqual(fit['intercept_deg'], 7)
        self.assertAlmostEqual(fit['slope_deg_per_s'], .5)
        np.testing.assert_allclose(fit['residuals'], noise, atol=1e-12)

    def test_constant_shift_does_not_change_standard_deviation(self):
        time = np.arange(10.)
        error = np.sin(time)
        a, b = linear_detrend(time, error), linear_detrend(time, error+100)
        np.testing.assert_allclose(a['residuals'], b['residuals'], atol=1e-12)
        self.assertAlmostEqual(np.std(error), np.std(b['demeaned']))

    def test_invalid_or_degenerate_data(self):
        for times, errors in [([0,0,0],[1,2,3]), ([0,1],[1,2]),
                              ([0,1,2],[1,2,float('nan')])]:
            with self.assertRaises(ValueError):
                linear_detrend(times, errors)


if __name__ == '__main__':
    unittest.main()
