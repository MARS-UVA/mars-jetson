"""Descriptive line fitting for separating mean bias, time drift and residuals."""
import numpy as np


def linear_detrend(times, errors):
    times, errors = np.asarray(times, dtype=float), np.asarray(errors, dtype=float)
    if times.ndim != 1 or times.shape != errors.shape or len(times) < 3:
        raise ValueError('Need matching 1D arrays with at least three observations')
    if not np.isfinite(times).all() or not np.isfinite(errors).all():
        raise ValueError('Fit requires finite time and error values')
    origin = float(times.min())
    design = np.column_stack((times-origin, np.ones(len(times))))
    coefficients, _, rank, _ = np.linalg.lstsq(design, errors, rcond=None)
    if rank < 2:
        raise ValueError('Need at least two distinct observation times')
    fitted = design @ coefficients
    return {'time_origin_s': origin, 'slope_per_s': float(coefficients[0]),
            'intercept': float(coefficients[1]), 'slope_deg_per_s': float(coefficients[0]),
            'intercept_deg': float(coefficients[1]), 'fitted': fitted,
            'demeaned': errors-errors.mean(), 'residuals': errors-fitted}
