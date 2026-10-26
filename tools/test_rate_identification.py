"""Synthetic recovery tests; requires numpy, scipy and pyulog, not ROS or a vehicle."""
import unittest

import numpy as np
from scipy.signal import TransferFunction, lsim

from analyze_rate_identification import DT, fit, scores, simulate


class RateIdentificationTests(unittest.TestCase):
    def test_separates_dead_time_from_first_order_lag(self):
        t = np.arange(0., 27., DT)
        rng = np.random.default_rng(8)
        # 50 Hz piecewise-constant input, independently simulated continuous plant.
        u = np.repeat(rng.normal(0., .2, int(np.ceil(len(t)/4))), 4)[:len(t)]
        delayed = np.interp(t-.03, t, u, left=u[0])
        _, y, _ = lsim(TransferFunction([1.1], [.045, 1.]), U=delayed, T=t, interp=False)
        p = fit(u, y, 'first_order')
        self.assertAlmostEqual(p[0], 1.1, delta=.02)
        self.assertAlmostEqual(p[1], .03, delta=.006)
        self.assertAlmostEqual(p[2], .045, delta=.006)

    def test_underdamped_response_requires_second_order_on_held_out_input(self):
        t = np.arange(0., 27., DT)
        u = .2*np.sin(2*np.pi*(.4*t+.5*(5.6/27)*t*t))
        plant = TransferFunction([1.05*9.3**2], [1., 2*.49*9.3, 9.3**2])
        _, y, _ = lsim(plant, U=np.interp(t-.015, t, u, left=0.), T=t, interp=False)
        p = fit(u, y, 'second_order')
        self.assertAlmostEqual(p[0], 1.05, delta=.02)
        self.assertAlmostEqual(p[1], .015, delta=.006)
        self.assertAlmostEqual(p[2], 9.3, delta=.2)
        self.assertAlmostEqual(p[3], .49, delta=.03)
        v = np.where(t % 2 < .3, .2, np.where(t % 2 < .6, -.2, 0.))
        _, measured, _ = lsim(plant, U=np.interp(t-.015, t, v, left=.2), T=t, interp=False)
        second_error = scores(measured, simulate(p, 'second_order', v))['rmse_rad_s']
        first = fit(u, y, 'first_order')
        first_error = scores(measured, simulate(first, 'first_order', v))['rmse_rad_s']
        self.assertLess(second_error, .003)
        self.assertLess(second_error, first_error*.2)


if __name__ == '__main__':
    unittest.main()
