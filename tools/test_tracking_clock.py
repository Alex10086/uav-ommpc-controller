#!/usr/bin/env python3
"""Clock sign, drift, interpolation boundaries and body-force projection regressions."""
import unittest
from types import SimpleNamespace
import numpy as np
from analyze_tracking_diagnosis import ClockMap, interpolate, drag_evidence, validate_frame


def log_with_offsets(offsets):
    data = {'timestamp': np.array([1e6, 2e6, 3e6]),
            'estimated_offset': np.asarray(offsets),
            'round_trip_time': np.array([100., 200., 300.]),
            'source_protocol': np.array([2, 2, 2])}
    return SimpleNamespace(data_list=[SimpleNamespace(name='timesync_status', data=data)])


class TimingDiagnostics(unittest.TestCase):
    def test_heading_reset_is_not_a_position_origin_reset(self):
        fields = ('xy_reset_counter', 'z_reset_counter', 'ref_timestamp',
                  'vxy_reset_counter', 'vz_reset_counter', 'heading_reset_counter')
        local = {field: np.zeros(3) for field in fields}
        local['timestamp'] = np.array([1e6, 2e6, 3e6])
        local['heading_reset_counter'] = np.array([1, 1, 2])
        self.assertEqual(validate_frame(local, 1.)['heading_reset_counter'], [3.])
        local['xy_reset_counter'] = np.array([0, 0, 1])
        with self.assertRaises(ValueError):
            validate_frame(local, 1.)

    def test_negative_dds_offset_maps_native_to_epoch(self):
        clock = ClockMap(log_with_offsets([-1.7e15]*3))
        np.testing.assert_allclose(clock.ros([1.25, 2.75]), [1.7e9+1.25, 1.7e9+2.75], rtol=0, atol=1e-6)
        np.testing.assert_allclose(clock.native([1.7e9+1.25, 1.7e9+2.75]), [1.25, 2.75], rtol=0, atol=1e-6)
        with self.assertRaises(ValueError):
            clock.native(1.7e9+3.1)

    def test_drift_interpolation_does_not_optimize_flight_error(self):
        clock = ClockMap(log_with_offsets([-1.7e15, -1.7e15-10000, -1.7e15-20000]))
        ros = clock.ros([1.5, 2.5])
        np.testing.assert_allclose(ros, [1.7e9+1.505, 1.7e9+2.515], rtol=0, atol=1e-6)
        np.testing.assert_allclose(clock.native(ros), [1.5, 2.5], rtol=0, atol=1e-6)

    def test_nonmonotonic_epoch_mapping_rejected(self):
        with self.assertRaises(ValueError):
            ClockMap(log_with_offsets([-1.7e15, -1.7e15+2e6, -1.7e15+4e6]))

    def test_setpoint_hold_and_no_extrapolation(self):
        t, y = np.array([1., 2., 3.]), np.array([[1.], [4.], [-2.]])
        np.testing.assert_array_equal(interpolate([1., 1.99, 2.], t, y, hold=True)[:, 0], [1., 1., 4.])
        with self.assertRaises(ValueError):
            interpolate([.99], t, y)

    def test_lateral_projection_removes_unknown_thrust(self):
        t = np.linspace(0, 4, 201)
        velocity = np.column_stack([2*np.sin(t), np.zeros_like(t), np.zeros_like(t)])
        R = np.broadcast_to(np.eye(3), (len(t), 3, 3))
        acceleration = -.08*velocity
        acceleration[:, 2] = -3.+np.sin(2*t)  # varying axial thrust must not bias lateral fit
        report, residual, _, _ = drag_evidence(t, velocity, R, acceleration, velocity)
        self.assertAlmostEqual(report['effective_lateral_fit_xy'][0]['effective_drag_s_inverse'], .08)
        np.testing.assert_allclose(residual[:, 0], -.08*velocity[:, 0])
        np.testing.assert_array_equal(residual[:, 2], 0.)
        self.assertIsNone(report['effective_lateral_fit_xy'][1])


if __name__ == '__main__':
    unittest.main()
