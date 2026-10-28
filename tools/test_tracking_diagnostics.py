"""Analytic tracking-error tests; no ROS, flight data, or fitted time shifts."""
import csv
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from tracking_diagnostics import analyze_phase


class TrackingDiagnosticsTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.prefix = Path(self.directory.name) / 'phase'

    def analyze(self, times, error, reference_velocity, **optional):
        error = np.asarray(error, dtype=float)
        reference_velocity = np.asarray(reference_velocity, dtype=float)
        reference_position = np.zeros_like(error)
        data = {
            'time_s': np.asarray(times, dtype=float),
            'reference_position': reference_position,
            'position': reference_position - error,
            'reference_velocity': reference_velocity,
            'velocity': reference_velocity.copy(),
            **optional,
        }
        metrics = analyze_phase(data, self.prefix)
        # All metrics, including absent statistics, must serialize as real JSON.
        json.dumps(metrics, allow_nan=False)
        with Path(metrics['outputs']['errors_csv']).open(newline='', encoding='utf-8') as stream:
            rows = list(csv.DictReader(stream))
        return metrics, rows

    def test_oblique_geometry_decomposes_error_without_losing_norm(self):
        direction = np.array([1., 2., 2.]) / 3.
        perpendicular = np.array([2., -1., 0.])
        errors = np.array([2. * direction + perpendicular, -3. * direction + 2. * perpendicular])
        metrics, rows = self.analyze([120., 120.25], errors, np.tile(direction, (2, 1)))
        for row, error in zip(rows, errors):
            tangent = float(row['tangent_error_m'])
            cross = float(row['cross_track_error_m'])
            vector = np.array([float(row[f'cross_track_vector_m_{axis}']) for axis in 'xyz'])
            self.assertAlmostEqual(tangent ** 2 + cross ** 2, np.dot(error, error))
            self.assertAlmostEqual(np.dot(vector, direction), 0.)
            np.testing.assert_allclose(tangent * direction + vector, error)
        self.assertAlmostEqual(float(rows[0]['tangent_error_m']), 2.)
        self.assertAlmostEqual(float(rows[1]['tangent_error_m']), -3.)
        self.assertAlmostEqual(metrics['full']['position_rmse_m'], np.sqrt(np.mean(np.sum(errors ** 2, axis=1))))
        np.testing.assert_allclose(metrics['full']['position_axis_rmse_m'], np.sqrt(np.mean(errors ** 2, axis=0)))

    def test_positive_lag_means_behind_with_no_time_shift(self):
        velocity = np.tile([2., 0., 0.], (3, 1))
        metrics, rows = self.analyze([50., 50.2, 50.4], np.tile([0.4, 0., 0.], (3, 1)), velocity)
        self.assertEqual(metrics['peak']['sample_index'], 0)
        self.assertAlmostEqual(metrics['full']['tangent_mean_m'], 0.4)
        self.assertAlmostEqual(metrics['moving']['position_rmse_m'], 0.4)
        self.assertTrue(all(float(row['tangent_error_m']) > 0. for row in rows))
        self.assertTrue(all(float(row['cross_track_error_m']) == 0. for row in rows))

    def test_stationary_maximum_retains_total_but_not_direction(self):
        metrics, rows = self.analyze(
            [2., 2.5, 3.], [[3., 4., 0.], [0., 0., 0.], [1., 0., 0.]],
            [[0., 0., 0.], [0.05, 0., 0.], [1., 0., 0.]])
        self.assertEqual(metrics['peak']['position_error_m'], 5.)
        self.assertIsNone(metrics['peak']['tangent_error_m'])
        self.assertIsNone(metrics['peak']['cross_track_error_m'])
        self.assertEqual(metrics['peak']['cross_track_vector_m'], [None, None, None])
        self.assertFalse(metrics['peak']['moving'])
        self.assertEqual(metrics['moving']['sample_count'], 1)
        self.assertEqual(metrics['moving']['position_rmse_m'], 1.)
        self.assertAlmostEqual(metrics['full']['position_rmse_m'], np.sqrt(26. / 3.))
        for row in rows[:2]:
            self.assertEqual(row['tangent_error_m'], '')
            self.assertEqual(row['cross_track_error_m'], '')
        self.assertEqual(float(rows[0]['position_error_m']), 5.)

    def test_all_stationary_single_sample_has_no_empty_statistics(self):
        metrics, rows = self.analyze([4.], [[0., 0., 0.]], [[0., 0., 0.]])
        self.assertEqual(metrics['duration_s'], 0.)
        self.assertEqual(metrics['full']['position_rmse_m'], 0.)
        self.assertIsNone(metrics['full']['tangent_rmse_m'])
        self.assertEqual(metrics['moving']['sample_count'], 0)
        self.assertIsNone(metrics['moving']['position_rmse_m'])
        self.assertEqual(metrics['moving']['position_axis_rmse_m'], [None, None, None])
        self.assertIsNone(metrics['peak_window']['max_sample_gap_s'])
        self.assertEqual(metrics['peak_window']['before_peak_s'], 0.)
        self.assertEqual(metrics['peak_window']['after_peak_s'], 0.)
        self.assertEqual(len(rows), 1)

    def test_first_maximum_timestamp_and_honest_boundary_coverage(self):
        times = np.array([101.125, 101.375, 101.625, 102.25, 102.625])
        errors = np.column_stack(([1., 4., 4., 2., 0.], np.zeros((5, 2))))
        metrics, rows = self.analyze(times, errors, np.tile([1., 0., 0.], (5, 1)))
        peak = metrics['peak']
        self.assertEqual(peak['sample_index'], 1)
        self.assertEqual(peak['native_time_s'], 101.375)
        self.assertEqual(peak['phase_time_s'], 0.25)
        coverage = metrics['peak_window']
        self.assertEqual(coverage['requested_native_start_s'], 100.375)
        self.assertEqual(coverage['requested_native_end_s'], 102.375)
        self.assertEqual(coverage['observed_native_start_s'], 101.125)
        self.assertEqual(coverage['observed_native_end_s'], 102.25)
        self.assertEqual(coverage['before_peak_s'], 0.25)
        self.assertEqual(coverage['after_peak_s'], 0.875)
        self.assertEqual(coverage['max_sample_gap_s'], 0.625)
        self.assertTrue(coverage['clipped_at_phase_start'])
        self.assertFalse(coverage['clipped_at_phase_end'])
        np.testing.assert_array_equal([float(row['time_s']) for row in rows], times)
        np.testing.assert_array_equal([float(row['phase_time_s']) for row in rows], times - times[0])
        self.assertNotIn('actual_rates', peak)
        self.assertNotIn('actual_rates_x', rows[0])
        for path in metrics['outputs'].values():
            self.assertGreater(Path(path).stat().st_size, 0)

    def test_last_sample_peak_and_optional_evidence_are_not_padded(self):
        metrics, rows = self.analyze(
            [20., 20.5, 21.], [[0., 0., 0.], [1., 0., 0.], [2., 0., 0.]],
            np.tile([1., 0., 0.], (3, 1)),
            actual_rates=np.tile([0.1, 0.2, 0.3], (3, 1)),
            desired_rates=np.tile([0.2, 0.3, 0.4], (3, 1)),
            attitude_error=np.tile([0.01, 0., 0.], (3, 1)),
            state_age_ms=np.array([2., 3., 4.]),
            model_residual=np.tile([0., 0.01, 0.], (3, 1)))
        self.assertTrue(metrics['peak_window']['clipped_at_phase_end'])
        self.assertFalse(metrics['peak_window']['clipped_at_phase_start'])
        self.assertEqual(metrics['peak_window']['after_peak_s'], 0.)
        self.assertEqual(metrics['peak_window']['before_peak_s'], 1.)
        self.assertEqual(metrics['peak']['state_age_ms'], 4.)
        self.assertEqual(metrics['peak']['actual_rates'], [0.1, 0.2, 0.3])
        self.assertEqual(len(rows), 3)
        self.assertEqual(float(rows[-1]['model_residual_y']), 0.01)

    def test_rejects_empty_nonfinite_misaligned_and_unordered_data(self):
        base = {
            'time_s': np.array([1., 2.]),
            'reference_position': np.zeros((2, 3)),
            'reference_velocity': np.ones((2, 3)),
            'position': np.zeros((2, 3)),
            'velocity': np.ones((2, 3)),
        }
        invalid = [
            {'time_s': np.array([])},
            {'time_s': np.array([1., 1.])},
            {'time_s': np.array([2., 1.])},
            {'time_s': np.array([1., np.inf])},
            {'position': np.zeros((1, 3))},
            {'position': np.full((2, 3), np.nan)},
            {'actual_rates': np.full((2, 3), np.inf)},
            {'state_age_ms': np.zeros((2, 1))},
        ]
        for replacement in invalid:
            with self.subTest(replacement=replacement):
                with self.assertRaises(ValueError):
                    analyze_phase({**base, **replacement}, self.prefix)


if __name__ == '__main__':
    unittest.main()
