#!/usr/bin/env python3
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from analyze_peak_control import payload_match, rate_relation, drag_metrics, reference_elapsed_at_anchor


class PeakControlDiagnostics(unittest.TestCase):
    def test_reference_start_tolerance_matches_controller(self):
        start = 1793199141951547996
        self.assertEqual(reference_elapsed_at_anchor(start-1907040,start),0.0)
        self.assertEqual(reference_elapsed_at_anchor(start-20000000,start),0.0)
        self.assertAlmostEqual(reference_elapsed_at_anchor(start+30000000,start),.03)
        with self.assertRaises(ValueError):
            reference_elapsed_at_anchor(start-20000001,start)
        with self.assertRaises(ValueError):
            reference_elapsed_at_anchor(0,start)

    def test_repeated_payload_is_ambiguous_not_nearest(self):
        t = np.array([1., 1.02, 1.3])
        payload = np.array([[.2, .1, -.3]]*3)
        indices, counts = payload_match(np.array([1.01, 1.3, 2.]), payload, t, payload)
        np.testing.assert_array_equal(indices, [-1, 2, -1])
        np.testing.assert_array_equal(counts, [2, 1, 0])

    def test_matching_uses_float32_wire_precision(self):
        source = np.array([[.22+1e-10, 0., -.3]])
        query = np.array([[.22, -0., -.3], [.22001, 0., -.3]])
        indices, _ = payload_match(np.array([1., 1.]), query, np.array([1.]), source)
        np.testing.assert_array_equal(indices, [0, -1])

    def test_lag_sign_and_axes_are_not_confused(self):
        t = np.arange(0., 4., .01)
        signal = lambda s: np.sin(2*np.pi*.7*s)+.4*np.sin(2*np.pi*1.3*s)
        scales, delays = [1., .7, -.5], [.042, -.026, .080]
        source = np.column_stack([scale*signal(t) for scale in scales])
        response = np.column_stack([scale*signal(t-delay) for scale,delay in zip(scales,delays)])
        result = rate_relation(t, source, response)
        for axis, delay in zip(result, delays):
            self.assertAlmostEqual(axis['effective_lag_ms'], delay*1000, places=6)
            self.assertFalse(axis['lag_search_boundary'])

    def test_constant_rates_do_not_produce_identified_lag(self):
        t = np.arange(0., 2., .02)
        result = rate_relation(t, np.zeros((len(t),3)), np.zeros((len(t),3)))
        for axis in result:
            self.assertIsNone(axis['zero_lag_correlation'])
            self.assertNotIn('effective_lag_ms', axis)

    def test_unknown_axial_thrust_does_not_become_lateral_drag(self):
        t = np.linspace(0., 3., 101)
        R = np.broadcast_to(Rotation.from_euler('xyz',[.4,-.3,.8]).as_matrix(),(len(t),3,3))
        v = np.column_stack([np.sin(t),np.cos(2*t),np.zeros_like(t)])
        axis = R[:,:,2]
        lateral_v = v-axis*np.sum(v*axis,axis=1)[:,None]
        a = np.array([0.,0.,9.8])-(10+3*np.sin(t))[:,None]*axis-.079*lateral_v
        direction = np.broadcast_to([1.,0.,0.],v.shape)
        metrics, measured, expected = drag_metrics(R,v,a,direction,.079)
        np.testing.assert_allclose(measured,expected,atol=1e-14)
        self.assertAlmostEqual(metrics['descriptive_best_k_s_inv'],.079,places=12)
        self.assertLess(metrics['mismatch_vector_rms_m_s2'],1e-14)


if __name__=='__main__':
    unittest.main()
