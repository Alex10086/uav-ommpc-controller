#!/usr/bin/env python3
"""Run sourced ROS: python3 -m unittest discover -s tools -p 'test_reference_test_publisher.py'."""
import copy
import math
import unittest

import numpy as np

from reference_test_publisher import (G, Trajectory, check_message, evaluate_segment,
                                      hermite7, rotation, sample_flat, sample_reference)


def local_elapsed(message, trajectory_elapsed):
    stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
    return trajectory_elapsed-(stamp-10**9)*1e-9


class PolynomialTrajectoryTests(unittest.TestCase):
    def test_hover_holds_position_and_hamilton_yaw_with_stable_stamp(self):
        origin = [1., -2., -3.]
        for yaw in [0., 1.67, math.pi-1e-10, -math.pi+1e-10]:
            trajectory = Trajectory('hover', origin=origin, yaw=yaw)
            msg = trajectory.message(1234567890123, 5.)
            self.assertEqual(msg.header.frame_id, 'ned')
            self.assertEqual(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec, 1234567890123)
            self.assertIs(msg, trajectory.message(1234567890123, 100.))
            self.assertTrue(msg.hold_final)
            self.assertEqual(len(msg.segments), 1)
            for elapsed in [0., 2.17, 100.]:
                state = sample_reference(msg, elapsed)
                np.testing.assert_allclose(state.position, origin, atol=1e-14)
                np.testing.assert_allclose(state.velocity, np.zeros(3), atol=1e-14)
                np.testing.assert_allclose(rotation(state.q)[:, 0], [math.cos(yaw), math.sin(yaw), 0.], atol=1e-14)
                np.testing.assert_allclose(state.body_rates, np.zeros(3), atol=1e-14)
                self.assertAlmostEqual(state.thrust_acceleration, G)

    def test_target_step_replaces_whole_equilibrium_plan(self):
        trajectory = Trajectory('target_step', origin=[1., 2., -2.], start_delay=2., offset=[.6, .3, 0.])
        for elapsed, goal in [(1.999, [1., 2., -2.]), (2., [1.6, 2.3, -2.])]:
            msg = trajectory.message(10**9, elapsed)
            self.assertTrue(msg.hold_final)
            self.assertEqual(len(msg.segments), 1)
            for t in [0., 1.999, 2., 30.]:
                state = sample_reference(msg, t)
                np.testing.assert_allclose(state.position, goal, atol=1e-14)
                np.testing.assert_allclose(state.velocity, np.zeros(3), atol=1e-14)

    def test_hermite_matches_known_degree_seven_and_physical_derivatives(self):
        coefficients = np.arange(24, dtype=float).reshape(8, 3)/19.
        duration = .7
        def derivatives(s):
            return [np.array([np.polynomial.polynomial.polyval(s,
                    np.polynomial.polynomial.polyder(coefficients[:, axis], k))
                    for axis in range(3)])/duration**k for k in range(4)]
        fitted = hermite7(derivatives(0.), derivatives(1.), duration)
        np.testing.assert_allclose(fitted, coefficients, atol=2e-12)

    def test_line_exact_ramp_cruise_braking_and_terminal_hold(self):
        trajectory = Trajectory('constant_velocity', direction=[0., 2., 0.], speed=.3,
                                ramp=3., line_duration=8., start_delay=2.)
        for elapsed in [0., 1.99, 2.01, 3.371, 5., 7., 12.99, 13., 14.129, 16., 17.]:
            msg = trajectory.message(10**9, elapsed)
            actual = sample_flat(msg, local_elapsed(msg, elapsed))
            exact = trajectory.flat(elapsed)
            for name in ('position', 'velocity', 'acceleration', 'jerk'):
                np.testing.assert_allclose(getattr(actual, name), getattr(exact, name), atol=1e-9)
        msg = trajectory.message(10**9, 17.)
        self.assertTrue(msg.hold_final)
        state = sample_reference(msg, 100.)
        np.testing.assert_allclose(state.position, [0., 3.3, 0.], atol=1e-12)
        np.testing.assert_allclose(state.velocity, np.zeros(3), atol=1e-12)

    def test_circle_approximates_analytic_geometry_and_preserves_heading(self):
        origin = np.array([1., -2., -3.])
        trajectory = Trajectory('circle', origin=origin, yaw=1.67, radius=.6, speed=.25)
        msg = trajectory.message(10**9, 8.)
        self.assertFalse(msg.hold_final)
        center = origin+[0., .6, 0.]
        for elapsed in np.linspace(8., 9., 11):
            local = local_elapsed(msg, elapsed)
            flat, state = sample_flat(msg, local), sample_reference(msg, local)
            exact = trajectory.flat(elapsed)
            for name in ('position', 'velocity', 'acceleration', 'jerk'):
                np.testing.assert_allclose(getattr(flat, name), getattr(exact, name), atol=1e-7)
            self.assertAlmostEqual(np.linalg.norm(state.position[:2]-center[:2]), .6, places=9)
            r = rotation(state.q)
            self.assertAlmostEqual(math.atan2(r[1, 0], r[0, 0]), 1.67, places=12)
            np.testing.assert_allclose(G*np.array([0., 0., 1.])-state.thrust_acceleration*r[:, 2], flat.acceleration, atol=1e-12)
        # The true analytic curve does not have artificially zero Euler defect.
        first = sample_flat(msg, local_elapsed(msg, 8.))
        following = sample_flat(msg, local_elapsed(msg, 8.02))
        self.assertGreater(np.linalg.norm(following.position-first.position-.02*first.velocity), 1e-6)

    def test_analytic_body_rates_match_orientation_derivative(self):
        trajectory = Trajectory('circle', yaw=.9)
        msg = trajectory.message(10**9, 3.1)
        t, epsilon = local_elapsed(msg, 3.1), 1e-5
        sample = sample_reference(msg, t)
        r = rotation(sample.q)
        derivative = (rotation(sample_reference(msg, t+epsilon).q)-rotation(sample_reference(msg, t-epsilon).q))/(2.*epsilon)
        w = r.T@derivative
        np.testing.assert_allclose(sample.body_rates, .5*np.array([w[2, 1]-w[1, 2], w[0, 2]-w[2, 0], w[1, 0]-w[0, 1]]), atol=1e-8)

    def test_knots_and_overlapping_windows_are_c3_and_time_continuous(self):
        for mode in ['constant_velocity', 'circle']:
            trajectory = Trajectory(mode, start_delay=2.173, ramp=2.837)
            previous = None
            for elapsed in [0., 2., 2.51, 3.01, 4.99, 5.51, 13.01, 16.01, 100.]:
                msg = trajectory.message(10**9, elapsed)
                check_message(msg)
                for left, right in zip(msg.segments, msg.segments[1:]):
                    a, b = evaluate_segment(left, left.duration), evaluate_segment(right, 0.)
                    for name in ('position', 'velocity', 'acceleration', 'jerk', 'yaw', 'yaw_rate'):
                        np.testing.assert_allclose(getattr(a, name), getattr(b, name), atol=1e-6)
                if previous is not None:
                    local = local_elapsed(previous, elapsed)
                    if 0. <= local <= sum(segment.duration for segment in previous.segments):
                        a, b = sample_flat(previous, local), sample_flat(msg, local_elapsed(msg, elapsed))
                        for name in ('position', 'velocity', 'acceleration', 'jerk'):
                            np.testing.assert_allclose(getattr(a, name), getattr(b, name), atol=1e-7)
                previous = msg

    def test_invalid_physics_time_and_hold_are_rejected_without_clipping(self):
        with self.assertRaises(ValueError):
            Trajectory('circle', radius=.1, speed=10.).message(10**9, 6.)
        with self.assertRaises(ValueError):
            Trajectory('polynomial')
        msg = Trajectory('circle').message(10**9, 8.)
        for elapsed in [-.01, math.nan, sum(s.duration for s in msg.segments)+.01]:
            with self.assertRaises(ValueError):
                sample_reference(msg, elapsed)
        malformed = copy.deepcopy(msg)
        malformed.hold_final = True
        with self.assertRaises(ValueError):
            check_message(malformed)
        malformed = copy.deepcopy(msg)
        malformed.segments[1].x[0] += .001
        with self.assertRaises(ValueError):
            check_message(malformed)


if __name__ == '__main__':
    unittest.main()
