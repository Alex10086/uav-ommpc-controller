#!/usr/bin/env python3
"""Run in the sourced ROS environment: python3 -m unittest discover -s tools -p 'test_reference_test_publisher.py'."""
import math
import unittest

import numpy as np

from reference_test_publisher import Trajectory


def matrix(q):
    # Independent Hamilton quaternion action, not the generator's conversion helper.
    w = q[0]
    v = np.array(q[1:])
    skew = np.array([[0., -v[2], v[1]], [v[2], 0., -v[0]], [-v[1], v[0], 0.]])
    return (w*w-v.dot(v))*np.eye(3) + 2.*np.outer(v, v) + 2.*w*skew


class ReferenceTrajectoryTests(unittest.TestCase):
    def test_hover_keeps_position_velocity_and_hamilton_yaw(self):
        origin = [1., -2., -3.]
        for yaw in [0., 1.67, math.pi-1e-10, -math.pi+1e-10]:
            msg = Trajectory('hover', origin=origin, yaw=yaw).message(1234567890123, 5.)
            self.assertEqual(msg.header.frame_id, 'ned')
            self.assertEqual(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec, 1234567890123)
            self.assertEqual(len(msg.states), len(msg.inputs)+1)
            for state in msg.states:
                np.testing.assert_allclose(state.position, origin, atol=1e-14)
                np.testing.assert_allclose(state.velocity, [0., 0., 0.], atol=1e-14)
                np.testing.assert_allclose(matrix(state.q)[:, 0], [math.cos(yaw), math.sin(yaw), 0.], atol=1e-14)
            self.assertAlmostEqual(msg.inputs[0].thrust_acceleration, 9.80665)

    def test_target_step_switches_equilibria_without_an_in_horizon_teleport(self):
        trajectory = Trajectory('target_step', origin=[1., 2., -2.], start_delay=2., offset=[.6, .3, 0.])
        for elapsed, goal in [(1.999, [1., 2., -2.]), (2., [1.6, 2.3, -2.])]:
            msg = trajectory.message(10**9, elapsed)
            for state in msg.states:
                np.testing.assert_allclose(state.position, goal, atol=1e-14)
                np.testing.assert_allclose(state.velocity, [0., 0., 0.], atol=1e-14)

    def test_line_reaches_cruise_then_brakes_and_holds_the_correct_endpoint(self):
        trajectory = Trajectory('constant_velocity', direction=[0., 2., 0.], speed=.3,
                                ramp=3., line_duration=8., start_delay=2.)
        cruise = trajectory.message(10**9, 7.).states[0]
        np.testing.assert_allclose(cruise.velocity, [0., .3, 0.], atol=1e-12)
        stopped = trajectory.message(10**9, 17.)
        for state in stopped.states:
            np.testing.assert_allclose(state.position, [0., 3.3, 0.], atol=1e-12)
            np.testing.assert_allclose(state.velocity, [0., 0., 0.], atol=1e-12)

    def test_circle_stays_on_radius_and_preserves_heading_not_tangent_yaw(self):
        origin = np.array([1., -2., -3.])
        yaw = 1.67
        trajectory = Trajectory('circle', origin=origin, yaw=yaw, radius=.6, speed=.25)
        msg = trajectory.message(10**9, 8.)
        center = origin + [0., .6, 0.]
        for state in msg.states:
            self.assertAlmostEqual(np.linalg.norm(np.array(state.position)[:2]-center[:2]), .6, places=12)
            self.assertAlmostEqual(state.position[2], origin[2], places=12)
            r = matrix(state.q)
            self.assertAlmostEqual(math.atan2(r[1, 0], r[0, 0]), yaw, places=12)
        self.assertAlmostEqual(np.linalg.norm(msg.states[0].velocity), .25, delta=1e-5)

    def test_smooth_windows_obey_the_nonlinear_discrete_model(self):
        for mode in ['constant_velocity', 'circle']:
            trajectory = Trajectory(mode, origin=[.5, -.7, -2.], yaw=1.67, speed=.25)
            for elapsed in [0., 1.999, 2., 2.01, 4.99, 5., 12.99, 13., 15.99, 16.]:
                msg = trajectory.message(10**9, elapsed)
                for k, control in enumerate(msg.inputs):
                    left, right = msg.states[k:k+2]
                    r = matrix(left.q)
                    next_r = matrix(right.q)
                    np.testing.assert_allclose(np.array(left.position)+msg.dt*np.array(left.velocity), right.position, atol=1e-12)
                    np.testing.assert_allclose(np.array(left.velocity)+msg.dt*(np.array([0., 0., 9.80665])-control.thrust_acceleration*r[:, 2]), right.velocity, atol=1e-12)
                    delta = np.array(control.body_rates)*msg.dt
                    theta = np.linalg.norm(delta)
                    if theta > 1e-14:
                        axis = delta/theta
                        q = [math.cos(theta/2.), *(math.sin(theta/2.)*axis)]
                        np.testing.assert_allclose(r@matrix(q), next_r, atol=1e-12)
                    else:
                        np.testing.assert_allclose(r, next_r, atol=1e-12)

    def test_infeasible_circle_is_rejected_instead_of_clipped(self):
        trajectory = Trajectory('circle', radius=.1, speed=10.)
        with self.assertRaises(ValueError):
            trajectory.message(10**9, 6.)
        with self.assertRaises(ValueError):
            Trajectory('polynomial')


if __name__ == '__main__':
    unittest.main()
