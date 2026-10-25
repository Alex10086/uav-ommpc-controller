#!/usr/bin/env python3
"""Publish analytic NED polynomial plans only; never arm or change flight modes.

target_step replaces the entire hover target after start_delay; no polynomial
contains a teleport. Lines ramp, cruise, brake and hold. Circles, figure-8 and
3D helix trajectories ramp their angular speed and either continue indefinitely
or smoothly brake after a configured number of loops using overlapping C3
Hermite windows. All modes preserve initial yaw. Repeated plans retain their
ROS start stamp; publication refreshes the controller's receipt lease, not the
trajectory clock. Source the ROS/project overlays before running, including
with --check.
"""
import argparse
import math
import sys
from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import VehicleOdometry
from uav_ommpc_interfaces.msg import PolynomialSegment, PolynomialTrajectory

G = 9.80665
MODES = ('hover', 'target_step', 'constant_velocity', 'circle', 'figure8', 'helix', 'hover_to_hover')
RATE_LIMITS = np.array([3.0, 3.0, 1.5])
MAX_SPEED = 4.0
MAX_ACCELERATION = 8.0
MAX_JERK = 20.0
VEHICLES = {
    'x500': {
        'max_thrust': 12.0,
        'hover_thrust': 0.728744,
        'mass_kg': 2.06430769,
    },
    'super': {
        'max_thrust': 20.0,
        'hover_thrust': 0.309128,
        'mass_kg': 1.64,        # hku-mars/SUPER planner config (traj_opt.flatness.mass)
        'paper_mass_kg': 1.50,  # Ren et al., Science Robotics 10(98), eado6187 (2025)
    },
}


def vector3(value, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f'{name} must contain three finite values')
    return result.copy()


def rotation(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or norm < 1e-12:
        raise ValueError('invalid wxyz quaternion')
    w, x, y, z = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def quaternion(R):
    """Stable Hamilton wxyz conversion, including rotations near pi."""
    trace = float(np.trace(R))
    if trace > 0.0:
        s = 2.0 * math.sqrt(max(0.0, trace + 1.0))
        q = np.array([0.25*s, (R[2, 1]-R[1, 2])/s,
                      (R[0, 2]-R[2, 0])/s, (R[1, 0]-R[0, 1])/s])
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i+1) % 3, (i+2) % 3
        s = 2.0 * math.sqrt(max(0.0, 1.0+R[i, i]-R[j, j]-R[k, k]))
        q = np.zeros(4)
        q[0] = (R[k, j]-R[j, k])/s
        q[i+1] = 0.25*s
        q[j+1] = (R[j, i]+R[i, j])/s
        q[k+1] = (R[k, i]+R[i, k])/s
    q /= np.linalg.norm(q)
    return q if q[0] >= 0.0 else -q


@dataclass
class FlatSample:
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    jerk: np.ndarray
    yaw: float
    yaw_rate: float


@dataclass
class ReferenceSample:
    """Diagnostic evaluation of a polynomial, not a sampled wire message."""
    position: np.ndarray
    velocity: np.ndarray
    q: np.ndarray
    thrust_acceleration: float
    body_rates: np.ndarray


def flat_reference(flat):
    force = np.array([0., 0., G])-flat.acceleration
    thrust = float(np.linalg.norm(force))
    if not math.isfinite(thrust) or thrust <= 1e-6:
        raise ValueError('undefined thrust direction')
    b3 = force/thrust
    if b3[2] <= 1e-3:
        raise ValueError('reference is not upright')
    b3_dot = (-flat.jerk+b3*np.dot(b3, flat.jerk))/thrust
    ypsi = np.array([-math.sin(flat.yaw), math.cos(flat.yaw), 0.])
    ypsi_dot = flat.yaw_rate*np.array([-math.cos(flat.yaw), -math.sin(flat.yaw), 0.])
    h = np.cross(ypsi, b3)
    h_norm = np.linalg.norm(h)
    if h_norm <= 1e-6:
        raise ValueError('singular yaw/thrust frame')
    b1 = h/h_norm
    h_dot = np.cross(ypsi_dot, b3)+np.cross(ypsi, b3_dot)
    b1_dot = (h_dot-b1*np.dot(b1, h_dot))/h_norm
    b2 = np.cross(b3, b1)
    b2_dot = np.cross(b3_dot, b1)+np.cross(b3, b1_dot)
    R = np.column_stack((b1, b2, b3))
    R_dot = np.column_stack((b1_dot, b2_dot, b3_dot))
    W = R.T@R_dot
    rates = .5*np.array([W[2, 1]-W[1, 2], W[0, 2]-W[2, 0], W[1, 0]-W[0, 1]])
    if not np.all(np.isfinite([*flat.position, *flat.velocity, *R.flat, thrust, *rates])):
        raise ValueError('nonfinite generated reference')
    return ReferenceSample(flat.position, flat.velocity, quaternion(R), thrust, rates)


def hermite7(left, right, duration):
    """Normalized-power coefficients matching physical p/v/a/j at both ends."""
    c = np.zeros((8, 3))
    for k in range(4):
        c[k] = left[k]*duration**k/math.factorial(k)
    residual = np.array([
        right[k]*duration**k-sum(math.factorial(i)/math.factorial(i-k)*c[i]
                               for i in range(k, 4))
        for k in range(4)])
    # Inverse of the fixed endpoint system for coefficients s^4 .. s^7.
    c[4:] = np.array([[35., -15., 2.5, -1./6.],
                     [-84., 39., -7., .5],
                     [70., -34., 6.5, -.5],
                     [-20., 10., -2., 1./6.]])@residual
    return c


def _eval_axis(coeffs, s, inv_d, inv_d2, inv_d3):
    d0 = d1 = d2 = d3 = 0.0
    for c in reversed(coeffs):
        d3 = d3 * s + 3.0 * d2
        d2 = d2 * s + 2.0 * d1
        d1 = d1 * s + d0
        d0 = d0 * s + c
    return d0, d1 * inv_d, d2 * inv_d2, d3 * inv_d3


def evaluate_segment(segment, local_time):
    inv_d = 1.0 / segment.duration
    inv_d2 = inv_d * inv_d
    inv_d3 = inv_d2 * inv_d
    s = local_time * inv_d
    x0, x1, x2, x3 = _eval_axis(segment.x, s, inv_d, inv_d2, inv_d3)
    y0, y1, y2, y3 = _eval_axis(segment.y, s, inv_d, inv_d2, inv_d3)
    z0, z1, z2, z3 = _eval_axis(segment.z, s, inv_d, inv_d2, inv_d3)
    w0, w1, _, _ = _eval_axis(segment.yaw, s, inv_d, inv_d2, inv_d3)
    return FlatSample(
        np.array([x0, y0, z0]), np.array([x1, y1, z1]),
        np.array([x2, y2, z2]), np.array([x3, y3, z3]), w0, w1)


def sample_flat(message, elapsed):
    """Analytic evaluation, choosing the right-hand piece at internal knots."""
    if not math.isfinite(elapsed) or elapsed < 0. or not message.segments:
        raise ValueError('reference time is invalid or plan is empty')
    end = sum(segment.duration for segment in message.segments)
    if elapsed > end:
        if not message.hold_final:
            raise ValueError('polynomial plan exhausted')
        final = evaluate_segment(message.segments[-1], message.segments[-1].duration)
        return FlatSample(final.position, np.zeros(3), np.zeros(3), np.zeros(3), final.yaw, 0.)
    start = 0.
    for index, segment in enumerate(message.segments):
        if elapsed < start+segment.duration or index == len(message.segments)-1:
            return evaluate_segment(segment, elapsed-start)
        start += segment.duration
    raise ValueError('polynomial plan exhausted')


def sample_reference(message, elapsed):
    return flat_reference(sample_flat(message, elapsed))


def ramp_integral(value):
    """Integral from 0 to u of quintic smootherstep, clamped to u in [0, 1]."""
    u = np.clip(value, 0.0, 1.0)
    return u**4*(2.5 + u*(-3.0 + u))


class Trajectory:
    """Analytic geometry and cached, overlapping polynomial publication windows."""
    def __init__(self, mode='hover', origin=None, yaw=None, window=4., segment_duration=.5,
                 rate=25., offset=(.6, .3, 0.), speed=.3, radius=.6, z_amp=.35, loops=None,
                 start_delay=2., ramp=3., line_duration=8., direction=(1., 0., 0.),
                 max_thrust=12., max_speed=MAX_SPEED, max_acceleration=MAX_ACCELERATION,
                 max_jerk=MAX_JERK):
        if mode not in MODES:
            raise ValueError(f'mode must be one of {MODES}')
        self.mode = mode
        self.origin = vector3((0., 0., 0.) if origin is None else origin, 'origin')
        self.yaw = 0.0 if yaw is None else float(yaw)
        if not math.isfinite(self.yaw):
            raise ValueError('yaw must be finite')
        for name, value in (('window', window), ('segment_duration', segment_duration),
                            ('rate', rate), ('speed', speed), ('radius', radius),
                            ('z_amp', z_amp), ('ramp', ramp), ('line_duration', line_duration),
                            ('max_thrust', max_thrust), ('max_speed', max_speed),
                            ('max_acceleration', max_acceleration), ('max_jerk', max_jerk)):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
            setattr(self, name, float(value))
        if self.max_thrust <= G:
            raise ValueError('max_thrust must exceed gravity')
        if not math.isfinite(start_delay) or start_delay < 0.0:
            raise ValueError('start_delay must be finite and nonnegative')
        if loops is not None:
            if not math.isfinite(loops) or loops <= 0.0:
                raise ValueError('loops must be finite and positive')
            self.loops = float(loops)
        else:
            self.loops = None
        if window < 2. or segment_duration < .05 or segment_duration > window/2.:
            raise ValueError('window must be >=2 s and segment_duration in [.05, window/2]')
        if math.ceil(window/segment_duration)+5 > 100:
            raise ValueError('window requires more than 100 polynomial segments')
        self._cached_key = self._cached_message = None
        self.start_delay = float(start_delay)
        self.offset = vector3(offset, 'offset')
        self.direction = vector3(direction, 'direction')
        length = float(np.linalg.norm(self.direction))
        if not math.isfinite(length) or length < 1e-12:
            raise ValueError('direction must have a finite nonzero norm')
        self.direction /= length

    def _curve_omega(self):
        if self.mode == 'circle':
            return self.speed / self.radius
        if self.mode == 'figure8':
            return self.speed / (math.sqrt(2.0) * self.radius)
        if self.mode == 'helix':
            return self.speed / math.sqrt(self.radius**2 + 4.0 * self.z_amp**2)
        return 0.0

    def _curve_cruise_duration(self):
        if self.loops is None:
            return None
        omega = self._curve_omega()
        total_tau = (2.0 * math.pi * self.loops) / omega
        if total_tau <= self.ramp:
            raise ValueError('loops are too short for the configured ramp duration')
        return total_tau - self.ramp

    def _angular_progress(self, travel):
        omega = self._curve_omega()
        u = min(travel / self.ramp, 1.)
        smooth = u**3 * (10. + u * (-15. + 6. * u))
        smooth_dot = 30. * u**2 * (1. - u)**2 / self.ramp
        smooth_ddot = 60. * u * (1. - u) * (1. - 2. * u) / self.ramp**2
        integral = self.ramp * ramp_integral(u)
        cruise = self._curve_cruise_duration()
        if cruise is None or travel < self.ramp + cruise:
            tau = integral if travel < self.ramp else travel - 0.5 * self.ramp
            return omega * tau, omega * smooth, omega * smooth_dot, omega * smooth_ddot
        b = min((travel - self.ramp - cruise) / self.ramp, 1.)
        tau = 0.5 * self.ramp + cruise + self.ramp * (b - ramp_integral(b))
        tau_d1 = 1. - b**3 * (10. + b * (-15. + 6. * b))
        tau_d2 = -30. * b**2 * (1. - b)**2 / self.ramp
        tau_d3 = -60. * b * (1. - b) * (1. - 2. * b) / self.ramp**2
        return omega * tau, omega * tau_d1, omega * tau_d2, omega * tau_d3

    def flat(self, elapsed):
        """Exact underlying trajectory and derivatives, before polynomial fitting."""
        travel = max(elapsed-self.start_delay, 0.)
        u = min(travel/self.ramp, 1.)
        smooth = u**3*(10.+u*(-15.+6.*u))
        smooth_dot = 30.*u**2*(1.-u)**2/self.ramp
        smooth_ddot = 60.*u*(1.-u)*(1.-2.*u)/self.ramp**2
        integral = self.ramp*ramp_integral(u)
        if self.mode == 'constant_velocity':
            if travel < self.ramp:
                distance, velocity, acceleration, jerk = integral, smooth, smooth_dot, smooth_ddot
            elif travel < self.ramp+self.line_duration:
                distance, velocity, acceleration, jerk = travel-.5*self.ramp, 1., 0., 0.
            else:
                b = min((travel-self.ramp-self.line_duration)/self.ramp, 1.)
                distance = .5*self.ramp+self.line_duration+self.ramp*(b-ramp_integral(b))
                velocity = 1.-b**3*(10.+b*(-15.+6.*b))
                acceleration = -30.*b**2*(1.-b)**2/self.ramp
                jerk = -60.*b*(1.-b)*(1.-2.*b)/self.ramp**2
            axis = self.speed*self.direction
            return FlatSample(self.origin+distance*axis, velocity*axis,
                              acceleration*axis, jerk*axis, self.yaw, 0.)
        if self.mode == 'circle':
            theta, w, alpha, beta = self._angular_progress(travel)
            tangent = np.array([math.cos(theta), math.sin(theta), 0.])
            normal = np.array([-math.sin(theta), math.cos(theta), 0.])
            p = self.origin+self.radius*np.array([math.sin(theta), 1.-math.cos(theta), 0.])
            return FlatSample(p, self.radius*w*tangent,
                              self.radius*(alpha*tangent+w*w*normal),
                              self.radius*((beta-w**3)*tangent+3.*w*alpha*normal), self.yaw, 0.)
        if self.mode == 'figure8':
            theta, w, alpha, beta = self._angular_progress(travel)
            s1, c1 = math.sin(theta), math.cos(theta)
            s2, c2 = math.sin(2.0*theta), math.cos(2.0*theta)
            p = self.origin + self.radius * np.array([s1, 0.5*s2, 0.0])
            dp = self.radius * np.array([c1, c2, 0.0])
            ddp = self.radius * np.array([-s1, -2.0*s2, 0.0])
            dddp = self.radius * np.array([-c1, -4.0*c2, 0.0])
            return FlatSample(p, dp*w, ddp*(w*w) + dp*alpha,
                              dddp*(w**3) + 3.0*ddp*w*alpha + dp*beta, self.yaw, 0.)
        if self.mode == 'helix':
            theta, w, alpha, beta = self._angular_progress(travel)
            s1, c1 = math.sin(theta), math.cos(theta)
            s2, c2 = math.sin(2.0*theta), math.cos(2.0*theta)
            p = self.origin + np.array([self.radius*s1, self.radius*(1.0-c1), -self.z_amp*s2])
            dp = np.array([self.radius*c1, self.radius*s1, -2.0*self.z_amp*c2])
            ddp = np.array([-self.radius*s1, self.radius*c1, 4.0*self.z_amp*s2])
            dddp = np.array([-self.radius*c1, -self.radius*s1, 8.0*self.z_amp*c2])
            return FlatSample(p, dp*w, ddp*(w*w) + dp*alpha,
                              dddp*(w**3) + 3.0*ddp*w*alpha + dp*beta, self.yaw, 0.)
        if self.mode == 'hover_to_hover':
            travel = max(elapsed-self.start_delay, 0.)
            dist = float(np.linalg.norm(self.offset))
            duration = (35.0/16.0)*dist/self.speed  # Peak speed equals self.speed.
            u = min(travel/duration, 1.0)
            s = u
            poly = s**4*(35. + s*(-84. + s*(70. - 20.*s)))
            poly_d1 = s**3*(140. + s*(-420. + s*(420. - 140.*s))) / duration
            poly_d2 = s**2*(420. + s*(-1680. + s*(2100. - 840.*s))) / duration**2
            poly_d3 = s*(840. + s*(-5040. + s*(8400. - 4200.*s))) / duration**3
            p = self.origin + poly*self.offset
            v = poly_d1*self.offset
            a = poly_d2*self.offset
            j = poly_d3*self.offset
            return FlatSample(p, v, a, j, self.yaw, 0.0)
        p = self.origin.copy()
        if self.mode == 'target_step' and elapsed >= self.start_delay:
            p += self.offset
        return FlatSample(p, np.zeros(3), np.zeros(3), np.zeros(3), self.yaw, 0.)

    def message(self, start_ns, elapsed_s):
        """start_ns is the unchanged ROS start of this trajectory, not publish time."""
        if not math.isfinite(elapsed_s) or elapsed_s < 0.:
            raise ValueError('elapsed_s must be finite and nonnegative')
        if not math.isfinite(start_ns) or start_ns < 0 or int(start_ns) != start_ns:
            raise ValueError('start_ns must be a nonnegative integer')
        stationary = self.mode in ('hover', 'target_step')
        switched = self.mode == 'target_step' and elapsed_s >= self.start_delay
        anchor = (0. if stationary else
                  max(0., math.floor((elapsed_s-self.window/2.)/self.segment_duration)*self.segment_duration))
        key = (int(start_ns), anchor, switched)
        if key == self._cached_key:
            return self._cached_message
        msg = PolynomialTrajectory()
        stamp = int(start_ns)+round(anchor*1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        msg.header.frame_id = 'ned'
        end = math.ceil((anchor+self.window)/self.segment_duration)*self.segment_duration
        if stationary:
            p = self.origin+(self.offset if switched else np.zeros(3))
            segment = PolynomialSegment()
            segment.duration = self.window
            segment.x, segment.y, segment.z = ([float(value)] for value in p)
            segment.yaw = [self.yaw]
            msg.segments.append(segment)
            msg.hold_final = True
        else:
            # Global knots make overlapping windows describe exactly the same curve.
            knots = [anchor, end]
            first = math.floor(anchor/self.segment_duration)+1
            last = math.ceil(end/self.segment_duration)
            knots.extend(k*self.segment_duration for k in range(first, last)
                         if anchor < k*self.segment_duration < end)
            if self.mode == 'constant_velocity':
                boundaries = [self.start_delay, self.start_delay+self.ramp,
                              self.start_delay+self.ramp+self.line_duration,
                              self.start_delay+2.*self.ramp+self.line_duration]
            elif self.mode == 'hover_to_hover':
                dist = float(np.linalg.norm(self.offset))
                duration = (35.0/16.0)*dist/self.speed
                boundaries = [self.start_delay, self.start_delay+duration]
            elif self.mode in ('circle', 'figure8', 'helix') and self.loops is not None:
                cruise = self._curve_cruise_duration()
                boundaries = [self.start_delay, self.start_delay+self.ramp,
                              self.start_delay+self.ramp+cruise,
                              self.start_delay+2.*self.ramp+cruise]
            else:
                boundaries = [self.start_delay, self.start_delay+self.ramp]
            for b in boundaries:
                if anchor < b < end and min(abs(b - k) for k in knots) > 0.02:
                    knots.append(b)
            ordered = sorted(set(knots))
            knots = [ordered[0]]
            for knot in ordered[1:]:
                # Floating arithmetic must not create near-zero Hermite intervals.
                if knot-knots[-1] > 1e-10:
                    knots.append(knot)
            for left, right in zip(knots, knots[1:]):
                a, b = self.flat(left), self.flat(right)
                coefficients = hermite7(
                    (a.position, a.velocity, a.acceleration, a.jerk),
                    (b.position, b.velocity, b.acceleration, b.jerk), right-left)
                segment = PolynomialSegment()
                segment.duration = right-left
                segment.x, segment.y, segment.z = (coefficients[:, k].tolist() for k in range(3))
                segment.yaw = [self.yaw]
                msg.segments.append(segment)
            if self.mode == 'constant_velocity':
                msg.hold_final = end >= self.start_delay+2.*self.ramp+self.line_duration
            elif self.mode == 'hover_to_hover':
                dist = float(np.linalg.norm(self.offset))
                duration = (35.0/16.0)*dist/self.speed  # Peak speed equals self.speed.
                msg.hold_final = end >= self.start_delay+duration
            elif self.mode in ('circle', 'figure8', 'helix') and self.loops is not None:
                cruise = self._curve_cruise_duration()
                msg.hold_final = end >= self.start_delay+2.*self.ramp+cruise
        check_message(msg, max_thrust=self.max_thrust, max_speed=self.max_speed,
                      max_acceleration=self.max_acceleration, max_jerk=self.max_jerk)
        self._cached_key, self._cached_message = key, msg
        return msg


class ReferencePublisher(Node):
    """Reference-only node; unspecified origin/yaw wait for valid NED odometry."""
    def __init__(self, *, mode='hover', origin=None, yaw=None, window=4., segment_duration=.5,
                 rate=25., offset=(.6, .3, 0.), speed=.3, radius=.6, z_amp=.35, loops=None,
                 start_delay=2., ramp=3., line_duration=8., direction=(1., 0., 0.),
                 max_thrust=12., max_speed=MAX_SPEED, max_acceleration=MAX_ACCELERATION,
                 max_jerk=MAX_JERK, node_name='reference_test_publisher'):
        self.config = dict(window=window, segment_duration=segment_duration, rate=rate, offset=offset,
                           speed=speed, radius=radius, z_amp=z_amp, loops=loops, start_delay=start_delay,
                           ramp=ramp, line_duration=line_duration, direction=direction,
                           max_thrust=max_thrust, max_speed=max_speed,
                           max_acceleration=max_acceleration, max_jerk=max_jerk)
        Trajectory(mode=mode, origin=origin, yaw=yaw, **self.config)
        super().__init__(node_name)
        self.publish_enabled = True
        self.last_reference = None
        self.last_publish_ns = None
        self.generation_error = None
        self.trajectory = None
        self.trajectory_start_ns = None
        self._mode, self._origin, self._yaw = mode, origin, yaw
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE)
        self.reference_pub = self.create_publisher(PolynomialTrajectory, '/uav/reference_polynomial', reliable)
        self.state_subscription = self.create_subscription(VehicleOdometry, '/uav/state', self._on_state, sensor)
        if origin is not None and yaw is not None:
            self.set_trajectory(mode, origin, yaw)
        else:
            self.get_logger().info('Waiting for valid NED odometry to capture reference origin/yaw')
        self.timer = self.create_timer(1.0/rate, self._publish)

    def set_trajectory(self, mode, origin, yaw, start_ns=None):
        trajectory = Trajectory(mode=mode, origin=origin, yaw=yaw, **self.config)
        start = self.get_clock().now().nanoseconds if start_ns is None else start_ns
        if not math.isfinite(start) or start < 0 or int(start) != start:
            raise ValueError('start_ns must be a nonnegative integer')
        self.trajectory = trajectory
        self.trajectory_start_ns = int(start)
        self.generation_error = None

    def _on_state(self, msg):
        if self.trajectory is not None:
            return
        if (msg.pose_frame != VehicleOdometry.POSE_FRAME_NED or
                msg.velocity_frame != VehicleOdometry.VELOCITY_FRAME_NED or
                not np.all(np.isfinite(list(msg.position)+list(msg.velocity)+list(msg.q))) or
                not 0.9 < np.linalg.norm(msg.q) < 1.1):
            return
        R = rotation(msg.q)
        yaw = math.atan2(R[1, 0], R[0, 0]) if self._yaw is None else self._yaw
        self.set_trajectory(self._mode, msg.position if self._origin is None else self._origin, yaw)
        self.get_logger().info(f'Reference {self.trajectory.mode}: origin={self.trajectory.origin.tolist()}, yaw={yaw:.4f}')

    def _publish(self):
        if not self.publish_enabled or self.trajectory is None:
            return
        now = self.get_clock().now().nanoseconds
        try:
            elapsed = (now-self.trajectory_start_ns)*1e-9
            msg = self.trajectory.message(self.trajectory_start_ns, elapsed)
            self.reference_pub.publish(msg)
            self.last_reference = msg
            self.last_publish_ns = now
        except (ValueError, ArithmeticError) as exc:
            self.publish_enabled = False
            self.last_reference = None
            self.generation_error = str(exc)
            self.get_logger().error(f'Reference generation failed; publishing STOPPED: {exc}')


def check_message(msg, max_thrust=12., max_speed=MAX_SPEED,
                  max_acceleration=MAX_ACCELERATION, max_jerk=MAX_JERK):
    """Check C3/C1 knots, stationary hold and sampled analytic physical bounds.

    This diagnostic is not a proof of between-sample actuator feasibility; the
    controller independently checks every feedforward input it actually uses.
    Euler discretization residuals are deliberately NOT required to vanish.
    """
    if not math.isfinite(max_thrust) or max_thrust <= G:
        raise ValueError('max_thrust must be finite and exceed gravity')
    for name, val in (('max_speed', max_speed), ('max_acceleration', max_acceleration), ('max_jerk', max_jerk)):
        if not math.isfinite(val) or val <= 0.:
            raise ValueError(f'{name} must be finite and positive')
    if msg.header.frame_id != 'ned' or not 1 <= len(msg.segments) <= 100:
        raise ValueError('invalid polynomial frame or segment count')
    previous = None
    maxima = np.zeros(4)  # thrust, |roll rate|, |pitch rate|, |yaw rate|
    for segment in msg.segments:
        if not math.isfinite(segment.duration) or segment.duration <= 0.:
            raise ValueError('invalid polynomial duration')
        for coefficients in (segment.x, segment.y, segment.z, segment.yaw):
            if not 1 <= len(coefficients) <= 10 or not np.all(np.isfinite(coefficients)):
                raise ValueError('invalid polynomial coefficients')
        left = evaluate_segment(segment, 0.)
        if previous is not None:
            for name in ('position', 'velocity', 'acceleration', 'jerk', 'yaw', 'yaw_rate'):
                if np.linalg.norm(np.asarray(getattr(left, name))-getattr(previous, name)) > 1e-6:
                    raise ValueError(f'discontinuous polynomial {name}')
        n_pts = max(2, math.ceil(segment.duration/.05)+1)
        s_vec = np.linspace(0., 1., n_pts)
        inv_d = 1.0 / segment.duration
        inv_d2 = inv_d * inv_d
        inv_d3 = inv_d2 * inv_d
        pos = np.empty((n_pts, 3))
        vel = np.empty((n_pts, 3))
        acc = np.empty((n_pts, 3))
        jrk = np.empty((n_pts, 3))
        for ax, coeffs in enumerate((segment.x, segment.y, segment.z)):
            d0 = d1 = d2 = d3 = 0.0
            for c in reversed(coeffs):
                d3 = d3 * s_vec + 3.0 * d2
                d2 = d2 * s_vec + 2.0 * d1
                d1 = d1 * s_vec + d0
                d0 = d0 * s_vec + c
            pos[:, ax] = d0
            vel[:, ax] = d1 * inv_d
            acc[:, ax] = d2 * inv_d2
            jrk[:, ax] = d3 * inv_d3
        w0 = w1 = 0.0
        for c in reversed(segment.yaw):
            w1 = w1 * s_vec + w0
            w0 = w0 * s_vec + c
        yaw_vec = w0
        yaw_rate_vec = w1 * inv_d
        if np.max(np.linalg.norm(vel, axis=1)) > max_speed + 1e-5:
            raise ValueError(f'generated speed exceeds {max_speed:g} m/s')
        if np.max(np.linalg.norm(acc, axis=1)) > max_acceleration + 1e-5:
            raise ValueError(f'generated acceleration exceeds {max_acceleration:g} m/s^2')
        if np.max(np.linalg.norm(jrk, axis=1)) > max_jerk + 1e-4:
            raise ValueError(f'generated jerk exceeds {max_jerk:g} m/s^3')
        force = np.array([0., 0., G]) - acc
        thrust = np.linalg.norm(force, axis=1)
        if not np.all(np.isfinite(thrust)) or np.any(thrust <= 1e-6):
            raise ValueError('undefined thrust direction')
        b3 = force / thrust[:, None]
        if np.any(b3[:, 2] <= 1e-3):
            raise ValueError('reference is not upright')
        b3_dot = (-jrk + b3 * np.sum(b3 * jrk, axis=1)[:, None]) / thrust[:, None]
        ypsi = np.column_stack((-np.sin(yaw_vec), np.cos(yaw_vec), np.zeros(n_pts)))
        ypsi_dot = yaw_rate_vec[:, None] * np.column_stack((-np.cos(yaw_vec), -np.sin(yaw_vec), np.zeros(n_pts)))
        h = np.cross(ypsi, b3)
        h_norm = np.linalg.norm(h, axis=1)
        if not np.all(np.isfinite(h_norm)) or np.any(h_norm <= 1e-6):
            raise ValueError('singular yaw/thrust frame')
        b1 = h / h_norm[:, None]
        h_dot = np.cross(ypsi_dot, b3) + np.cross(ypsi, b3_dot)
        b1_dot = (h_dot - b1 * np.sum(b1 * h_dot, axis=1)[:, None]) / h_norm[:, None]
        b2 = np.cross(b3, b1)
        b2_dot = np.cross(b3_dot, b1) + np.cross(b3, b1_dot)
        rates = np.column_stack((np.sum(b2_dot * b3, axis=1), np.sum(b3_dot * b1, axis=1), np.sum(b1_dot * b2, axis=1)))
        if not np.all(np.isfinite(rates)) or not np.all(np.isfinite(pos)) or not np.all(np.isfinite(vel)):
            raise ValueError('nonfinite generated reference')
        if np.any(thrust < 1.) or np.any(thrust > max_thrust):
            raise ValueError(f'generated thrust exceeds [1, {max_thrust:g}] m/s^2')
        if np.any(np.abs(rates) > RATE_LIMITS):
            raise ValueError('generated body rates exceed [3, 3, 1.5] rad/s')
        maxima = np.maximum(maxima, [np.max(thrust), *np.max(np.abs(rates), axis=0)])
        previous = FlatSample(pos[-1], vel[-1], acc[-1], jrk[-1], float(yaw_vec[-1]), float(yaw_rate_vec[-1]))
    if msg.hold_final and any(np.linalg.norm(getattr(previous, name)) > 1e-6
                             for name in ('velocity', 'acceleration', 'jerk', 'yaw_rate')):
        raise ValueError('hold_final requires a stationary endpoint')
    return maxima


def parser():
    cli = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    cli.add_argument('--mode', choices=MODES, default='hover')
    cli.add_argument('--vehicle', choices=tuple(VEHICLES), default='x500',
                     help='Vehicle profile setting default max-thrust (x500=12 m/s^2, super=20 m/s^2)')
    cli.add_argument('--max-thrust', type=float, default=None,
                     help='Override maximum allowed feedforward thrust acceleration in m/s^2')
    cli.add_argument('--origin', nargs=3, type=float, metavar=('N', 'E', 'D'), help='NED meters; default: first valid odometry')
    cli.add_argument('--yaw', type=float, help='NED yaw radians; default: first valid odometry')
    cli.add_argument('--window', type=float, default=4., help='Polynomial window seconds (>=2); must cover the controller horizon plus transport age')
    cli.add_argument('--segment-duration', type=float, default=.5, help='Hermite piece seconds; default .5')
    cli.add_argument('--rate', type=float, default=25., help='Publishing Hz; must keep reference age below controller timeout')
    cli.add_argument('--offset', nargs=3, type=float, default=(.6, .3, 0.), metavar=('N', 'E', 'D'), help='target_step equilibrium displacement; whole horizon switches after start-delay')
    cli.add_argument('--speed', type=float, default=.3)
    cli.add_argument('--radius', type=float, default=.6)
    cli.add_argument('--z-amp', type=float, default=.35, help='Vertical amplitude (m) for 3D helix mode')
    cli.add_argument('--loops', type=float, default=None, help='Optional number of 2*pi periods before smooth braking on periodic curves')
    cli.add_argument('--start-delay', type=float, default=2.)
    cli.add_argument('--ramp', type=float, default=3.)
    cli.add_argument('--line-duration', type=float, default=8., help='Constant-speed cruise duration, excluding ramps')
    cli.add_argument('--direction', nargs=3, type=float, default=(1., 0., 0.), help='Normalized NED line direction')
    cli.add_argument('--duration', type=float, help='Stop publishing after this many ROS-clock seconds from trajectory start')
    cli.add_argument('--check', action='store_true', help='Offline analytic polynomial/physical-bounds check, no ROS node or publishing')
    return cli


def main():
    cli = parser()
    args, ros_args = cli.parse_known_args()
    options = vars(args).copy()
    duration, check = options.pop('duration'), options.pop('check')
    vehicle = options.pop('vehicle')
    if options['max_thrust'] is None:
        options['max_thrust'] = VEHICLES[vehicle]['max_thrust']
    try:
        if duration is not None and (not math.isfinite(duration) or duration <= 0.0):
            raise ValueError('duration must be finite and positive')
        trajectory = Trajectory(**options)
        if check:
            maxima = np.zeros(4)
            if duration is not None:
                end = duration
            elif args.mode == 'constant_velocity':
                end = trajectory.start_delay+2*trajectory.ramp+trajectory.line_duration
            elif args.mode in ('circle', 'figure8', 'helix'):
                if trajectory.loops is not None:
                    end = trajectory.start_delay + 2*trajectory.ramp + trajectory._curve_cruise_duration()
                else:
                    end = trajectory.start_delay + trajectory.ramp + 2*math.pi/trajectory._curve_omega()
            else:
                end = trajectory.start_delay+trajectory.window
            for elapsed in np.arange(0., end+trajectory.segment_duration, trajectory.segment_duration):
                maxima = np.maximum(maxima, check_message(
                    trajectory.message(1_000_000_000, elapsed), max_thrust=trajectory.max_thrust))
            if args.mode == 'target_step':
                maxima = np.maximum(maxima, check_message(
                    trajectory.message(1_000_000_000, trajectory.start_delay), max_thrust=trajectory.max_thrust))
            print(f'{args.mode} ({vehicle}, max_thrust={trajectory.max_thrust:g}): '
                  f'analytic C3/C1 polynomial windows checked through {end:.3f}s; '
                  f'sampled max thrust/body-rate magnitudes={maxima.tolist()}; '
                  'not a continuous-time actuator-feasibility certificate')
            return 0
    except (ValueError, OverflowError) as exc:
        cli.error(str(exc))
    rclpy.init(args=ros_args)
    node = None
    try:
        node = ReferencePublisher(**options)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
            if node.generation_error is not None:
                return 1
            if (duration is not None and node.trajectory_start_ns is not None and
                    (node.get_clock().now().nanoseconds-node.trajectory_start_ns)*1e-9 >= duration):
                node.publish_enabled = False
                break
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
