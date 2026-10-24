#!/usr/bin/env python3
"""Publish analytic NED polynomial plans only; never arm or change flight modes.

target_step replaces the entire hover target after start_delay; no polynomial
contains a teleport. Lines ramp, cruise, brake and hold. Circles ramp their
angular speed and continue indefinitely using overlapping C3 Hermite windows.
All modes preserve initial yaw. Repeated plans retain their ROS start stamp;
publication refreshes the controller's receipt lease, not the trajectory clock.
Source the ROS/project overlays before running, including with --check.
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
MODES = ('hover', 'target_step', 'constant_velocity', 'circle')
RATE_LIMITS = np.array([3.0, 3.0, 1.5])


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


def evaluate_segment(segment, local_time):
    s = local_time/segment.duration
    derivatives = []
    for order in range(4):
        derivatives.append(np.array([
            np.polynomial.polynomial.polyval(
                s, np.polynomial.polynomial.polyder(coefficients, order))/segment.duration**order
            for coefficients in (segment.x, segment.y, segment.z)]))
    yaw = float(np.polynomial.polynomial.polyval(s, segment.yaw))
    yaw_rate = float(np.polynomial.polynomial.polyval(
        s, np.polynomial.polynomial.polyder(segment.yaw)))/segment.duration
    return FlatSample(*derivatives, yaw, yaw_rate)


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
                 rate=25., offset=(.6, .3, 0.), speed=.3, radius=.6,
                 start_delay=2., ramp=3., line_duration=8., direction=(1., 0., 0.)):
        if mode not in MODES:
            raise ValueError(f'mode must be one of {MODES}')
        self.mode = mode
        self.origin = vector3((0., 0., 0.) if origin is None else origin, 'origin')
        self.yaw = 0.0 if yaw is None else float(yaw)
        if not math.isfinite(self.yaw):
            raise ValueError('yaw must be finite')
        for name, value in (('window', window), ('segment_duration', segment_duration),
                            ('rate', rate), ('speed', speed), ('radius', radius),
                            ('ramp', ramp), ('line_duration', line_duration)):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
            setattr(self, name, float(value))
        if not math.isfinite(start_delay) or start_delay < 0.0:
            raise ValueError('start_delay must be finite and nonnegative')
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
            omega = self.speed/self.radius
            theta = omega*(integral if travel < self.ramp else travel-.5*self.ramp)
            w, alpha, beta = omega*smooth, omega*smooth_dot, omega*smooth_ddot
            tangent = np.array([math.cos(theta), math.sin(theta), 0.])
            normal = np.array([-math.sin(theta), math.cos(theta), 0.])
            p = self.origin+self.radius*np.array([math.sin(theta), 1.-math.cos(theta), 0.])
            return FlatSample(p, self.radius*w*tangent,
                              self.radius*(alpha*tangent+w*w*normal),
                              self.radius*((beta-w**3)*tangent+3.*w*alpha*normal), self.yaw, 0.)
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
            boundaries = [self.start_delay, self.start_delay+self.ramp]
            if self.mode == 'constant_velocity':
                boundaries += [self.start_delay+self.ramp+self.line_duration,
                               self.start_delay+2.*self.ramp+self.line_duration]
            knots.extend(t for t in boundaries if anchor < t < end)
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
            msg.hold_final = (self.mode == 'constant_velocity' and
                              end >= self.start_delay+2.*self.ramp+self.line_duration)
        check_message(msg)
        self._cached_key, self._cached_message = key, msg
        return msg


class ReferencePublisher(Node):
    """Reference-only node; unspecified origin/yaw wait for valid NED odometry."""
    def __init__(self, *, mode='hover', origin=None, yaw=None, window=4., segment_duration=.5,
                 rate=25., offset=(.6, .3, 0.), speed=.3, radius=.6,
                 start_delay=2., ramp=3., line_duration=8., direction=(1., 0., 0.),
                 node_name='reference_test_publisher'):
        self.config = dict(window=window, segment_duration=segment_duration, rate=rate, offset=offset,
                           speed=speed, radius=radius, start_delay=start_delay,
                           ramp=ramp, line_duration=line_duration, direction=direction)
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


def check_message(msg):
    """Check C3/C1 knots, stationary hold and sampled analytic physical bounds.

    This diagnostic is not a proof of between-sample actuator feasibility; the
    controller independently checks every feedforward input it actually uses.
    Euler discretization residuals are deliberately NOT required to vanish.
    """
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
        for t in np.linspace(0., segment.duration, max(2, math.ceil(segment.duration/.02)+1)):
            ref = flat_reference(evaluate_segment(segment, t))
            if not 1. <= ref.thrust_acceleration <= 12.:
                raise ValueError('generated thrust exceeds [1, 12] m/s^2')
            if np.any(np.abs(ref.body_rates) > RATE_LIMITS):
                raise ValueError('generated body rates exceed [3, 3, 1.5] rad/s')
            maxima = np.maximum(maxima, [ref.thrust_acceleration, *np.abs(ref.body_rates)])
        previous = evaluate_segment(segment, segment.duration)
    if msg.hold_final and any(np.linalg.norm(getattr(previous, name)) > 1e-6
                             for name in ('velocity', 'acceleration', 'jerk', 'yaw_rate')):
        raise ValueError('hold_final requires a stationary endpoint')
    return maxima


def parser():
    cli = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    cli.add_argument('--mode', choices=MODES, default='hover')
    cli.add_argument('--origin', nargs=3, type=float, metavar=('N', 'E', 'D'), help='NED meters; default: first valid odometry')
    cli.add_argument('--yaw', type=float, help='NED yaw radians; default: first valid odometry')
    cli.add_argument('--window', type=float, default=4., help='Polynomial window seconds (>=2); must cover the controller horizon plus transport age')
    cli.add_argument('--segment-duration', type=float, default=.5, help='Hermite piece seconds; default .5')
    cli.add_argument('--rate', type=float, default=25., help='Publishing Hz; must keep reference age below controller timeout')
    cli.add_argument('--offset', nargs=3, type=float, default=(.6, .3, 0.), metavar=('N', 'E', 'D'), help='target_step equilibrium displacement; whole horizon switches after start-delay')
    cli.add_argument('--speed', type=float, default=.3)
    cli.add_argument('--radius', type=float, default=.6)
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
            elif args.mode == 'circle':
                end = trajectory.start_delay+trajectory.ramp+2*math.pi*trajectory.radius/trajectory.speed
            else:
                end = trajectory.start_delay+trajectory.window
            for elapsed in np.arange(0., end+trajectory.segment_duration, trajectory.segment_duration):
                maxima = np.maximum(maxima, check_message(trajectory.message(1_000_000_000, elapsed)))
            if args.mode == 'target_step':
                maxima = np.maximum(maxima, check_message(
                    trajectory.message(1_000_000_000, trajectory.start_delay)))
            print(f'{args.mode}: analytic C3/C1 polynomial windows checked through {end:.3f}s; '
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
