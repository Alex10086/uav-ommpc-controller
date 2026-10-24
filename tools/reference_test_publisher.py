#!/usr/bin/env python3
"""Publish NED reference horizons only; never arm or change flight modes.

The target_step mode switches entire hover-equilibrium horizons after
start_delay: it is a desired-target step, NOT a dynamically feasible teleport
inside one horizon. The line mode ramps velocity up, cruises for line_duration,
ramps down, then holds; terminal displacement is speed*(line_duration+ramp)
times the normalized direction. Circle starts at origin tangent to +North:
p = origin + radius*[sin(theta), 1-cos(theta), 0]; angular speed ramps from
zero to speed/radius, and cycling continues. All modes preserve initial yaw.
Polynomial trajectories are future work, not a mode. Source the ROS/project
overlays before running, including with --check.
"""
import argparse
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import VehicleOdometry
from uav_ommpc_interfaces.msg import ReferenceState, ReferenceTrajectory, ThrustRates

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


def so3_log(R):
    q = quaternion(R)
    length = float(np.linalg.norm(q[1:]))
    scale = 2.0 if length < 1e-12 else 2.0*math.atan2(length, q[0])/length
    return scale*q[1:]


def so3_exp(value):
    angle = float(np.linalg.norm(value))
    x, y, z = value
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    if angle < 1e-8:
        return np.eye(3) + skew + 0.5*skew@skew
    return np.eye(3) + math.sin(angle)/angle*skew + (1-math.cos(angle))/angle**2*(skew@skew)


def attitude(force, yaw):
    """Align body Z with force while retaining the requested Euler yaw."""
    norm = float(np.linalg.norm(force))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('undefined thrust direction')
    bz = force / norm
    c, s = math.cos(yaw), math.sin(yaw)
    rotated = np.array([c*bz[0]+s*bz[1], -s*bz[0]+c*bz[1], bz[2]])
    roll = math.asin(float(np.clip(-rotated[1], -1.0, 1.0)))
    pitch = math.atan2(rotated[0], rotated[2])
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    return np.array([[c*cp, c*sp*sr-s*cr, c*sp*cr+s*sr],
                     [s*cp, s*sp*sr+c*cr, s*sp*cr-c*sr],
                     [-sp, cp*sr, cp*cr]])


def ramp_integral(value):
    """Integral from 0 to u of quintic smootherstep, clamped to u in [0, 1]."""
    u = np.clip(value, 0.0, 1.0)
    return u**4*(2.5 + u*(-3.0 + u))


class Trajectory:
    """Pure generator. samples is M inputs, with M+1 states (maximum 2000).

    dt must equal the controller dt and M must cover its horizon (default 20).
    The header stamps state zero at now_ns, not at trajectory start or PX4 time.
    """
    def __init__(self, mode='hover', origin=None, yaw=None, dt=.02, samples=60,
                 rate=25., offset=(.6, .3, 0.), speed=.3, radius=.6,
                 start_delay=2., ramp=3., line_duration=8., direction=(1., 0., 0.)):
        if mode not in MODES:
            raise ValueError(f'mode must be one of {MODES}')
        self.mode = mode
        self.origin = vector3((0., 0., 0.) if origin is None else origin, 'origin')
        self.yaw = 0.0 if yaw is None else float(yaw)
        if not math.isfinite(self.yaw):
            raise ValueError('yaw must be finite')
        for name, value in (('dt', dt), ('rate', rate), ('speed', speed),
                            ('radius', radius), ('ramp', ramp), ('line_duration', line_duration)):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
            setattr(self, name, float(value))
        if not math.isfinite(start_delay) or start_delay < 0.0:
            raise ValueError('start_delay must be finite and nonnegative')
        if isinstance(samples, bool) or int(samples) != samples or not 20 <= samples <= 2000:
            raise ValueError('samples must be an integer in [20, 2000] (default controller horizon: 20)')
        self.samples = int(samples)
        self.start_delay = float(start_delay)
        self.offset = vector3(offset, 'offset')
        self.direction = vector3(direction, 'direction')
        length = float(np.linalg.norm(self.direction))
        if not math.isfinite(length) or length < 1e-12:
            raise ValueError('direction must have a finite nonzero norm')
        self.direction /= length

    def positions(self, times):
        times = np.asarray(times, dtype=float)
        travel = np.maximum(times-self.start_delay, 0.0)
        if self.mode == 'constant_velocity':
            # Integrating a quintic velocity ramp gives continuous jerk too.
            acceleration = self.speed*self.ramp*ramp_integral(travel/self.ramp)
            cruise = self.speed*(.5*self.ramp + travel-self.ramp)
            braking_time = travel-self.ramp-self.line_duration
            braking = self.speed*(.5*self.ramp+self.line_duration +
                self.ramp*(np.clip(braking_time/self.ramp, 0.0, 1.0) -
                           ramp_integral(braking_time/self.ramp)))
            distance = np.where(travel < self.ramp, acceleration,
                np.where(travel < self.ramp+self.line_duration, cruise, braking))
            return self.origin + distance[:, None]*self.direction
        if self.mode == 'circle':
            phase_time = np.where(travel < self.ramp,
                self.ramp*ramp_integral(travel/self.ramp), travel-.5*self.ramp)
            angle = (self.speed/self.radius)*phase_time
            displacement = self.radius*np.column_stack((
                np.sin(angle), 1.0-np.cos(angle), np.zeros_like(angle)))
            return self.origin + displacement
        return np.repeat(self.origin[None, :], len(times), axis=0)

    def message(self, now_ns, elapsed_s, samples=None):
        count = self.samples if samples is None else samples
        if isinstance(count, bool) or int(count) != count or not 20 <= count <= 2000:
            raise ValueError('samples must be an integer in [20, 2000]')
        if not math.isfinite(elapsed_s) or elapsed_s < 0.0:
            raise ValueError('elapsed_s must be finite and nonnegative')
        if now_ns < 0 or int(now_ns) != now_ns:
            raise ValueError('now_ns must be a nonnegative integer')
        count = int(count)
        # Two extra future positions yield final velocity/acceleration/attitude.
        p = self.positions(elapsed_s + self.dt*np.arange(count+3))
        if self.mode == 'target_step' and elapsed_s >= self.start_delay:
            p += self.offset
        v = np.diff(p, axis=0)/self.dt
        a = np.diff(v, axis=0)/self.dt
        force = np.array([0., 0., G])-a
        thrust = np.linalg.norm(force, axis=1)
        R = np.array([attitude(f, self.yaw) for f in force])
        rates = np.array([so3_log(R[k].T@R[k+1])/self.dt for k in range(count)])
        if (not np.all(np.isfinite(p)) or not np.all(np.isfinite(v)) or
                not np.all(np.isfinite(thrust)) or not np.all(np.isfinite(rates))):
            raise ValueError('nonfinite generated reference')
        if np.any(thrust < 1.0) or np.any(thrust > 12.0):
            raise ValueError(f'generated thrust [{thrust.min():.6g}, {thrust.max():.6g}] exceeds [1, 12] m/s^2')
        if np.any(np.abs(rates) > RATE_LIMITS):
            raise ValueError(f'generated body rates exceed [3, 3, 1.5] rad/s: {np.max(np.abs(rates), axis=0)}')
        msg = ReferenceTrajectory()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(int(now_ns), 1_000_000_000)
        msg.header.frame_id = 'ned'
        msg.dt = self.dt
        for k in range(count+1):
            state = ReferenceState()
            state.position, state.velocity = p[k].tolist(), v[k].tolist()
            state.q = quaternion(R[k]).tolist()
            msg.states.append(state)
        for k in range(count):
            control = ThrustRates()
            control.thrust_acceleration = float(thrust[k])
            control.body_rates = rates[k].tolist()
            msg.inputs.append(control)
        return msg


class ReferencePublisher(Node):
    """Reference-only node; unspecified origin/yaw wait for valid NED odometry."""
    def __init__(self, *, mode='hover', origin=None, yaw=None, dt=.02, samples=60,
                 rate=25., offset=(.6, .3, 0.), speed=.3, radius=.6,
                 start_delay=2., ramp=3., line_duration=8., direction=(1., 0., 0.),
                 node_name='reference_test_publisher'):
        self.config = dict(dt=dt, samples=samples, rate=rate, offset=offset,
                           speed=speed, radius=radius, start_delay=start_delay,
                           ramp=ramp, line_duration=line_duration, direction=direction)
        Trajectory(mode=mode, origin=origin, yaw=yaw, **self.config)
        super().__init__(node_name)
        self.publish_enabled = True
        self.last_reference = None
        self.generation_error = None
        self.trajectory = None
        self.trajectory_start_ns = None
        self._mode, self._origin, self._yaw = mode, origin, yaw
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE)
        self.reference_pub = self.create_publisher(ReferenceTrajectory, '/uav/reference_trajectory', reliable)
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
            msg = self.trajectory.message(now, elapsed, self.config['samples'])
            self.reference_pub.publish(msg)
            self.last_reference = msg
        except (ValueError, ArithmeticError) as exc:
            self.publish_enabled = False
            self.last_reference = None
            self.generation_error = str(exc)
            self.get_logger().error(f'Reference generation failed; publishing STOPPED: {exc}')


def check_message(msg):
    """Return maximum position, velocity, rotation and quaternion-norm defects."""
    defects = np.zeros(4)
    for k, control in enumerate(msg.inputs):
        left, right = msg.states[k:k+2]
        p, v, R = np.array(left.position), np.array(left.velocity), rotation(left.q)
        next_R = rotation(right.q)
        predicted_v = v + msg.dt*(np.array([0., 0., G])-control.thrust_acceleration*R[:, 2])
        defects = np.maximum(defects, [np.linalg.norm(np.array(right.position)-p-msg.dt*v),
            np.linalg.norm(np.array(right.velocity)-predicted_v),
            np.linalg.norm(so3_log((R@so3_exp(msg.dt*np.array(control.body_rates))).T@next_R)),
            abs(np.linalg.norm(left.q)-1.0)])
    defects[3] = max(defects[3], abs(np.linalg.norm(msg.states[-1].q)-1.0))
    if not np.all(np.isfinite(defects)) or np.any(defects > 1e-6):
        raise ValueError(f'discrete dynamics/quaternion defects exceed 1e-6: {defects}')
    return defects


def parser():
    cli = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    cli.add_argument('--mode', choices=MODES, default='hover')
    cli.add_argument('--origin', nargs=3, type=float, metavar=('N', 'E', 'D'), help='NED meters; default: first valid odometry')
    cli.add_argument('--yaw', type=float, help='NED yaw radians; default: first valid odometry')
    cli.add_argument('--dt', type=float, default=.02, help='Must match controller dt (default .02 s)')
    cli.add_argument('--samples', type=int, default=60, help='M inputs and M+1 states; 20..2000, must cover controller horizon')
    cli.add_argument('--rate', type=float, default=25., help='Publishing Hz; must keep reference age below controller timeout')
    cli.add_argument('--offset', nargs=3, type=float, default=(.6, .3, 0.), metavar=('N', 'E', 'D'), help='target_step equilibrium displacement; whole horizon switches after start-delay')
    cli.add_argument('--speed', type=float, default=.3)
    cli.add_argument('--radius', type=float, default=.6)
    cli.add_argument('--start-delay', type=float, default=2.)
    cli.add_argument('--ramp', type=float, default=3.)
    cli.add_argument('--line-duration', type=float, default=8., help='Constant-speed cruise duration, excluding ramps')
    cli.add_argument('--direction', nargs=3, type=float, default=(1., 0., 0.), help='Normalized NED line direction')
    cli.add_argument('--duration', type=float, help='Stop publishing after this many ROS-clock seconds from trajectory start')
    cli.add_argument('--check', action='store_true', help='Offline generation/dynamics check, no ROS node or publishing; unspecified origin/yaw use zero')
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
            defects = np.zeros(4)
            if duration is not None:
                end = duration
            elif args.mode == 'constant_velocity':
                end = trajectory.start_delay+2*trajectory.ramp+trajectory.line_duration
            elif args.mode == 'circle':
                end = trajectory.start_delay+trajectory.ramp+2*math.pi*trajectory.radius/trajectory.speed
            else:
                end = trajectory.start_delay+trajectory.dt
            # Consecutive blocks cover every dt-grid transition, including ramps
            # and terminal hold. Maximum ROS message bounds still apply.
            steps = math.ceil(end/trajectory.dt)+trajectory.samples
            for first in range(0, steps, 2000):
                count = max(20, min(2000, steps-first))
                elapsed = first*trajectory.dt
                defects = np.maximum(defects, check_message(
                    trajectory.message(1_000_000_000, elapsed, count)))
            if args.mode == 'target_step':
                defects = np.maximum(defects, check_message(
                    trajectory.message(1_000_000_000, trajectory.start_delay)))
            print(f'{args.mode}: feasible across {steps} dt-grid transitions; '
                  f'max p/v/rotation/quaternion defects={defects.tolist()}')
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
