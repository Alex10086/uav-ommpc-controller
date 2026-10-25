#!/usr/bin/env python3
"""PD + SO(3) baseline controller for SITL comparison; test-only, never real hardware.

Consumes the SAME /uav/reference_polynomial message as ommpc_node and publishes the
same ThrustRates interface, so trajectory, vehicle, limits and initial conditions can
be held identical. Gating mirrors ommpc_node: fresh advancing odometry, healthy
status, receipt lease, estimator-reset/clock-rollback invalidation and sampling at
the odometry receipt time. No arm/mode commands are ever published.

Control law (NED/FRD, thrust along body -Z) follows the repository's existing
PD+SO(3) test controller in tools/sitl_flight_test.py, extended with reference
velocity/acceleration feedforward so it can track a moving polynomial:

  a_cmd  = a_ref + kp*(p_ref-p) + kd*(v_ref-v),  per-axis clipped to max_accel
  force  = g*e3 - a_cmd,  b3 = force/|force|,  b1 from the reference yaw
  rates  = clip(k_rot * vee(R^T R_des - R_des^T R), ±max_body_rates)
  thrust = clip(force . R[:,2], min_thrust, max_thrust)

Thrust is projected onto the CURRENT body z, so a tilted vehicle is not commanded
more force than it can deliver. Defaults reproduce the existing gains
kp=[1.2,1.2,1.8], kd=[1.8,1.8,2.1], k_rot=4.0, rate clamp ±1.0 rad/s; thrust bounds
[1,12] match the OMMPC controller interface for a like-for-like comparison.
"""
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import VehicleOdometry, VehicleStatus
from uav_ommpc_interfaces.msg import PolynomialTrajectory, ThrustRates

sys.path.insert(0, str(Path(__file__).resolve().parent))
from reference_test_publisher import FlatSample, flat_reference, rotation, sample_flat

G = 9.80665


def vee(m):
    return np.array([m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1]]) * 0.5


class PdSo3Node(Node):
    def __init__(self):
        super().__init__('pd_so3_node')
        p = self.declare_parameter
        self.kp = np.array([float(x) for x in p('kp', [1.2, 1.2, 1.8]).value])
        self.kd = np.array([float(x) for x in p('kd', [1.8, 1.8, 2.1]).value])
        self.max_accel = np.array([float(x) for x in p('max_accel', [1.5, 1.5, 2.0]).value])
        self.k_rot = float(p('k_rot', 4.0).value)
        self.min_thrust = float(p('min_thrust', 1.0).value)
        self.max_thrust = float(p('max_thrust', 12.0).value)
        self.max_rates = np.array([float(x) for x in p('max_body_rates', [1.0, 1.0, 1.0]).value])
        self.odometry_timeout = float(p('odometry_timeout', 0.2).value)
        self.status_timeout = float(p('status_timeout', 1.5).value)
        self.reference_timeout = float(p('reference_timeout', 0.5).value)
        self.rate = float(p('rate', 50.0).value)
        self.period_ns = int(round(1e9 / self.rate))

        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(ThrustRates, '/uav/control_input', reliable)
        self.create_subscription(VehicleOdometry, '/uav/state', self.on_odometry, sensor)
        self.create_subscription(VehicleStatus, '/uav/status', self.on_status, sensor)
        self.create_subscription(PolynomialTrajectory, '/uav/reference_polynomial',
                                 self.on_reference, reliable)
        self.create_timer(1.0 / self.rate, self.tick)

        self.state = None
        self.state_at = None
        self.state_ros_ns = 0
        self.last_sample = 0
        self.reset_counter = None
        self.status_at = None
        self.status_ok = False
        self.last_status = 0
        self.plan = None
        self.plan_start_ns = 0
        self.plan_at = None
        self.last_ros_ns = 0
        self.last_warn = 0.0
        self.get_logger().info(
            f'PD+SO(3) baseline ready: kp={self.kp.tolist()} kd={self.kd.tolist()} '
            f'k_rot={self.k_rot} max_accel={self.max_accel.tolist()}; '
            f'limits thrust [{self.min_thrust},{self.max_thrust}] '
            f'rates ±{self.max_rates.tolist()}; no automatic arm or mode change')

    # ---- input gating (mirrors ommpc_node) ----
    def on_odometry(self, msg):
        if msg.timestamp_sample == 0 or msg.timestamp_sample > msg.timestamp:
            self.state_at = None
            return
        if msg.timestamp_sample <= self.last_sample:
            return
        self.last_sample = msg.timestamp_sample
        if self.reset_counter is not None and self.reset_counter != msg.reset_counter:
            self.plan = None
            self.get_logger().warn('Estimator reset: waiting for a new reference')
        self.reset_counter = msg.reset_counter
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED or \
                msg.velocity_frame not in (VehicleOdometry.VELOCITY_FRAME_NED,
                                           VehicleOdometry.VELOCITY_FRAME_BODY_FRD):
            self.state_at = None
            return
        values = np.array([*msg.position, *msg.velocity, *msg.q, *msg.angular_velocity], dtype=float)
        if not np.all(np.isfinite(values)):
            self.state_at = None
            return
        norm = float(np.linalg.norm(msg.q))
        if abs(norm - 1.0) > 1e-3:
            self.state_at = None
            return
        R = rotation(msg.q)
        v = np.array(msg.velocity, dtype=float)
        if msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_BODY_FRD:
            v = R @ v
        self.state = (np.array(msg.position, dtype=float), v, R, np.array(msg.angular_velocity, dtype=float))
        self.state_at = self.now()
        self.state_ros_ns = self.get_clock().now().nanoseconds

    def on_status(self, msg):
        if msg.timestamp == 0:
            self.status_at = None
            return
        if msg.timestamp <= self.last_status:
            return
        self.last_status = msg.timestamp
        self.status_at = self.now()
        self.status_ok = (not msg.failsafe and not msg.in_transition_mode and
                          msg.vehicle_type == VehicleStatus.VEHICLE_TYPE_ROTARY_WING and
                          msg.nav_state != VehicleStatus.NAVIGATION_STATE_TERMINATION)

    def on_reference(self, msg):
        self.plan = None
        start = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if msg.header.frame_id != 'ned' or start <= 0 or not msg.segments:
            return
        self.plan, self.plan_start_ns, self.plan_at = msg, start, self.now()

    @staticmethod
    def now():
        return time.monotonic()

    def fresh(self, at, timeout):
        return at is not None and self.now() - at <= timeout

    def elapsed(self, stamp_ns, tolerance_ns):
        """Nonnegative reference time; a stamp within one period before start maps to zero."""
        if stamp_ns <= 0:
            return None
        if stamp_ns < self.plan_start_ns:
            return 0.0 if self.plan_start_ns - stamp_ns <= tolerance_ns else None
        return (stamp_ns - self.plan_start_ns) * 1e-9

    def anchor(self):
        ros_now = self.get_clock().now().nanoseconds
        if ros_now < self.last_ros_ns:
            self.plan = None
            self.get_logger().warn('ROS clock moved backwards: reference invalidated')
        self.last_ros_ns = ros_now
        if not (self.fresh(self.state_at, self.odometry_timeout) and
                self.fresh(self.status_at, self.status_timeout) and self.status_ok and
                self.plan is not None and self.fresh(self.plan_at, self.reference_timeout)):
            return None
        now_elapsed = self.elapsed(ros_now, self.period_ns)
        anchor_elapsed = self.elapsed(self.state_ros_ns, self.period_ns)
        if now_elapsed is None or anchor_elapsed is None or self.state_ros_ns > ros_now:
            return None
        try:
            sample_flat(self.plan, now_elapsed)
            sample_flat(self.plan, anchor_elapsed)
        except (ValueError, ArithmeticError):
            return None
        return anchor_elapsed

    # ---- control ----
    def tick(self):
        elapsed = self.anchor()
        if elapsed is None:
            return
        try:
            flat = sample_flat(self.plan, elapsed)
        except (ValueError, ArithmeticError):
            return
        p, v, R, _omega = self.state
        # Reference feedforward plus per-axis position/velocity PD, clipped like the
        # repository's existing PD+SO(3) test controller (tools/sitl_flight_test.py).
        a_cmd = flat.acceleration + self.kp * (flat.position - p) + self.kd * (flat.velocity - v)
        a_cmd = np.clip(a_cmd, -self.max_accel, self.max_accel)
        force = np.array([0.0, 0.0, G]) - a_cmd
        norm = float(np.linalg.norm(force))
        if not math.isfinite(norm) or norm < 1e-6:
            return
        bz = force / norm
        heading = np.array([math.cos(flat.yaw), math.sin(flat.yaw), 0.0])
        by = np.cross(bz, heading)
        by_norm = float(np.linalg.norm(by))
        if not math.isfinite(by_norm) or by_norm < 1e-6:
            return
        by = by / by_norm
        R_des = np.column_stack((np.cross(by, bz), by, bz))
        e_R = vee(R.T @ R_des - R_des.T @ R)
        rates = np.clip(self.k_rot * e_R, -self.max_rates, self.max_rates)
        # Project onto the CURRENT body z: a tilted vehicle cannot deliver the full
        # commanded force, so it must not keep commanding it (avoids saturation).
        thrust = float(np.clip(float(force @ R[:, 2]), self.min_thrust, self.max_thrust))
        if not (np.all(np.isfinite(rates)) and math.isfinite(thrust)):
            return
        out = ThrustRates()
        out.thrust_acceleration = thrust
        out.body_rates = [float(rates[0]), float(rates[1]), float(rates[2])]
        if time.monotonic() - self.last_warn >= 1.0:
            self.last_warn = time.monotonic()
            self.get_logger().info(
                f'elapsed={elapsed:.3f} p_err={np.linalg.norm(flat.position - p):.3f} '
                f'v_err={np.linalg.norm(flat.velocity - v):.3f} '
                f'a_cmd={np.round(a_cmd, 3).tolist()} thrust={thrust:.3f} '
                f'|e_R|={np.linalg.norm(e_R):.3f} rates={np.round(rates, 3).tolist()}')
        self.pub.publish(out)


def main():
    rclpy.init()
    try:
        rclpy.spin(PdSo3Node())
    finally:
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
