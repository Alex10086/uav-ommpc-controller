#!/usr/bin/env python3
"""SITL-only acceptance flight; not an OMMPC controller.

NED/FRD, Hamilton wxyz body-to-world attitude. Position PD requests world
acceleration; thrust acts along -body-Z. Desired body-Z aligns with g-a,
heading sets body-X, and SO(3) attitude error generates FRD body rates.
Uses the bridge's fixed hover-thrust feedforward (no thrust identification).
Run with sourced ROS/px4_msgs/project overlays and an already running SITL,
XRCE agent and bridge: python3 tools/sitl_flight_test.py --output-dir PATH.
Never changes PX4 parameters, force-arms, or force-disarms. Native PX4 landing
intentionally exits Offboard; landing is not evaluated as external control.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import time
import traceback

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import (VehicleOdometry, VehicleStatus, VehicleCommand,
                          VehicleCommandAck, VehicleLandDetected,
                          VehicleRatesSetpoint, OffboardControlMode)
from uav_ommpc_interfaces.msg import ThrustRates

G = 9.80665


def rotation(q):
    w, x, y, z = np.asarray(q) / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class Flight(Node):
    def __init__(self, out):
        super().__init__('sitl_flight_acceptance')
        self.out = out
        self.started = time.monotonic()
        self.phase = 'initializing'
        self.state = self.status = self.land = None
        self.state_at = self.status_at = self.land_at = -math.inf
        self.last_stamp = None
        self.advances = 0
        self.state_gaps = []
        self.status_gaps = []
        self.acks = []
        self.trace = []
        self.commands = []
        self.holds = {}
        self.counts = dict(state=0, status=0, input=0, rates_output=0, heartbeat_output=0)
        self.input_max = np.zeros(3)
        self.output_max = np.zeros(3)
        self.output_thrust_max = 0.0
        self.observed_offboard = self.observed_armed = False
        self.native_land_seen = False
        self.origin = self.target = None
        self.yaw = 0.0
        self.stream = False
        self.control = False
        self.last_pub = -math.inf
        self.last_trace = -math.inf
        self.max_height = 0.0
        self.result = {'passed': False, 'assumptions': [
            'SITL x500, bridge hover_thrust=0.728744, gravity=9.80665 m/s^2 (see README calibration)',
            'NED position/velocity, FRD rates, body-to-NED wxyz quaternion',
            'Native PX4 NAV_LAND after Offboard exit; no forced disarm'],
            'criteria': {'hold_seconds': 5.0, 'position_error_m': 0.35,
                         'speed_m_s': 0.30, 'tilt_rad': 0.25,
                         'yaw_error_rad': 0.15, 'state_freshness_seconds': 0.5,
                         'status_freshness_seconds': 1.5}}
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE)
        self.input_pub = self.create_publisher(ThrustRates, '/uav/control_input', reliable)
        self.command_pub = self.create_publisher(VehicleCommand, '/uav/vehicle_command', reliable)
        self.subscriptions_kept = [
            self.create_subscription(VehicleOdometry, '/uav/state', self.on_state, sensor),
            self.create_subscription(VehicleStatus, '/uav/status', self.on_status, sensor),
            self.create_subscription(VehicleCommandAck, '/uav/command_ack', self.on_ack, sensor),
            self.create_subscription(VehicleLandDetected, '/fmu/out/vehicle_land_detected', self.on_land, sensor),
            self.create_subscription(VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', self.on_rates, sensor),
            self.create_subscription(OffboardControlMode, '/fmu/in/offboard_control_mode', self.on_heartbeat, sensor)]

    def on_state(self, msg):
        now = time.monotonic()
        values = list(msg.position) + list(msg.velocity) + list(msg.q) + list(msg.angular_velocity)
        if (msg.pose_frame != VehicleOdometry.POSE_FRAME_NED or
                msg.velocity_frame != VehicleOdometry.VELOCITY_FRAME_NED or
                not np.all(np.isfinite(values)) or not 0.9 < np.linalg.norm(msg.q) < 1.1):
            return
        if self.last_stamp is not None and msg.timestamp <= self.last_stamp:
            return
        if self.state is not None:
            self.state_gaps.append(now - self.state_at)
        self.advances += 1
        self.last_stamp = int(msg.timestamp)
        self.state, self.state_at = msg, now
        self.counts['state'] += 1
        if self.origin is not None:
            self.max_height = max(self.max_height, float(self.origin[2] - msg.position[2]))

    def on_status(self, msg):
        now = time.monotonic()
        if self.status is not None:
            self.status_gaps.append(now - self.status_at)
        self.status, self.status_at = msg, now
        self.counts['status'] += 1
        self.observed_offboard |= msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.observed_armed |= msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.native_land_seen |= msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND

    def on_ack(self, msg):
        self.acks.append({'time_s': time.monotonic()-self.started,
                          'command': int(msg.command), 'result': int(msg.result),
                          'target_system': int(msg.target_system),
                          'target_component': int(msg.target_component),
                          'from_external': bool(msg.from_external)})

    def on_land(self, msg):
        self.land, self.land_at = msg, time.monotonic()

    def on_rates(self, msg):
        self.counts['rates_output'] += 1
        self.output_max = np.maximum(self.output_max, np.abs([msg.roll, msg.pitch, msg.yaw]))
        self.output_thrust_max = max(self.output_thrust_max, abs(float(msg.thrust_body[2])))

    def on_heartbeat(self, msg):
        if msg.body_rate and not any((msg.position, msg.velocity, msg.acceleration, msg.attitude)):
            self.counts['heartbeat_output'] += 1

    def set_phase(self, name):
        self.phase = name
        print(f'[{time.monotonic()-self.started:6.1f}s] {name}', flush=True)

    def fresh(self):
        now = time.monotonic()
        # Commander publishes unchanged VehicleStatus at 2 Hz, not odometry rate.
        return self.state is not None and self.status is not None and now-self.state_at < 0.5 and now-self.status_at < 1.5

    def metrics(self):
        p, v = np.array(self.state.position), np.array(self.state.velocity)
        R = rotation(self.state.q)
        yaw = math.atan2(R[1, 0], R[0, 0])
        return (float(np.linalg.norm(p-self.target)), float(np.linalg.norm(v)),
                math.acos(float(np.clip(R[2, 2], -1, 1))),
                abs(math.atan2(math.sin(yaw-self.yaw), math.cos(yaw-self.yaw))))

    def publish_input(self):
        thrust, rates = G, np.zeros(3)
        if self.control and self.fresh():
            p, v = np.array(self.state.position), np.array(self.state.velocity)
            a = np.array([1.2, 1.2, 1.8])*(self.target-p)-np.array([1.8, 1.8, 2.1])*v
            a[:2] = np.clip(a[:2], -1.5, 1.5)
            a[2] = np.clip(a[2], -2.0, 2.0)
            force = np.array([0., 0., G])-a
            bz = force / np.linalg.norm(force)
            heading = np.array([math.cos(self.yaw), math.sin(self.yaw), 0.])
            by = np.cross(bz, heading)
            by /= np.linalg.norm(by)
            desired = np.column_stack((np.cross(by, bz), by, bz))
            R = rotation(self.state.q)
            skew = 0.5*(R.T@desired-desired.T@R)
            rates = np.clip(4.0*np.array([skew[2, 1], skew[0, 2], skew[1, 0]]), -1., 1.)
            thrust = float(np.clip(np.dot(force, R[:, 2]), 0., 14.0))
        msg = ThrustRates()
        msg.thrust_acceleration = thrust
        msg.body_rates = rates.tolist()
        self.input_pub.publish(msg)
        self.counts['input'] += 1
        self.input_max = np.maximum(self.input_max, np.abs(rates))
        return thrust, rates

    def tick(self):
        rclpy.spin_once(self, timeout_sec=0.005)
        now = time.monotonic()
        thrust, rates = math.nan, np.full(3, math.nan)
        if self.stream and now-self.last_pub >= 0.019:
            if self.control and not self.fresh():
                raise RuntimeError(f'Stale telemetry: state age={now-self.state_at:.3f}s, status age={now-self.status_at:.3f}s')
            thrust, rates = self.publish_input()
            self.last_pub = now
        if self.state is not None and now-self.last_trace >= 0.1:
            self.last_trace = now
            self.trace.append([now-self.started, self.phase, int(self.state.timestamp),
                *self.state.position, *self.state.velocity, *self.state.q,
                *(self.target if self.target is not None else [math.nan]*3),
                self.yaw, thrust, *rates,
                int(self.status.nav_state) if self.status else -1,
                int(self.status.arming_state) if self.status else -1])

    def until(self, predicate, timeout, description):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.tick()
            if predicate():
                return
        raise RuntimeError(f'Timeout: {description}')

    def send(self, command, p1=0., p2=0.):
        msg = VehicleCommand()
        msg.command, msg.param1, msg.param2 = int(command), float(p1), float(p2)
        msg.timestamp = int(self.get_clock().now().nanoseconds//1000)
        msg.source_system, msg.source_component = 250, 190
        msg.target_system, msg.target_component = 1, 1
        msg.from_external = True
        self.command_pub.publish(msg)
        self.commands.append({'time_s': time.monotonic()-self.started,
                              'command': int(command), 'param1': p1, 'param2': p2})

    def command(self, code, predicate, p1=0., p2=0., timeout=15.):
        begin = len(self.acks)
        last = -math.inf
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            if time.monotonic()-last > 1.0:
                self.send(code, p1, p2)
                last = time.monotonic()
            self.tick()
            accepted = any(a['command'] == code and a['target_system'] == 250
                           and a['target_component'] == 190 and a['result'] == 0
                           for a in self.acks[begin:])
            if accepted and self.fresh() and predicate():
                return
        raise RuntimeError(f'Command {code}: no matching accepted ACK and status transition')

    def hold(self, name, target, yaw):
        self.set_phase(name)
        self.target, self.yaw = np.array(target), yaw
        deadline = time.monotonic()+40.
        stable_since = None
        samples = []
        last_sample = -math.inf
        while time.monotonic() < deadline:
            self.tick()
            if self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD or self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                raise RuntimeError('Unexpected Offboard/armed exit during hold')
            now = time.monotonic()
            if now-last_sample < 0.05:
                continue
            last_sample = now
            metrics = self.metrics()
            stable = all(x < b for x, b in zip(metrics, (0.35, 0.30, 0.25, 0.15)))
            if stable:
                if stable_since is None:
                    stable_since = now
                    samples = []
                samples.append(metrics)
                if now-stable_since >= 5.0:
                    a = np.array(samples)
                    self.holds[name] = {'duration_s': now-stable_since,
                        'target_ned_m': self.target.tolist(), 'yaw_reference_rad': yaw,
                        'position_error_rms_m': float(np.sqrt(np.mean(a[:, 0]**2))),
                        'position_error_max_m': float(a[:, 0].max()),
                        'speed_max_m_s': float(a[:, 1].max()),
                        'tilt_max_rad': float(a[:, 2].max()),
                        'yaw_error_max_rad': float(a[:, 3].max())}
                    print(json.dumps(self.holds[name]), flush=True)
                    return
            else:
                stable_since = None
        raise RuntimeError(f'{name}: no sustained five-second stable window; final metrics={self.metrics()}')

    def landed(self):
        if not self.fresh() or self.origin is None:
            return False
        disarmed = self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED
        ground = abs(float(self.state.position[2])-self.origin[2]) < 0.35
        slow = np.linalg.norm(self.state.velocity) < 0.2
        detected = self.land is not None and time.monotonic()-self.land_at < 1. and self.land.landed
        return disarmed and ground and slow and (detected or self.native_land_seen)

    def land_safely(self, failure=False):
        self.set_phase('failure_native_landing' if failure else 'native_landing')
        # Stop external streams only after NAV_LAND is accepted / mode changes.
        command_error = None
        try:
            self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND,
                         lambda: self.status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND or self.landed(),
                         timeout=12.)
        except Exception as exc:
            command_error = exc
            self.result["landing_command_error"] = str(exc)
        finally:
            self.stream = self.control = False
        self.until(self.landed, 70., 'native landing and ground disarm')
        self.result['landing'] = {'ground_disarmed': True,
            'native_land_nav_state_observed': self.native_land_seen,
            'land_detector_landed': bool(self.land and self.land.landed),
            'final_position_ned_m': [float(value) for value in self.state.position],
            'final_speed_m_s': float(np.linalg.norm(self.state.velocity)),
            'final_arming_state': int(self.status.arming_state)}
        if command_error is not None and not failure:
            raise command_error

    def run(self):
        self.set_phase('waiting_for_fresh_NED_state_and_status')
        self.until(lambda: self.fresh() and self.advances >= 20, 35., 'fresh advancing NED odometry/status')
        if self.status.arming_state != VehicleStatus.ARMING_STATE_DISARMED:
            raise RuntimeError('Flight test requires initially disarmed vehicle')
        self.origin = np.array(self.state.position)
        self.target = self.origin.copy()
        R = rotation(self.state.q)
        initial_yaw = math.atan2(R[1, 0], R[0, 0])
        self.yaw = initial_yaw
        self.result['initial_position_ned_m'] = self.origin.tolist()
        self.stream = True
        self.set_phase('prestream_hover')
        end = time.monotonic()+2.
        self.until(lambda: time.monotonic() >= end, 4., 'two-second prestream')
        self.set_phase('request_offboard')
        self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                     lambda: self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD, 1., 6.)
        self.set_phase('request_arm')
        self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                     lambda: self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED, 1.)
        self.control = True
        self.set_phase('ramped_takeoff')
        start = time.monotonic()
        while time.monotonic()-start < 5.:
            self.target = self.origin + np.array([0., 0., -min(2., 0.4*(time.monotonic()-start))])
            self.tick()
        first = self.origin + np.array([0., 0., -2.])
        self.hold('hover', first, initial_yaw)
        self.hold('horizontal_displacement_and_yaw', first+np.array([1., 0.5, 0.]), initial_yaw+0.25)
        self.land_safely()
        if not self.observed_offboard or not self.observed_armed or self.max_height < 1.7:
            raise RuntimeError('Missing observed flight status or takeoff height')
        if not np.all(self.input_max > 0.01) or not np.all(self.output_max > 0.005):
            raise RuntimeError('All three nonzero requested and forwarded rate axes were not observed')
        if self.output_thrust_max <= 0.1 or self.counts['heartbeat_output'] == 0:
            raise RuntimeError('Missing meaningful bridge output thrust/body-rate heartbeat observations')
        self.result['passed'] = True

    def save(self):
        self.result.update({'duration_s': time.monotonic()-self.started,
            'last_phase': self.phase, 'holds': self.holds,
            'takeoff_height_max_m': self.max_height,
            'offboard_observed': self.observed_offboard, 'armed_observed': self.observed_armed,
            'topic_counts': self.counts, 'state_timestamp_advances': self.advances,
            'max_state_receive_gap_s': max(self.state_gaps, default=None),
            'max_status_receive_gap_s': max(self.status_gaps, default=None),
            'requested_rates_abs_max_rad_s': self.input_max.tolist(),
            'forwarded_rates_abs_max_rad_s': self.output_max.tolist(),
            'forwarded_normalized_thrust_abs_max': self.output_thrust_max,
            'commands': self.commands, 'command_acks': self.acks})
        self.out.mkdir(parents=True, exist_ok=True)
        with (self.out/'result.json').open('w') as f:
            json.dump(self.result, f, indent=2, allow_nan=False)
        with (self.out/'trace.csv').open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_s', 'phase', 'px4_timestamp_us', 'north_m', 'east_m', 'down_m',
                'vn_m_s', 've_m_s', 'vd_m_s', 'qw', 'qx', 'qy', 'qz',
                'target_n_m', 'target_e_m', 'target_d_m', 'yaw_reference_rad',
                'thrust_acceleration_m_s2', 'roll_rate_rad_s', 'pitch_rate_rad_s',
                'yaw_rate_rad_s', 'nav_state', 'arming_state'])
            writer.writerows(self.trace)
        print(json.dumps(self.result, indent=2), flush=True)
        print(f'Evidence: {self.out}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path,
                        default=Path.home()/'uav_control_ws/flight-test-results')
    args = parser.parse_args()
    rclpy.init()
    flight = Flight(args.output_dir.expanduser())
    try:
        flight.run()
    except (Exception, KeyboardInterrupt) as exc:
        flight.result['error'] = f'{type(exc).__name__}: {exc}'
        traceback.print_exc()
        # Best-effort bounded native landing; never force-disarm even on failure.
        if flight.origin is not None:
            try:
                flight.control = False
                flight.land_safely(failure=True)
            except (Exception, KeyboardInterrupt) as landing_exc:
                flight.result['recovery_error'] = f'{type(landing_exc).__name__}: {landing_exc}'
    finally:
        flight.stream = False
        flight.save()
        flight.destroy_node()
        rclpy.shutdown()
    return 0 if flight.result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
