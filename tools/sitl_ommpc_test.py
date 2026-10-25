#!/usr/bin/env python3
"""Operator-confirmed SITL flights through OMMPC reference trajectories only.

Start PX4/Gazebo, XRCE-DDS, the bridge and OMMPC separately, then source the
ROS/project overlays. Example:
  python3 tools/sitl_ommpc_test.py --confirm-sitl --mode hover --record-bag
The operator must ensure this DDS domain contains ONLY the intended SITL.
DDS telemetry alone cannot prove a vehicle is simulated. No parameters are
changed, no preflight checks bypassed, and no control inputs published here.
"""
import argparse
import csv
import json
import math
import signal
import subprocess
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import (VehicleOdometry, VehicleStatus, VehicleCommand,
                          VehicleCommandAck, VehicleLandDetected,
                          VehicleRatesSetpoint, OffboardControlMode)
from uav_ommpc_interfaces.msg import ThrustRates
from reference_test_publisher import ReferencePublisher, VEHICLES, MODES, sample_reference

TOPICS = ('/uav/reference_polynomial', '/uav/state', '/uav/status',
          '/uav/control_input', '/uav/vehicle_command', '/uav/command_ack',
          '/fmu/out/vehicle_odometry', '/fmu/out/vehicle_status_v1',
          '/fmu/out/vehicle_command_ack', '/fmu/out/vehicle_land_detected',
          '/fmu/in/vehicle_rates_setpoint', '/fmu/in/offboard_control_mode',
          '/fmu/in/vehicle_command')
TRACE_FIELDS = ('time_s phase px4_timestamp_us north_m east_m down_m vn_m_s '
                've_m_s vd_m_s qw qx qy qz ref_n_m ref_e_m ref_d_m ref_vn_m_s '
                'ref_ve_m_s ref_vd_m_s ref_qw ref_qx ref_qy ref_qz '
                'reference_stamp_ns reference_elapsed_s position_error_m '
                'velocity_error_m_s speed_m_s tilt_rad yaw_error_rad '
                'input_thrust_m_s2 input_roll_rad_s input_pitch_rad_s input_yaw_rad_s '
                'bridge_roll_rad_s bridge_pitch_rad_s bridge_yaw_rad_s bridge_thrust_z '
                'input_age_s rates_age_s state_age_s status_age_s nav_state arming_state').split()
SOLVE_TRACE_PATH = Path('/tmp/ommpc_solve_trace.csv')


def rotation(q):
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def stamp_ns(stamp):
    return int(stamp.sec)*1_000_000_000 + int(stamp.nanosec)


def finite_or_none(value):
    """JSON must never hide missing observations behind NaN or infinity."""
    if isinstance(value, dict):
        return {k: finite_or_none(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_or_none(v) for v in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    return value


def load_solve_trace():
    if not SOLVE_TRACE_PATH.is_file():
        return []
    rows = []
    try:
        with SOLVE_TRACE_PATH.open('r', newline='') as handle:
            for row in csv.DictReader(handle):
                try:
                    rows.append({
                        'steady_s': float(row['steady_s']),
                        'solve_us': float(row['solve_us']),
                        'step_us': float(row['step_us']),
                        'iterations': int(row['iterations']),
                        'status': int(row['status']),
                        'solver_status': int(row['solver_status']),
                    })
                except (KeyError, TypeError, ValueError):
                    continue
    except OSError:
        return []
    return rows


class Flight(ReferencePublisher):
    def __init__(self, out, modes, vehicle='x500', max_thrust=None,
                 abort_tilt_rad=0.6, max_pos_rmse=0.35, max_vel_rmse=0.30,
                 speed_override=None, radius_override=None, z_amp_override=None,
                 loops_override=None, ramp_override=None):
        self.f_vehicle = vehicle
        self.f_max_thrust = float(VEHICLES[vehicle]['max_thrust'] if max_thrust is None else max_thrust)
        self.f_abort_tilt_rad = float(abort_tilt_rad)
        self.f_max_pos_rmse = float(max_pos_rmse)
        self.f_max_vel_rmse = float(max_vel_rmse)
        self.f_speed_override = speed_override
        self.f_radius_override = radius_override
        self.f_z_amp_override = z_amp_override
        self.f_loops_override = loops_override
        self.f_ramp_override = ramp_override
        super().__init__(mode='hover', origin=None, yaw=None, window=4., segment_duration=.5,
                         rate=25., offset=(.6, .3, 0.), speed=.3, radius=.6, z_amp=.35, loops=None,
                         start_delay=2., ramp=3., line_duration=8.,
                         direction=(1., 0., 0.), max_thrust=self.f_max_thrust,
                         node_name='sitl_ommpc_test')
        self.publish_enabled = False
        self.f_out, self.f_modes = out, modes
        self.f_started = time.monotonic()
        self.f_phase = 'initializing'
        self.f_state = self.f_status = self.f_land = None
        self.f_state_at = self.f_status_at = self.f_land_at = -math.inf
        self.f_state_ros_ns = None
        self.f_input = self.f_rates = None
        self.f_input_at = self.f_rates_at = -math.inf
        self.f_raw_state_at = self.f_raw_status_at = -math.inf
        self.f_last_state_stamp = self.f_raw_stamp = -1
        self.f_last_status_stamp = self.f_raw_status_stamp = -1
        self.f_last_land_stamp = -1
        self.f_advances = self.f_raw_advances = 0
        self.f_active = self.f_landing = self.f_engaged = False
        self.f_native_land_seen = False
        self.f_ground = self.f_desired = None
        self.f_yaw = 0.
        self.f_last_reference = None
        self.f_acks, self.f_commands, self.f_rows = [], [], []
        self.f_solve_rows = []
        self.f_phase_info = {}
        self.f_counts = {k: 0 for k in ('state', 'status', 'raw_state', 'raw_status',
                                      'land', 'control', 'rates', 'heartbeat', 'reference')}
        self.f_events = []
        self.f_bound_violations = 0
        self.f_max_height = 0.
        self.f_result = {'passed': False, 'modes_requested': modes,
                         'vehicle': vehicle,
                         'max_thrust_m_s2': self.f_max_thrust,
                         'sitl_confirmation': 'explicit operator flag; DDS does not establish model identity',
                         'criteria': {'settle_window_s': 5., 'position_m': .35,
                                      'speed_m_s': .3, 'tilt_rad': .25, 'yaw_rad': .2,
                                      'moving_position_rmse_m': self.f_max_pos_rmse,
                                      'moving_velocity_rmse_m_s': self.f_max_vel_rmse,
                                      'tracking_position_max_m': 1.,
                                      'abort_position_m': 2., 'abort_tilt_rad': self.f_abort_tilt_rad,
                                      'state_freshness_s': .2, 'status_freshness_s': 1.5,
                                      'control_freshness_s': .3},
                         'control_source': 'OMMPC; this process publishes references and vehicle commands only'}
        reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        sensor = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE)
        self.f_command_pub = self.create_publisher(VehicleCommand, '/uav/vehicle_command', reliable)
        self.f_subs = [
            self.create_subscription(VehicleOdometry, '/uav/state', self.f_on_state, sensor),
            self.create_subscription(VehicleStatus, '/uav/status', self.f_on_status, sensor),
            self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.f_on_raw_state, sensor),
            self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.f_on_raw_status, sensor),
            self.create_subscription(VehicleCommandAck, '/uav/command_ack', self.f_on_ack, sensor),
            self.create_subscription(VehicleLandDetected, '/fmu/out/vehicle_land_detected', self.f_on_land, sensor),
            self.create_subscription(ThrustRates, '/uav/control_input', self.f_on_input, reliable),
            self.create_subscription(VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', self.f_on_rates, sensor),
            self.create_subscription(OffboardControlMode, '/fmu/in/offboard_control_mode', self.f_on_heartbeat, sensor)]
        self.f_files = []
        self.f_trace_writer = self.f_csv('trace.csv', TRACE_FIELDS)
        self.f_input_writer = self.f_csv('control_input.csv', ['time_s', 'phase', 'thrust_acceleration', 'roll', 'pitch', 'yaw', 'gap_s', 'bounds_valid'])
        self.f_rates_writer = self.f_csv('bridge_rates.csv', ['time_s', 'phase', 'timestamp_us', 'roll', 'pitch', 'yaw', 'thrust_x', 'thrust_y', 'thrust_z', 'gap_s', 'bounds_valid'])
        self.f_ref_writer = self.f_csv('reference.csv', ['time_s', 'phase', 'stamp_ns', 'elapsed_s', 'north', 'east', 'down', 'vn', 've', 'vd', 'qw', 'qx', 'qy', 'qz', 'thrust', 'roll', 'pitch', 'yaw'])

    def f_csv(self, name, fields):
        file = (self.f_out/name).open('w', newline='')
        self.f_files.append(file)
        writer = csv.writer(file)
        writer.writerow(fields)
        return writer

    def f_time(self):
        return time.monotonic()-self.f_started

    def f_event(self, kind, **values):
        self.f_events.append({'time_s': self.f_time(), 'phase': self.f_phase, 'kind': kind, **values})

    @staticmethod
    def f_valid_state(msg):
        return (msg.pose_frame == VehicleOdometry.POSE_FRAME_NED and
                msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_NED and
                np.all(np.isfinite([*msg.position, *msg.velocity, *msg.q])) and
                .9 < np.linalg.norm(msg.q) < 1.1)

    def f_on_state(self, msg):
        if not self.f_valid_state(msg) or msg.timestamp <= self.f_last_state_stamp:
            return
        self.f_last_state_stamp = int(msg.timestamp)
        self.f_state, self.f_state_at = msg, time.monotonic()
        self.f_state_ros_ns = self.get_clock().now().nanoseconds
        self.f_counts['state'] += 1
        self.f_advances += 1
        if self.f_ground is not None:
            self.f_max_height = max(self.f_max_height, float(self.f_ground[2]-msg.position[2]))
        self.f_record_state()

    def f_on_raw_state(self, msg):
        if self.f_valid_state(msg) and msg.timestamp > self.f_raw_stamp:
            self.f_raw_stamp = int(msg.timestamp)
            self.f_raw_state_at = time.monotonic()
            self.f_raw_advances += 1
            self.f_counts['raw_state'] += 1

    def f_on_status(self, msg):
        if msg.timestamp <= self.f_last_status_stamp:
            return
        self.f_last_status_stamp = int(msg.timestamp)
        self.f_status, self.f_status_at = msg, time.monotonic()
        self.f_counts['status'] += 1
        if self.f_landing and msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            self.f_native_land_seen = True
            self.publish_enabled = False

    def f_on_raw_status(self, msg):
        if msg.timestamp > self.f_raw_status_stamp:
            self.f_raw_status_stamp = int(msg.timestamp)
            self.f_raw_status_at = time.monotonic()
            self.f_counts['raw_status'] += 1

    def f_on_land(self, msg):
        if msg.timestamp > self.f_last_land_stamp:
            self.f_last_land_stamp = int(msg.timestamp)
            self.f_land, self.f_land_at = msg, time.monotonic()
            self.f_counts['land'] += 1

    def f_on_ack(self, msg):
        self.f_acks.append({'time_s': self.f_time(), 'timestamp_us': int(msg.timestamp),
                            'command': int(msg.command), 'result': int(msg.result),
                            'target_system': int(msg.target_system),
                            'target_component': int(msg.target_component),
                            'from_external': bool(msg.from_external)})

    def f_on_input(self, msg):
        now = time.monotonic()
        values = np.array([msg.thrust_acceleration, *msg.body_rates], dtype=float)
        valid = bool(np.all(np.isfinite(values)) and 1.-1e-4 <= values[0] <= self.f_max_thrust+1e-4 and
                     np.all(np.abs(values[1:]) <= np.array([3., 3., 1.5])+1e-4))
        gap = now-self.f_input_at
        self.f_input_writer.writerow([self.f_time(), self.f_phase, *values, gap if math.isfinite(gap) else '', valid])
        self.f_input, self.f_input_at = values, now
        self.f_counts['control'] += 1
        self.f_event('control', gap_s=gap, bounds_valid=valid)
        self.f_bound_violations += int(not valid)

    def f_on_rates(self, msg):
        now = time.monotonic()
        values = np.array([msg.roll, msg.pitch, msg.yaw, *msg.thrust_body], dtype=float)
        valid = bool(np.all(np.isfinite(values)) and
                     np.all(np.abs(values[:3]) <= np.array([3., 3., 1.5])+1e-6) and
                     np.all(np.abs(values[3:5]) <= 1e-6) and -.9-1e-6 <= values[5] <= 1e-6)
        gap = now-self.f_rates_at
        self.f_rates_writer.writerow([self.f_time(), self.f_phase, int(msg.timestamp), *values, gap if math.isfinite(gap) else '', valid])
        self.f_rates, self.f_rates_at = values, now
        self.f_counts['rates'] += 1
        self.f_event('rates', gap_s=gap, bounds_valid=valid)
        self.f_bound_violations += int(not valid)

    def f_on_heartbeat(self, msg):
        if msg.body_rate and not any((msg.position, msg.velocity, msg.acceleration, msg.attitude)):
            self.f_counts['heartbeat'] += 1

    def f_reference(self, receipt_ns=None):
        ref = self.last_reference
        if ref is None:
            return None
        now_ns = self.get_clock().now().nanoseconds if receipt_ns is None else receipt_ns
        elapsed = (now_ns-stamp_ns(ref.header.stamp))*1e-9
        try:
            desired = sample_reference(ref, elapsed)
        except (ValueError, ArithmeticError):
            return None
        return desired, elapsed, stamp_ns(ref.header.stamp)

    def f_record_reference(self):
        ref = self.last_reference
        if ref is None or self.last_publish_ns == self.f_last_reference:
            return
        self.f_last_reference = self.last_publish_ns
        self.f_counts['reference'] += 1
        elapsed = (self.last_publish_ns-stamp_ns(ref.header.stamp))*1e-9
        desired = sample_reference(ref, elapsed)
        self.f_ref_writer.writerow([self.f_time(), self.f_phase, stamp_ns(ref.header.stamp), elapsed,
                                   *desired.position, *desired.velocity, *desired.q,
                                   desired.thrust_acceleration, *desired.body_rates])

    def f_record_state(self):
        selected = self.f_reference(self.f_state_ros_ns)
        if selected is None:
            return
        desired, elapsed, reference_stamp = selected
        state = self.f_state
        R, Rd = rotation(state.q), rotation(desired.q)
        yaw = math.atan2(R[1, 0], R[0, 0])
        yaw_desired = math.atan2(Rd[1, 0], Rd[0, 0])
        error = float(np.linalg.norm(np.array(state.position)-desired.position))
        velocity_error = float(np.linalg.norm(np.array(state.velocity)-desired.velocity))
        speed = float(np.linalg.norm(state.velocity))
        tilt = math.acos(float(np.clip(R[2, 2], -1., 1.)))
        yaw_error = abs(math.atan2(math.sin(yaw-yaw_desired), math.cos(yaw-yaw_desired)))
        now = time.monotonic()
        control = self.f_input if self.f_input is not None else [math.nan]*4
        rates = self.f_rates if self.f_rates is not None else [math.nan]*6
        row = [self.f_time(), self.f_phase, int(state.timestamp), *state.position, *state.velocity, *state.q,
               *desired.position, *desired.velocity, *desired.q, reference_stamp, elapsed,
               error, velocity_error, speed, tilt, yaw_error, *control, *rates[:3], rates[5],
               now-self.f_input_at, now-self.f_rates_at, now-self.f_state_at, now-self.f_status_at,
               int(self.f_status.nav_state) if self.f_status else -1,
               int(self.f_status.arming_state) if self.f_status else -1]
        self.f_rows.append(dict(zip(TRACE_FIELDS, row)))
        self.f_trace_writer.writerow(row)

    def f_fresh(self):
        now = time.monotonic()
        c1 = self.f_state is not None
        c2 = self.f_status is not None
        c3 = now-self.f_state_at < .2
        c4 = now-self.f_status_at < 1.5
        c5 = now-self.f_raw_state_at < .2
        c6 = now-self.f_raw_status_at < 1.5
        if not (c1 and c2 and c3 and c4 and c5 and c6):
            if self.f_active and not self.f_landing:
                print(f"FRESH_FAIL: state_age={now-self.f_state_at:.4f} status_age={now-self.f_status_at:.4f} raw_state_age={now-self.f_raw_state_at:.4f} raw_status_age={now-self.f_raw_status_at:.4f}", flush=True)
            return False
        return True

    def f_tick(self):
        rclpy.spin_once(self, timeout_sec=.003)
        for _ in range(15):
            rclpy.spin_once(self, timeout_sec=0.0)
        self.f_record_reference()
        if not self.f_active or self.f_landing:
            return
        if not self.f_fresh():
            raise RuntimeError('Stale bridge or actual /fmu telemetry while controlling')
        if (self.f_status.arming_state != VehicleStatus.ARMING_STATE_ARMED or
                self.f_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD):
            raise RuntimeError('Unexpected Offboard/armed exit')
        now = time.monotonic()
        if now-self.f_input_at > .3 or now-self.f_rates_at > .3:
            raise RuntimeError('OMMPC command/bridge rate gap exceeds 0.3 seconds')
        selected = self.f_reference()
        if selected is None:
            raise RuntimeError('No current reference sample at ROS receipt time')
        if np.linalg.norm(np.array(self.f_state.position)-selected[0].position) > 2.:
            raise RuntimeError('Position error exceeds 2 m')
        if math.acos(float(np.clip(rotation(self.f_state.q)[2, 2], -1., 1.))) > self.f_abort_tilt_rad:
            raise RuntimeError(f'Vehicle tilt exceeds {self.f_abort_tilt_rad:g} rad')
        if self.f_bound_violations:
            raise RuntimeError('Observed controller/bridge input bound violation')

    def f_until(self, predicate, timeout, description):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            self.f_tick()
            if predicate():
                return
        raise RuntimeError('Timeout: '+description)

    def f_phase_start(self, name):
        if self.f_phase in self.f_phase_info:
            self.f_phase_info[self.f_phase]['end_s'] = self.f_time()
            self.f_phase_info[self.f_phase]['counts_end'] = self.f_counts.copy()
        self.f_phase = name
        self.f_phase_info[name] = {'start_s': self.f_time(), 'completed': False,
                                   'counts_start': self.f_counts.copy()}
        print(f'[{self.f_time():7.2f}s] {name}', flush=True)
        for file in self.f_files:
            file.flush()

    def f_phase_done(self):
        self.f_phase_info[self.f_phase]['completed'] = True
        self.f_phase_info[self.f_phase]['end_s'] = self.f_time()
        self.f_phase_info[self.f_phase]['counts_end'] = self.f_counts.copy()
        print(json.dumps(self.f_metrics(self.f_phase)), flush=True)

    def f_send(self, command, p1=0., p2=0.):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds//1000
        msg.command, msg.param1, msg.param2 = int(command), float(p1), float(p2)
        msg.source_system, msg.source_component = 250, 190
        msg.target_system, msg.target_component = 1, 1
        msg.from_external = True
        self.f_command_pub.publish(msg)
        self.f_commands.append({'time_s': self.f_time(), 'command': int(command), 'param1': p1, 'param2': p2})

    def f_command(self, code, predicate, p1=0., p2=0., timeout=15.):
        start = len(self.f_acks)
        begin_time = self.f_time()
        last_send = -math.inf
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            if time.monotonic()-last_send >= 1.:
                self.f_send(code, p1, p2)
                last_send = time.monotonic()
            self.f_tick()
            matching = [a for a in self.f_acks[start:] if a['command'] == code and
                        a['target_system'] == 250 and a['target_component'] == 190 and
                        a['time_s'] >= begin_time]
            if any(a['result'] == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED for a in matching):
                if self.f_fresh() and predicate():
                    return
            permanent = [a for a in matching if a['result'] in
                         (VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED,
                          VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED,
                          VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED,
                          VehicleCommandAck.VEHICLE_CMD_RESULT_CANCELLED)]
            if permanent:
                raise RuntimeError(f'Command {code} rejected: {permanent[-1]}')
        raise RuntimeError(f'Command {code}: no matching accepted ACK and fresh state transition within {timeout}s')

    def f_set_hover(self, target):
        self.f_desired = np.asarray(target, dtype=float).copy()
        saved_loops = self.config.get('loops')
        self.config['loops'] = None
        self.set_trajectory('hover', self.f_desired, self.f_yaw)
        self.config['loops'] = saved_loops

    def f_settle(self, target, timeout=45.):
        self.f_set_hover(target)
        stable_since = None
        deadline = time.monotonic()+timeout
        last_stamp = -1
        while time.monotonic() < deadline:
            self.f_tick()
            if self.f_state.timestamp == last_stamp:
                continue
            last_stamp = self.f_state.timestamp
            R = rotation(self.f_state.q)
            yaw = math.atan2(R[1, 0], R[0, 0])
            metrics = (np.linalg.norm(np.array(self.f_state.position)-target),
                       np.linalg.norm(self.f_state.velocity),
                       math.acos(float(np.clip(R[2, 2], -1., 1.))),
                       abs(math.atan2(math.sin(yaw-self.f_yaw), math.cos(yaw-self.f_yaw))))
            stable = all(v <= limit for v, limit in zip(metrics, (.35, .3, .25, .2)))
            if stable:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic()-stable_since >= 5.:
                    self.f_phase_info[self.f_phase]['stable_window_s'] = time.monotonic()-stable_since
                    return
            else:
                stable_since = None
        raise RuntimeError(f'No five-second stable hover at {list(target)}')

    def f_mode(self, mode):
        self.f_phase_start(mode)
        origin = self.f_desired.copy()
        if mode == 'hover':
            self.f_settle(origin)
        elif mode == 'target_step':
            self.set_trajectory(mode, origin, self.f_yaw)
            end = time.monotonic()+3.
            self.f_until(lambda: time.monotonic() >= end, 5., 'target-step delay and transition')
            self.f_settle(origin+np.array([.6, .3, 0.]))
        elif mode == 'constant_velocity':
            self.config['speed'] = .3 if self.f_speed_override is None else float(self.f_speed_override)
            self.set_trajectory(mode, origin, self.f_yaw)
            duration = self.config['start_delay']+self.config['line_duration']+2.*self.config['ramp']
            end = time.monotonic()+duration+.1
            self.f_until(lambda: time.monotonic() >= end, duration+3., 'ramped finite straight-line trajectory')
            selected = self.f_reference()
            if selected is None:
                raise RuntimeError('Missing terminal line reference')
            terminal = np.array(selected[0].position)
            self.f_settle(terminal)
        elif mode in ('circle', 'figure8', 'helix', 'scurve'):
            if self.f_speed_override is not None:
                self.config['speed'] = float(self.f_speed_override)
            elif mode == 'circle':
                self.config['speed'] = .25
            else:
                self.config['speed'] = 1.2
            if self.f_radius_override is not None:
                self.config['radius'] = float(self.f_radius_override)
            elif mode in ('figure8', 'helix', 'scurve'):
                self.config['radius'] = 1.2
            if self.f_z_amp_override is not None:
                self.config['z_amp'] = float(self.f_z_amp_override)
            if self.f_ramp_override is not None:
                self.config['ramp'] = float(self.f_ramp_override)
            use_loops = self.f_loops_override
            if use_loops is None and (mode in ('figure8', 'helix', 'scurve') or self.config['speed'] > 0.4):
                use_loops = 2.0
            self.config['loops'] = use_loops
            self.set_trajectory(mode, origin, self.f_yaw)
            if self.config['loops'] is not None:
                cruise = self.trajectory._curve_cruise_duration()
                duration = self.config['start_delay'] + 2.0*self.config['ramp'] + cruise
                end = time.monotonic() + duration + 0.5
                self.f_until(lambda: time.monotonic() >= end, duration + 4., f'smooth {mode} trajectory')
                terminal = np.array(self.trajectory.flat(duration + 1.0).position)
                self.f_settle(terminal)
            else:
                duration = 2.+3.+max(18., 2.*math.pi*self.config['radius']/self.config['speed'])
                end = time.monotonic()+duration
                self.f_until(lambda: time.monotonic() >= end, duration+3., 'one complete smooth circle')
                selected = self.f_reference()
                if selected is None:
                    raise RuntimeError('Missing circle reference at completion')
                self.f_settle(np.array(selected[0].position))
        elif mode == 'sharp3d':
            if self.f_speed_override is not None:
                self.config['speed'] = float(self.f_speed_override)
            else:
                self.config['speed'] = 3.6
            if self.f_radius_override is not None:
                self.config['radius'] = float(self.f_radius_override)
            else:
                self.config['radius'] = 1.6
            if self.f_z_amp_override is not None:
                self.config['z_amp'] = float(self.f_z_amp_override)
            else:
                self.config['z_amp'] = 0.60
            self.set_trajectory(mode, origin, self.f_yaw)
            T_leg, _, end_pt = self.trajectory._sharp3d_setup()
            duration = self.config['start_delay'] + 4.0 * T_leg
            end = time.monotonic() + duration + 0.5
            self.f_until(lambda: time.monotonic() >= end, duration + 4., 'smooth 4-leg 3D sharp turn trajectory')
            self.f_settle(np.array(end_pt))
        elif mode == 'hover_to_hover':
            self.set_trajectory(mode, origin, self.f_yaw)
            dist = float(np.linalg.norm(self.config['offset']))
            duration = self.config['start_delay'] + (35.0/16.0)*dist/self.config['speed']
            end = time.monotonic()+duration+.5
            self.f_until(lambda: time.monotonic() >= end, duration+4., 'smooth degree-7 hover-to-hover trajectory')
            terminal = origin+self.config['offset']
            self.f_settle(terminal)
        metrics = self.f_metrics(mode)
        if metrics['position_error_max'] is None or metrics['position_error_max'] > 1.:
            raise RuntimeError(f'{mode}: maximum tracking error exceeds 1 m')
        if mode in ('constant_velocity', 'circle', 'figure8', 'helix', 'scurve', 'sharp3d', 'hover_to_hover'):
            if metrics['position_error_rmse'] > self.f_max_pos_rmse or metrics['velocity_error_rmse'] > self.f_max_vel_rmse:
                raise RuntimeError(f'{mode}: moving tracking RMSE exceeds position/velocity limits')
        self.f_phase_done()

    def f_landed(self):
        now = time.monotonic()
        return (self.f_fresh() and self.f_land is not None and now-self.f_land_at < 1. and
                bool(self.f_land.landed) and
                self.f_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED and
                np.linalg.norm(self.f_state.velocity) < .2)

    def f_land_safely(self, failure=False):
        self.f_landing = True
        self.f_active = False
        self.f_phase_start('failure_native_landing' if failure else 'native_landing')
        command_error = None
        try:
            self.f_command(VehicleCommand.VEHICLE_CMD_NAV_LAND,
                           lambda: self.f_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND or self.f_landed(),
                           timeout=12.)
        except Exception as exc:
            command_error = str(exc)
            self.f_result['landing_command_error'] = command_error
        # Never stop references solely because the command timed out: an observed
        # fresh AUTO_LAND transition (callback) is the handover condition.
        if self.f_status is not None and self.f_fresh() and self.f_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            self.f_native_land_seen = True
            self.publish_enabled = False
        self.f_until(self.f_landed, 75., 'fresh landed detector AND disarmed status after native landing')
        self.publish_enabled = False
        self.f_result['landing'] = {'landed_and_disarmed': True,
                                     'native_land_observed': self.f_native_land_seen,
                                     'position_ned_m': list(self.f_state.position),
                                     'final_speed_m_s': float(np.linalg.norm(self.f_state.velocity)),
                                     'command_ack_succeeded': command_error is None}
        self.f_phase_done()
        if command_error is not None and not failure:
            raise RuntimeError(command_error)

    def f_run(self):
        self.f_phase_start('waiting_for_actual_DDS_and_bridge')
        self.f_until(lambda: self.f_fresh() and self.f_advances >= 20 and self.f_raw_advances >= 20,
                     35., 'advancing actual DDS and bridge NED telemetry')
        if self.f_status.arming_state != VehicleStatus.ARMING_STATE_DISARMED:
            raise RuntimeError('Refusing flight: vehicle must initially be DISARMED')
        self.f_until(lambda: self.f_land is not None and time.monotonic()-self.f_land_at < 1., 10., 'fresh initial land detector')
        if not self.f_land.landed or np.linalg.norm(self.f_state.velocity) > .2:
            raise RuntimeError('Refusing flight: initial vehicle is not stationary and landed')
        self.f_ground = np.array(self.f_state.position, dtype=float)
        R = rotation(self.f_state.q)
        self.f_yaw = math.atan2(R[1, 0], R[0, 0])
        self.f_result['initial_position_ned_m'] = self.f_ground.tolist()
        self.f_result['initial_yaw_rad'] = self.f_yaw
        self.f_set_hover(self.f_ground)
        self.publish_enabled = True
        self.f_phase_done()
        self.f_phase_start('prestream_ground_hover')
        end = time.monotonic()+2.5
        self.f_until(lambda: time.monotonic() >= end, 5., 'at least two-second reference prestream')
        if not self.f_fresh() or time.monotonic()-self.f_input_at > .3 or self.f_counts['heartbeat'] < 10:
            raise RuntimeError('Missing fresh OMMPC output or body-rate bridge heartbeat before arming')
        self.f_phase_done()
        self.f_phase_start('request_offboard')
        self.f_engaged = True
        self.f_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                       lambda: self.f_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD, 1., 6.)
        self.f_phase_done()
        self.f_phase_start('request_arm')
        self.f_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                       lambda: self.f_status.arming_state == VehicleStatus.ARMING_STATE_ARMED, 1.)
        self.f_active = True
        self.f_phase_done()
        self.f_phase_start('ramped_takeoff')
        airborne = self.f_ground+np.array([0., 0., -2.])
        saved_offset = np.asarray(self.config['offset'], dtype=float).copy()
        saved_speed = float(self.config['speed'])
        saved_delay = float(self.config['start_delay'])
        saved_loops = self.config.get('loops')
        self.config['offset'] = np.array([0., 0., -2.])
        self.config['speed'] = 0.4
        self.config['start_delay'] = 0.5
        self.config['loops'] = None
        self.set_trajectory('hover_to_hover', self.f_ground, self.f_yaw)
        takeoff_duration = self.config['start_delay'] + (35.0/16.0)*2.0/0.4
        end = time.monotonic()+takeoff_duration+.5
        self.f_until(lambda: time.monotonic() >= end, takeoff_duration+5., 'smooth degree-7 polynomial takeoff')
        self.config['offset'] = saved_offset
        self.config['speed'] = saved_speed
        self.config['start_delay'] = saved_delay
        self.config['loops'] = saved_loops
        self.f_settle(airborne)
        self.f_phase_done()
        for mode in self.f_modes:
            self.f_mode(mode)
        self.f_land_safely()
        if self.f_max_height < 1.7:
            raise RuntimeError('No actual 1.7 m takeoff observed')
        if any(not e['bounds_valid'] for e in self.f_events if 'bounds_valid' in e):
            raise RuntimeError('Input bound violation was observed; see phase metrics')
        self.f_result['passed'] = True

    def f_metrics(self, phase=None):
        rows = [r for r in self.f_rows if phase is None or r['phase'] == phase]
        events = [e for e in self.f_events if phase is None or e['phase'] == phase]
        result = {'phase': phase or 'all', 'telemetry_samples': len(rows)}
        for field, prefix in [('position_error_m', 'position_error'), ('velocity_error_m_s', 'velocity_error')]:
            values = [r[field] for r in rows if math.isfinite(r[field])]
            result[prefix+'_rmse'] = float(np.sqrt(np.mean(np.square(values)))) if values else None
            result[prefix+'_max'] = max(values, default=None)
        for field in ('speed_m_s', 'tilt_rad', 'yaw_error_rad'):
            result[field+'_max'] = max((r[field] for r in rows), default=None)
        ref_speeds = [math.sqrt(r['ref_vn_m_s']**2 + r['ref_ve_m_s']**2 + r['ref_vd_m_s']**2)
                      for r in rows if all(math.isfinite(r[k]) for k in ('ref_vn_m_s', 'ref_ve_m_s', 'ref_vd_m_s'))]
        result['ref_speed_m_s_max'] = max(ref_speeds, default=None)
        for field in ('input_thrust_m_s2', 'input_roll_rad_s', 'input_pitch_rad_s',
                      'input_yaw_rad_s', 'bridge_roll_rad_s', 'bridge_pitch_rad_s',
                      'bridge_yaw_rad_s', 'bridge_thrust_z'):
            values = [abs(r[field]) for r in rows if math.isfinite(r[field])]
            result[field+'_abs_max'] = max(values, default=None)
        thrust_vals = [r['input_thrust_m_s2'] for r in rows if math.isfinite(r['input_thrust_m_s2'])]
        result['input_thrust_m_s2_min'] = min(thrust_vals, default=None)
        result['input_thrust_m_s2_max'] = max(thrust_vals, default=None)
        # Moving-only tracking, along/cross-track decomposition, and optimal phase lag
        result['moving_only_position_rmse'] = None
        result['along_track_lag_m_mean'] = None
        result['along_track_error_rmse'] = None
        result['cross_track_error_rmse'] = None
        result['tracking_lag_ms'] = None
        result['lag_compensated_position_rmse'] = None
        if len(rows) >= 10:
            t_arr = np.array([r['time_s'] for r in rows], dtype=float)
            p_arr = np.array([[r['north_m'], r['east_m'], r['down_m']] for r in rows], dtype=float)
            pr_arr = np.array([[r['ref_n_m'], r['ref_e_m'], r['ref_d_m']] for r in rows], dtype=float)
            vr_arr = np.array([[r['ref_vn_m_s'], r['ref_ve_m_s'], r['ref_vd_m_s']] for r in rows], dtype=float)
            vr_norm = np.linalg.norm(vr_arr, axis=1)
            moving = vr_norm > 0.05
            if int(np.sum(moving)) >= 10 and (t_arr[-1] - t_arr[0]) > 0.5:
                p_mov = p_arr[moving]
                pr_mov = pr_arr[moving]
                vr_mov = vr_arr[moving]
                vn_mov = vr_norm[moving]
                t_mov = t_arr[moving]
                err_vec = pr_mov - p_mov  # positive along velocity => actual lags behind reference
                t_hat = vr_mov / vn_mov[:, None]
                along = np.sum(err_vec * t_hat, axis=1)
                cross_vec = err_vec - along[:, None] * t_hat
                result['moving_only_position_rmse'] = float(np.sqrt(np.mean(np.sum(err_vec**2, axis=1))))
                result['along_track_lag_m_mean'] = float(np.mean(along))
                result['along_track_error_rmse'] = float(np.sqrt(np.mean(along**2)))
                result['cross_track_error_rmse'] = float(np.sqrt(np.mean(np.sum(cross_vec**2, axis=1))))
                shifts = np.linspace(-0.35, 0.35, 701)
                best_dt = 0.0
                best_rmse = math.inf
                for dt in shifts:
                    shifted = np.column_stack([np.interp(t_mov - dt, t_arr, pr_arr[:, ax]) for ax in range(3)])
                    rmse = float(np.sqrt(np.mean(np.sum((p_mov - shifted)**2, axis=1))))
                    if rmse < best_rmse:
                        best_rmse = rmse
                        best_dt = float(dt)
                result['tracking_lag_ms'] = best_dt * 1000.0
                result['lag_compensated_position_rmse'] = best_rmse
        for kind in ('control', 'rates'):
            observations = [e for e in events if e['kind'] == kind]
            gaps = [e['gap_s'] for e in observations if math.isfinite(e['gap_s'])]
            result[kind+'_observations'] = len(observations)
            result[kind+'_max_gap_s'] = max(gaps, default=None)
            result[kind+'_gaps_over_0_1s'] = sum(g > .1 for g in gaps)
            result[kind+'_bound_violations'] = sum(not e['bounds_valid'] for e in observations)
        # Controller solve timing metrics from /tmp/ommpc_solve_trace.csv
        if phase is not None and phase in self.f_phase_info:
            start_steady = self.f_started + self.f_phase_info[phase]['start_s']
            end_steady = self.f_started + self.f_phase_info[phase].get('end_s', self.f_time())
            solves = [s for s in self.f_solve_rows if start_steady <= s['steady_s'] <= end_steady]
        else:
            start_steady = self.f_started
            end_steady = self.f_started + self.f_time()
            solves = [s for s in self.f_solve_rows if start_steady <= s['steady_s'] <= end_steady]
        if solves:
            solve_ms = np.array([s['solve_us'] * 1e-3 for s in solves], dtype=float)
            step_ms = np.array([s['step_us'] * 1e-3 for s in solves], dtype=float)
            iters = np.array([s['iterations'] for s in solves], dtype=float)
            result['solve_samples'] = len(solves)
            result['solve_time_ms_mean'] = float(np.mean(solve_ms))
            result['solve_time_ms_p95'] = float(np.percentile(solve_ms, 95))
            result['solve_time_ms_max'] = float(np.max(solve_ms))
            result['step_time_ms_mean'] = float(np.mean(step_ms))
            result['step_time_ms_p95'] = float(np.percentile(step_ms, 95))
            result['step_time_ms_max'] = float(np.max(step_ms))
            result['osqp_iterations_mean'] = float(np.mean(iters))
            result['osqp_iterations_max'] = int(np.max(iters))
        else:
            result['solve_samples'] = 0
            result['solve_time_ms_mean'] = None
            result['solve_time_ms_p95'] = None
            result['solve_time_ms_max'] = None
            result['step_time_ms_mean'] = None
            result['step_time_ms_p95'] = None
            result['step_time_ms_max'] = None
            result['osqp_iterations_mean'] = None
            result['osqp_iterations_max'] = None
        if phase is not None:
            info = self.f_phase_info[phase].copy()
            counts_end = info.pop('counts_end', self.f_counts)
            counts_start = info.pop('counts_start')
            result.update(info)
            result['fresh_telemetry_counts'] = {k: counts_end[k]-counts_start[k] for k in self.f_counts}
        return finite_or_none(result)

    def f_save(self):
        if self.f_phase in self.f_phase_info:
            self.f_phase_info[self.f_phase].setdefault('end_s', self.f_time())
        self.f_solve_rows = load_solve_trace()
        phases = {}
        for name in self.f_phase_info:
            metric = self.f_metrics(name)
            phases[name] = metric
            (self.f_out/(name+'.json')).write_text(json.dumps(metric, indent=2, allow_nan=False)+'\n')
            with (self.f_out/(name+'.csv')).open('w', newline='') as file:
                writer = csv.DictWriter(file, TRACE_FIELDS)
                writer.writeheader()
                writer.writerows(r for r in self.f_rows if r['phase'] == name)
        if self.f_solve_rows:
            start_steady = self.f_started
            end_steady = self.f_started + self.f_time()
            run_solves = [s for s in self.f_solve_rows if start_steady <= s['steady_s'] <= end_steady]
            if run_solves:
                with (self.f_out/'solve_timing.csv').open('w', newline='') as file:
                    writer = csv.DictWriter(file, ['steady_s', 'solve_us', 'step_us', 'iterations', 'status', 'solver_status'])
                    writer.writeheader()
                    writer.writerows(run_solves)
        self.f_result.update({'duration_s': self.f_time(), 'last_phase': self.f_phase,
                              'takeoff_height_max_m': self.f_max_height,
                              'topic_counts': self.f_counts, 'commands': self.f_commands,
                              'command_acks': self.f_acks,
                              'metrics': self.f_metrics(),
                              'phases': phases})
        self.f_result = finite_or_none(self.f_result)
        (self.f_out/'result.json').write_text(json.dumps(self.f_result, indent=2, allow_nan=False)+'\n')
        for file in self.f_files:
            file.flush()
            file.close()
        print(json.dumps(self.f_result, indent=2), flush=True)
        print(f'Evidence: {self.f_out}', flush=True)


def start_bag(out):
    log = (out/'rosbag.log').open('w')
    try:
        process = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(out/'bag'), *TOPICS],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    except Exception:
        log.close()
        raise
    return process, log


def stop_bag(process, log):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=20.)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5.)
    log.close()
    return process.returncode


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--confirm-sitl', action='store_true', required=True,
                        help='Confirm ONLY the intended simulated PX4 is reachable in this DDS domain')
    parser.add_argument('--vehicle', choices=tuple(VEHICLES), default='x500',
                        help='Vehicle profile setting default max-thrust (x500=12 m/s^2, super=20 m/s^2)')
    parser.add_argument('--max-thrust', type=float, default=None,
                        help='Override maximum allowed thrust acceleration in m/s^2')
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument('--mode', choices=MODES, help='Run one mode after takeoff')
    selection.add_argument('--modes', choices=MODES, nargs='+', help='Run selected modes in sequence (default: original five)')
    parser.add_argument('--output-dir', type=Path,
                        help='New directory; must not exist. Default: timestamped under ~/uav_control_ws/ommpc-flight-results')
    parser.add_argument('--record-bag', action='store_true', help='Record essential reference/state/status/control/PX4 topics')
    parser.add_argument('--speed', type=float, default=None, help='Target speed for moving modes')
    parser.add_argument('--offset', type=float, nargs=3, default=None, help='Displacement offset N E D for target_step/hover_to_hover')
    parser.add_argument('--radius', type=float, default=None, help='Radius (m) for circle/figure8/helix')
    parser.add_argument('--z-amp', type=float, default=None, help='Vertical amplitude (m) for 3D helix')
    parser.add_argument('--loops', type=float, default=None, help='Number of periods for periodic curves')
    parser.add_argument('--ramp', type=float, default=None, help='Ramp duration (s) for line/curve modes')
    parser.add_argument('--abort-tilt', type=float, default=0.6, help='Abort tilt threshold in radians (default: 0.6)')
    parser.add_argument('--max-pos-rmse', type=float, default=0.35, help='Moving phase position RMSE threshold (m)')
    parser.add_argument('--max-vel-rmse', type=float, default=0.30, help='Moving phase velocity RMSE threshold (m/s)')
    args = parser.parse_args()
    default_modes = ['hover', 'target_step', 'constant_velocity', 'circle', 'hover_to_hover']
    modes = [args.mode] if args.mode else (args.modes or default_modes)
    if len(modes) != len(set(modes)):
        parser.error('Each mode may appear only once')
    out = (args.output_dir or Path.home()/'uav_control_ws/ommpc-flight-results'/
           datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')).expanduser().resolve()
    out.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    flight = None
    bag = bag_log = None
    try:
        flight = Flight(out, modes, vehicle=args.vehicle, max_thrust=args.max_thrust,
                        abort_tilt_rad=args.abort_tilt, max_pos_rmse=args.max_pos_rmse,
                        max_vel_rmse=args.max_vel_rmse, speed_override=args.speed,
                        radius_override=args.radius, z_amp_override=args.z_amp,
                        loops_override=args.loops, ramp_override=args.ramp)
        if args.speed is not None:
            flight.config['speed'] = float(args.speed)
        if args.offset is not None:
            flight.config['offset'] = np.array(args.offset, dtype=float)
        if args.radius is not None:
            flight.config['radius'] = float(args.radius)
        if args.z_amp is not None:
            flight.config['z_amp'] = float(args.z_amp)
        if args.loops is not None:
            flight.config['loops'] = float(args.loops)
        if args.ramp is not None:
            flight.config['ramp'] = float(args.ramp)
        if args.record_bag:
            bag, bag_log = start_bag(out)
            flight.f_until(lambda: (out/'bag').is_dir() or bag.poll() is not None, 10., 'rosbag recorder startup')
            if bag.poll() is not None:
                raise RuntimeError('rosbag exited during startup; see rosbag.log')
        flight.f_run()
    except (Exception, KeyboardInterrupt) as exc:
        traceback.print_exc()
        if flight is not None:
            flight.f_result['passed'] = False
            flight.f_result['error'] = f'{type(exc).__name__}: {exc}'
            if flight.f_engaged:
                try:
                    flight.f_land_safely(failure=True)
                except (Exception, KeyboardInterrupt) as recovery:
                    flight.f_result['recovery_error'] = f'{type(recovery).__name__}: {recovery}'
                    traceback.print_exc()
        else:
            (out/'result.json').write_text(json.dumps({
                'passed': False, 'modes_requested': modes,
                'error': f'{type(exc).__name__}: {exc}',
                'last_phase': 'node_construction'}, indent=2)+'\n')
    finally:
        if bag is not None:
            code = stop_bag(bag, bag_log)
            if flight is not None:
                flight.f_result['rosbag'] = {'returncode': code, 'metadata_present': (out/'bag'/'metadata.yaml').is_file()}
                if code != 0 or not (out/'bag'/'metadata.yaml').is_file():
                    flight.f_result['passed'] = False
                    flight.f_result['rosbag_error'] = 'Recorder failed or metadata missing; database files retained'
        if flight is not None:
            flight.publish_enabled = False
            flight.f_save()
            flight.destroy_node()
        rclpy.shutdown()
    return 0 if flight is not None and flight.f_result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
