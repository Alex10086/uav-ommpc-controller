#!/usr/bin/env python3
"""SITL only: hover with additive per-axis rate excitation, no controller changes.

Remap ommpc_node output to /uav/sysid/raw_control. This process is the only
publisher on /uav/control_input; it forwards each fresh raw command once.
Existing Flight performs takeoff, freshness/tilt/error checks and native landing.
No held-command timer or timeout fallback is introduced.
"""
import argparse
import math
import time
import traceback
from pathlib import Path

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from uav_ommpc_interfaces.msg import ThrustRates
from sitl_ommpc_test import Flight, start_bag, stop_bag


class RateIdentification(Flight):
    def __init__(self, out, step_validation=False):
        super().__init__(out, ['rate_identification'], vehicle='super')
        self.step_validation = step_validation
        self.excitation_axis = None
        self.excitation_kind = None
        self.excitation_start = 0.0
        self.injection_error = None
        self.command_pub = self.create_publisher(ThrustRates, '/uav/control_input', 1)
        self.raw_sub = self.create_subscription(
            ThrustRates, '/uav/sysid/raw_control', self.on_raw,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.excitation_csv = self.f_csv('excitation.csv', [
            'ros_ns', 'phase', 'elapsed_s', 'axis', 'excitation_rad_s',
            'raw_roll', 'raw_pitch', 'raw_yaw', 'sent_roll', 'sent_pitch', 'sent_yaw'])
        self.f_result['identification'] = {
            'controller': 'unchanged tuned OMMPC; output remapped through test-only relay',
            'bridge': 'quadratic mapping, hover_thrust=0.309128',
            'training': '24 s linear chirp, 0.4 to 6 Hz, 1 s cosine taper',
            'validation': '12 s independent alternating rate doublets',
            'amplitudes_rad_s': [0.22, 0.22, 0.18],
            'step_validation': step_validation,
            'compensation_settings_source': 'experiment manifest and bridge parameter snapshot'}

    def on_raw(self, raw):
        t = time.monotonic() - self.excitation_start
        addition = 0.0
        axis = self.excitation_axis
        if axis is not None and not self.f_landing:
            amplitude = (0.22, 0.22, 0.18)[axis]
            if self.excitation_kind == 'chirp' and 0.0 <= t < 24.0:
                edge = min(1.0, t, 24.0-t)
                envelope = 0.5-0.5*math.cos(math.pi*edge)
                addition = amplitude*envelope*math.sin(2*math.pi*(0.4*t+0.5*(5.6/24)*t*t))
            elif self.excitation_kind == 'doublet' and 0.0 <= t < 12.0:
                cycle = t % 2.0
                addition = amplitude*(1.0 if cycle < 0.3 else -1.0 if cycle < 0.6 else 0.0)
        sent = ThrustRates()
        sent.thrust_acceleration = raw.thrust_acceleration
        rates = np.asarray(raw.body_rates, dtype=float).copy()
        if axis is not None:
            if (self.step_validation and self.excitation_kind == 'doublet' and
                    not self.f_landing and 0.0 <= t < 12.0 and t % 2.0 < 0.8):
                # Direct, constant 300 ms rate plateaus permit actual step-overshoot measurement.
                # OMMPC resumes this axis for the remainder of each 2 s cycle.
                rates[axis] = addition
            else:
                rates[axis] += addition
        if not np.all(np.isfinite(rates)) or np.any(np.abs(rates) > [3., 3., 1.5]):
            self.injection_error = 'Excitation would exceed rate limits; no clipped identification input'
            return
        sent.body_rates = rates.tolist()
        self.command_pub.publish(sent)
        self.excitation_csv.writerow([
            self.get_clock().now().nanoseconds, self.f_phase, t, -1 if axis is None else axis,
            addition, *raw.body_rates, *rates])

    def f_tick(self):
        super().f_tick()
        if self.injection_error is not None and not self.f_landing:
            raise RuntimeError(self.injection_error)

    def f_mode(self, mode):
        target = self.f_desired.copy()
        for axis, name in enumerate(('roll', 'pitch', 'yaw')):
            for kind, duration in [('chirp', 24.0), ('doublet', 12.0)]:
                self.f_phase_start(f'{name}_{kind}')
                self.f_set_hover(target)
                self.excitation_start = time.monotonic()
                self.f_phase_info[self.f_phase]['ros_start_ns'] = self.get_clock().now().nanoseconds
                self.excitation_axis, self.excitation_kind = axis, kind
                deadline = self.excitation_start+duration
                try:
                    self.f_until(lambda: time.monotonic() >= deadline, duration+3., 'rate excitation')
                finally:
                    self.excitation_axis = None
                    self.f_phase_info[self.f_phase]['ros_end_ns'] = self.get_clock().now().nanoseconds
                # Avoid expensive lag statistics while the next excitation is active.
                self.f_phase_info[self.f_phase]['completed'] = True
                self.f_phase_start(f'{name}_{kind}_settle')
                self.f_settle(target)
                self.f_phase_info[self.f_phase]['completed'] = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--confirm-sitl', required=True, action='store_true')
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--step-validation', action='store_true')
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    flight = RateIdentification(args.output_dir, step_validation=args.step_validation)
    bag = log = None
    try:
        bag, log = start_bag(args.output_dir)
        flight.f_until(lambda: (args.output_dir/'bag').is_dir() or bag.poll() is not None,
                       10., 'recorder startup')
        if bag.poll() is not None:
            raise RuntimeError('Recorder failed to start')
        flight.f_run()
    except (Exception, KeyboardInterrupt) as exc:
        traceback.print_exc()
        flight.excitation_axis = None
        flight.f_result['passed'] = False
        flight.f_result['error'] = str(exc)
        if flight.f_engaged:
            try:
                flight.f_land_safely(failure=True)
            except Exception as recovery:
                flight.f_result['recovery_error'] = str(recovery)
    finally:
        flight.excitation_axis = None
        if bag is not None:
            code = stop_bag(bag, log)
            flight.f_result['rosbag_returncode'] = code
            if code != 0:
                flight.f_result['passed'] = False
        flight.publish_enabled = False
        flight.f_save()
        flight.destroy_node()
        rclpy.shutdown()
    return 0 if flight.f_result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
