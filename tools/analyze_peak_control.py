#!/usr/bin/env python3
"""Decompose recorded OMMPC turn peaks without shifting primary tracking errors.

Source ROS/project overlays; use Python with ROS, pyulog, scipy and matplotlib.
Example: analyze_peak_control.py CAMPAIGN --output CAMPAIGN/peak-control
Only reads flights; writes JSON, aligned CSV and PNG to the explicit output path.
Command minus reference is an optimization correction, NOT isolated feedback:
ommpc.cpp condenses H*x0 + D (nominal defects) across the prediction horizon.
"""
import argparse
import csv
import json
from collections import Counter
from pathlib import Path

import numpy as np
import rosbag2_py
from pyulog import ULog
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation, Slerp

from analyze_tracking_diagnosis import ClockMap, evaluate_reference, interpolate, read_bag, series
from reference_test_publisher import rotation, sample_reference


def stats(x):
    x = np.asarray(x)
    if not len(x):
        return None
    return {'rms': np.sqrt(np.mean(x*x, axis=0)).tolist(),
            'mean': np.mean(x, axis=0).tolist(), 'min': np.min(x, axis=0).tolist(),
            'max': np.max(x, axis=0).tolist()}


def correlation(x, y):
    return float(np.corrcoef(x, y)[0, 1]) if min(np.std(x), np.std(y)) > 1e-8 else None


def rate_relation(t, source, response):
    """Positive diagnostic lag: response(t) resembles source(t-lag)."""
    result = []
    shifts = np.arange(-.150, .1501, .002)
    interior = (t >= t[0]+.16) & (t <= t[-1]-.16)
    for axis in range(3):
        x, y = source[:, axis], response[:, axis]
        entry = {'zero_lag_correlation': correlation(x, y),
                 'zero_lag_rmse_rad_s': float(np.sqrt(np.mean((x-y)**2))),
                 'source_std_rad_s': float(np.std(x)), 'response_std_rad_s': float(np.std(y))}
        if min(np.std(x), np.std(y)) >= .025 and np.count_nonzero(interior) > 20:
            costs = [np.mean((np.interp(t[interior]-lag, t, x)-y[interior])**2) for lag in shifts]
            best = int(np.argmin(costs))
            entry.update(effective_lag_ms=float(shifts[best]*1000),
                         lag_search_boundary=best in (0, len(shifts)-1))
        result.append(entry)
    return result


def bag_controls(directory):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(directory/'bag'), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {x.name: x.type for x in reader.get_all_topics_and_types()}
    names = ('/uav/control_input', '/fmu/in/vehicle_rates_setpoint')
    classes = {name: get_message(types[name]) for name in names}
    controls, bridge = [], []
    while reader.has_next():
        topic, data, receipt = reader.read_next()
        if topic not in classes:
            continue
        msg = deserialize_message(data, classes[topic])
        if topic == names[0]:
            controls.append((receipt*1e-9, msg.thrust_acceleration, *msg.body_rates))
        else:
            bridge.append((receipt*1e-9, msg.timestamp*1e-6, msg.roll, msg.pitch, msg.yaw, *msg.thrust_body))
    return np.asarray(controls), np.asarray(bridge)


def payload_match(query_t, query_values, source_t, source_values, tolerance=.1):
    """Unique float32 wire payload within +/-100ms; never nearest-payload fallback."""
    source_values = np.asarray(source_values, dtype=np.float32)
    query_values = np.asarray(query_values, dtype=np.float32)
    indices = np.full(len(query_t), -1, dtype=int)
    counts = np.zeros(len(query_t), dtype=int)
    for i, (time, value) in enumerate(zip(query_t, query_values)):
        lo, hi = np.searchsorted(source_t, [time-tolerance, time+tolerance])
        hits = np.flatnonzero(np.all(source_values[lo:hi] == value, axis=1))+lo
        counts[i] = len(hits)
        if len(hits) == 1:
            indices[i] = hits[0]
    return indices, counts


def reference_elapsed_at_anchor(anchor_ns, start_ns, period_ns=20000000):
    """Mirror ommpc_inputs.hpp's one-period prestart tolerance, not a fitted shift."""
    if anchor_ns <= 0 or start_ns <= 0 or anchor_ns < start_ns-period_ns:
        raise ValueError('Anchor outside controller reference-start tolerance')
    return max(anchor_ns-start_ns, 0)*1e-9


def match_commands(directory, log, clock, references):
    control, bridge = bag_controls(directory)
    nt, native = series(log, 'vehicle_rates_setpoint',
                        ['roll', 'pitch', 'yaw', 'thrust_body[0]', 'thrust_body[1]', 'thrust_body[2]'], 'timestamp')
    # Only the common DDS-clock span is admissible; do not extrapolate clock mapping.
    keep = (nt >= clock.native_s[0]) & (nt <= clock.native_s[-1])
    nt, native = nt[keep], native[keep]
    bidx, bcount = payload_match(clock.ros(nt), native, bridge[:, 0], bridge[:, 2:8])
    cidx, ccount = payload_match(bridge[:, 0], bridge[:, 2:5], control[:, 0], control[:, 2:5])
    rows = list(csv.DictReader((directory/'solve_timing.csv').open()))
    rows = [r for r in rows if int(r['status']) == 0]
    finish = np.array([int(r['finished_ros_ns'])*1e-9 for r in rows])
    # No solve id is in ThrustRates. Accept only exactly one successful prior solve
    # within 10ms, and only if no other command points to that same solve.
    solve = np.full(len(control), -1, dtype=int)
    solve_counts = np.zeros(len(control), dtype=int)
    for i, time in enumerate(control[:, 0]):
        lo, hi = np.searchsorted(finish, [time-.010, time], side='right')
        solve_counts[i] = hi-lo
        if hi-lo == 1:
            solve[i] = lo
    duplicates = Counter(solve[solve >= 0])
    solve[np.array([duplicates.get(i, 0) > 1 for i in solve])] = -1
    reference_by_start = {}
    for receipt, msg in references:
        start = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        reference_by_start.setdefault(start, []).append((receipt, msg))
    matched = []
    reasons = Counter()
    prestart_anchors = 0
    for i, bi in enumerate(bidx):
        if bi < 0:
            reasons['native_bridge_ambiguous' if bcount[i] else 'native_bridge_missing'] += 1
            continue
        ci = cidx[bi]
        if ci < 0:
            reasons['bridge_control_ambiguous' if ccount[bi] else 'bridge_control_missing'] += 1
            continue
        si = solve[ci]
        if si < 0:
            reasons['solve_ambiguous_or_missing'] += 1
            continue
        row = rows[si]
        anchor_ns = int(row.get('reference_anchor_ros_ns', row['odometry_receipt_ros_ns']))
        anchor = anchor_ns*1e-9
        candidates = reference_by_start.get(int(row['reference_start_ns']), [])
        prior = [item for item in candidates if item[0] <= finish[si]]
        if not prior:
            reasons['reference_identity_unavailable'] += 1
            continue
        msg = prior[-1][1]
        start_ns = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        elapsed = reference_elapsed_at_anchor(anchor_ns, start_ns)
        prestart_anchors += int(anchor_ns < start_ns)
        ff = sample_reference(msg, elapsed)
        next_ref = sample_reference(msg, elapsed+.02)
        R0, R1 = rotation(ff.q), rotation(next_ref.q)
        dp = ff.position+.02*ff.velocity-next_ref.position
        dv = ff.velocity+.02*(np.array([0., 0., 9.80665])-ff.thrust_acceleration*R0[:, 2])-next_ref.velocity
        dR = Rotation.from_matrix(R1.T@R0@Rotation.from_rotvec(.02*ff.body_rates).as_matrix()).as_rotvec()
        fields = ('reference_thrust','reference_roll','reference_pitch','reference_yaw')
        ff_input = [float(row[k]) for k in fields] if all(k in row for k in fields) else [ff.thrust_acceleration,*ff.body_rates]
        matched.append((nt[i], ci, bi, si, anchor, finish[si], control[ci, 0],
                        *control[ci, 1:5], *ff_input,
                        bridge[bi, 0], *dp, *dv, *dR))
    data = np.asarray(matched)
    evidence = {'native_setpoints_in_clock_span': len(nt), 'bag_controls': len(control),
                'bag_bridge_setpoints': len(bridge), 'successful_solves': len(rows),
                'matched_chain': len(data), 'unmatched_reasons': dict(reasons),
                'accepted_prestart_anchors': prestart_anchors, 'reference_period_s': .02,
                'method': 'native->bag bridge: exact float32 rates+thrust; bridge->OMMPC: exact float32 rates; unique payload within +/-100ms. OMMPC->solve: unique prior successful completion within 10ms, one-to-one; NO solve id available.',
                'reference': 'Uses logged reference_anchor_ros_ns and reference_* when available; legacy receipt anchor otherwise. Polynomial identity is checked for geometry. planned_* never substitutes for a matched actual command.',
                'command_receipt_minus_solve_finish_ms': stats((data[:, 6]-data[:, 5])*1000),
                'native_stamp_minus_bag_bridge_receipt_ms': stats((data[:, 0]-clock.native(data[:, 15]))*1000),
                'native_setpoint_minus_anchor_ms': stats((data[:, 0]-clock.native(data[:, 4]))*1000)}
    return nt, native[:, :3], data, evidence


def project_lateral(R, vector):
    b3 = R[:, :, 2]
    return vector-b3*np.sum(vector*b3, axis=1)[:, None]


def drag_metrics(R, velocity, acceleration, direction, coefficient):
    measured = project_lateral(R, acceleration-[0., 0., 9.8])
    projected_velocity = project_lateral(R, velocity)
    expected = -coefficient*projected_velocity
    mismatch = measured-expected
    denom = np.sum(projected_velocity**2)
    centered = np.sum((measured-measured.mean(axis=0))**2)
    along = np.sum(measured*direction, axis=1)
    predicted_along = np.sum(expected*direction, axis=1)
    return {'known_k_s_inv': coefficient,
            'measured_lateral_vector_rms_m_s2': float(np.sqrt(np.mean(np.sum(measured**2, axis=1)))),
            'expected_drag_vector_rms_m_s2': float(np.sqrt(np.mean(np.sum(expected**2, axis=1)))),
            'mismatch_vector_rms_m_s2': float(np.sqrt(np.mean(np.sum(mismatch**2, axis=1)))),
            'fixed_k_centered_r2': float(1-np.sum(mismatch**2)/centered) if centered > 1e-12 else None,
            'descriptive_best_k_s_inv': float(-np.sum(measured*projected_velocity)/denom) if denom > 1e-9 else None,
            'projection_on_fixed_peak_actual_minus_reference_direction': {
                'measured_m_s2': stats(along), 'expected_drag_m_s2': stats(predicted_along),
                'correlation': correlation(along, predicted_along)},
            'note': 'Lateral projector I-b3*b3^T removes unknown axial thrust, not other dynamics. Fixed k from straight runs; best k is descriptive, not causal attribution.'}, measured, expected


def save_csv(path, columns):
    names, arrays = [], []
    for name, values in columns.items():
        values = np.asarray(values)
        if values.ndim == 1:
            names.append(name)
            arrays.append(values)
        else:
            names.extend(f'{name}_{axis}' for axis in range(values.shape[1]))
            arrays.extend(values.T)
    with path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(names)
        writer.writerows(zip(*arrays))


def plot_phase(path, t, peak, columns, mask):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(7, 1, figsize=(12, 18), sharex=True)
    dt = t-peak
    for axis in range(3):
        axes[axis].plot(dt, columns['ff_anchor_rates_rad_s'][:, axis], label='reference at solve anchor')
        axes[axis].plot(dt, columns['ommpc_rates_rad_s'][:, axis], label='OMMPC command')
        axes[axis].plot(dt, columns['optimization_correction_rad_s'][:, axis], label='command - reference')
        axes[axis].plot(dt, columns['gyro_rad_s'][:, axis], label='actual gyro', alpha=.75)
        axes[axis].set_ylabel(f'{"pqr"[axis]} [rad/s]')
    axes[3].plot(dt, columns['ff_anchor_thrust_m_s2'], label='reference thrust')
    axes[3].plot(dt, columns['ommpc_thrust_m_s2'], label='command thrust')
    axes[3].set_ylabel('Thrust [m/s²]')
    axes[4].plot(dt, np.linalg.norm(columns['attitude_error_rad'], axis=1), label='attitude error')
    axes[4].set_ylabel('Attitude error [rad]')
    axes[5].plot(dt, np.linalg.norm(columns['position_error_m'], axis=1), label='position error')
    axes[5].set_ylabel('Position error [m]')
    axes[6].plot(dt, columns['lateral_residual_peak_projection_m_s2'], label='observed lateral projection')
    axes[6].plot(dt, columns['drag_peak_projection_m_s2'], label='known drag projection')
    axes[6].set_ylabel('Acceleration [m/s²]')
    axes[6].set_xlabel('Native time relative to position-error peak [s]')
    for ax in axes:
        ax.axvline(0, color='black', linewidth=.8)
        ax.set_xlim(dt[mask][0], dt[mask][-1])
        ax.grid(alpha=.3)
        ax.legend(loc='upper right', fontsize=8, ncol=2)
    fig.suptitle(path.stem+' | no fitted time shift; gaps = unmatched command chain')
    fig.tight_layout(rect=(0, 0, 1, .97))
    fig.savefig(path, dpi=140)
    plt.close(fig)


def analyze(directory, output, half_window, coefficient):
    out = output/directory.name
    out.mkdir(parents=True, exist_ok=True)
    original = json.loads((directory/'diagnostics/report.json').read_text())
    result = json.loads((directory/'result.json').read_text())
    log = ULog(str(directory/'px4.ulg'))
    clock = ClockMap(log)
    _, references = read_bag(directory)
    ct, command, matches, association = match_commands(directory, log, clock, references)
    save_csv(out/'command_matches.csv', {'native_s': matches[:, 0], 'bag_control_index': matches[:, 1],
             'bag_bridge_index': matches[:, 2], 'solve_index': matches[:, 3],
             'anchor_ros_s': matches[:, 4], 'solve_finished_ros_s': matches[:, 5],
             'bag_control_receipt_ros_s': matches[:, 6], 'command_thrust_rates': matches[:, 7:11],
             'ff_anchor_thrust_rates': matches[:, 11:15], 'bag_bridge_receipt_ros_s': matches[:, 15],
             'first_step_nominal_defect_position_m': matches[:, 16:19],
             'first_step_nominal_defect_velocity_m_s': matches[:, 19:22],
             'first_step_nominal_defect_attitude_rad': matches[:, 22:25]})
    gt_t, gt = series(log, 'vehicle_local_position_groundtruth', ['x','y','z','vx','vy','vz'])
    at, quat = series(log, 'vehicle_attitude_groundtruth', [f'q[{i}]' for i in range(4)])
    attitude = Slerp(at, Rotation.from_quat(quat[:, [1,2,3,0]]))
    wt, gyro = series(log, 'vehicle_angular_velocity', [f'xyz[{i}]' for i in range(3)])
    origin = np.asarray(original['frame']['fixed_prearm_origin_offset_ned_m'])
    report = {'flight': directory.name, 'clock': clock.quality, 'association': association,
              'semantics': 'u_command=u_reference[0]+delta_u[0]. delta_u minimizes horizon cost with free_response=H*state_error+D, where D propagates nominal one-step defects; includes closed-loop correction, nominal defects and horizon anticipation, NOT pure feedback.',
              'nominal_defect': 'First-step predict_step(reference(anchor),u_ff(anchor),dt=.02)-reference(anchor+.02), using forward-Euler p/v and SO(3) Exp, model gravity 9.80665. Geometric/discretization mismatch, not flight tracking error. Not the whole condensed horizon D.',
              'limits_thrust_p_q_r': {'min': [1.,-3.,-3.,-1.5], 'max': [20.,3.,3.,1.5]},
              'lag_note': 'Positive lag means response(t) resembles source(t-lag); +/-150ms diagnostic RMSE search only. Closed-loop, correlated signals are not causal delay identification; primary data are never shifted.',
              'limitations': 'No applied motor thrust estimate or open-loop intervention. Finite-differenced truth, groundtruth attitude/gyro differences, DDS clock mapping and rotor-link dynamics contaminate lateral residual. Command-to-solve mapping lacks ids. No decomposition of optimized delta_u into causal feedback/defect/anticipation.',
              'phases': {}}
    for phase, original_phase in original['phases'].items():
        if 'peak' not in original_phase:
            continue
        info = result['phases'][phase]
        begin, end = clock.native([info['ros_start_ns']*1e-9, info['ros_end_ns']*1e-9])
        t = np.arange(begin, end, .02)
        peak = original_phase['peak']['native_time_s']
        extended = np.r_[t[0]+np.arange(-5, 0)*.02, t, t[-1]+np.arange(1, 6)*.02]
        truth_ext = interpolate(extended, gt_t, gt)
        acceleration = savgol_filter(truth_ext[:, 3:], 7, 3, deriv=1, delta=.02, axis=0)[5:-5]
        truth = truth_ext[5:-5]
        R = attitude(t).as_matrix()
        rp, rv, rR, thrust = evaluate_reference(clock.ros(t), references)
        rates_now = []
        publication = np.array([r[0] for r in references])
        for time in clock.ros(t):
            msg = references[np.searchsorted(publication, time, side='right')-1][1]
            rates_now.append(sample_reference(msg, time-msg.header.stamp.sec-msg.header.stamp.nanosec*1e-9).body_rates)
        rates_now = np.asarray(rates_now)
        # Hold matched command only while it is the SAME native setpoint currently active.
        current = np.searchsorted(ct, t, side='right')-1
        matched_index = np.searchsorted(matches[:, 0], t, side='right')-1
        valid = (matched_index >= 0) & (current >= 0)
        valid &= matches[np.maximum(matched_index, 0), 0] == ct[np.maximum(current, 0)]
        held = matches[np.maximum(matched_index, 0)]
        ff, total = held[:, 12:15].copy(), held[:, 8:11].copy()
        ff[~valid], total[~valid] = np.nan, np.nan
        fthrust, cthrust = held[:, 11].copy(), held[:, 7].copy()
        fthrust[~valid], cthrust[~valid] = np.nan, np.nan
        correction = total-ff
        actual = interpolate(t, wt, gyro)
        native_cmd = interpolate(t, ct, command, hold=True)
        error = truth[:, :3]+origin-rp
        attitude_error = Rotation.from_matrix(np.einsum('nji,njk->nik', rR, R)).as_rotvec()
        peak_index = int(np.argmin(abs(t-peak)))
        direction = error[peak_index]/np.linalg.norm(error[peak_index])
        directions = np.broadcast_to(direction, error.shape)
        window = abs(t-peak) <= half_window
        _, residual, drag = drag_metrics(R, truth[:, 3:], acceleration, directions, coefficient)
        ref_acceleration = np.gradient(rv, t, axis=0)
        attitude_acceleration = -thrust[:, None]*(R[:, :, 2]-rR[:, :, 2])
        columns = {'native_s': t, 'seconds_from_peak': t-peak, 'command_chain_valid': valid.astype(int),
                   'position_error_m': error, 'reference_velocity_m_s': rv,
                   'ff_now_rates_rad_s': rates_now, 'ff_anchor_rates_rad_s': ff,
                   'ommpc_rates_rad_s': total, 'native_rates_rad_s': native_cmd,
                   'optimization_correction_rad_s': correction, 'gyro_rad_s': actual,
                   'gyro_minus_native_command_rad_s': actual-native_cmd,
                   'ff_anchor_minus_now_rates_rad_s': ff-rates_now,
                   'first_step_nominal_defect_position_m': held[:, 16:19],
                   'first_step_nominal_defect_velocity_m_s': held[:, 19:22],
                   'first_step_nominal_defect_attitude_rad': held[:, 22:25],
                   'ff_anchor_thrust_m_s2': fthrust, 'ommpc_thrust_m_s2': cthrust,
                   'thrust_correction_m_s2': cthrust-fthrust, 'attitude_error_rad': attitude_error,
                   'acceleration_m_s2': acceleration, 'reference_acceleration_m_s2': ref_acceleration,
                   'acceleration_tracking_residual_m_s2': acceleration-ref_acceleration,
                   'attitude_only_acceleration_difference_m_s2': attitude_acceleration,
                   'lateral_residual_m_s2': residual, 'known_drag_m_s2': drag,
                   'lateral_residual_peak_projection_m_s2': residual@direction,
                   'drag_peak_projection_m_s2': drag@direction}
        for key in ('first_step_nominal_defect_position_m', 'first_step_nominal_defect_velocity_m_s',
                    'first_step_nominal_defect_attitude_rad'):
            columns[key] = columns[key].copy()
            columns[key][~valid] = np.nan
        save_csv(out/f'{phase}_aligned.csv', columns)
        plot_phase(out/f'{phase}.png', t, peak, columns, window)
        phase_report = {'peak_native_s': peak, 'peak_position_error_m': float(np.linalg.norm(error[peak_index])),
                        'peak_actual_minus_reference_direction_ned': direction.tolist(),
                        'peak_window_half_width_s': half_window, 'windows': {}}
        phase_report['peak_sample'] = {
            key: value[peak_index].tolist() if np.all(np.isfinite(value[peak_index])) else None
            for key, value in columns.items()}
        for label, mask in [('full', np.ones(len(t), dtype=bool)), ('peak', window)]:
            usable = mask & valid
            measurements = {key: stats(values[usable]) for key, values in columns.items()
                            if key not in ('native_s','seconds_from_peak','command_chain_valid')}
            measurements['grid_samples'] = int(np.sum(mask))
            measurements['matched_grid_samples'] = int(np.sum(usable))
            measurements['unmatched_grid_samples'] = int(np.sum(mask & ~valid))
            measurements['native_command_to_gyro'] = rate_relation(t[mask], native_cmd[mask], actual[mask])
            # Lag diagnostics on contiguous valid spans only: do not bridge unknown commands.
            runs = np.split(np.flatnonzero(usable), np.flatnonzero(np.diff(np.flatnonzero(usable)) != 1)+1)
            longest = max(runs, key=len)
            if len(longest) > 40:
                measurements['relation_contiguous_span_s'] = float(t[longest[-1]]-t[longest[0]])
                measurements['reference_to_total'] = rate_relation(t[longest], ff[longest], total[longest])
                measurements['reference_to_correction'] = rate_relation(t[longest], ff[longest], correction[longest])
                measurements['reference_to_gyro'] = rate_relation(t[longest], ff[longest], actual[longest])
            measurements['correction_reference_opposite_sign_fraction'] = [
                float(np.mean(ff[usable, axis]*correction[usable, axis] < 0)) for axis in range(3)]
            u = np.column_stack((cthrust[usable], total[usable]))
            lower, upper = np.array([1.,-3.,-3.,-1.5]), np.array([20.,3.,3.,1.5])
            measurements['min_distance_to_input_bounds'] = np.minimum(u-lower, upper-u).min(axis=0).tolist()
            measurements['bound_hit_count_1e_3'] = np.sum((u <= lower+.001) | (u >= upper-.001), axis=0).tolist()
            measurements['drag'], _, _ = drag_metrics(R[mask], truth[mask, 3:], acceleration[mask], directions[mask], coefficient)
            measurements['derivative_window_sensitivity'] = {}
            for length in (5, 11):
                accel = savgol_filter(truth_ext[:, 3:], length, 3, deriv=1, delta=.02, axis=0)[5:-5]
                sens, _, _ = drag_metrics(R[mask], truth[mask, 3:], accel[mask], directions[mask], coefficient)
                measurements['derivative_window_sensitivity'][str(length)+'_samples'] = sens
            phase_report['windows'][label] = measurements
        # Straight cruise is an external benchmark, not pooled with turning peak fits.
        cruise = (np.linalg.norm(ref_acceleration, axis=1) < .1) & (np.linalg.norm(rv, axis=1) > .5)
        cruise &= (t > t[0]+.15) & (t < t[-1]-.15)
        if np.sum(cruise) > 10:
            phase_report['cruise_drag'], _, _ = drag_metrics(R[cruise], truth[cruise, 3:], acceleration[cruise], directions[cruise], coefficient)
            phase_report['cruise_samples'] = int(np.sum(cruise))
        report['phases'][phase] = phase_report
    (out/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'flight': directory.name, 'matched': association['matched_chain'],
                     'phases': {k: {'peak_m': v['peak_position_error_m'],
                                    'unmatched_peak': v['windows']['peak']['unmatched_grid_samples']}
                                for k, v in report['phases'].items()}}), flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('path', type=Path, help='one flight or campaign root')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--half-window', type=float, default=1.)
    parser.add_argument('--drag-coefficient', type=float, default=.079)
    args = parser.parse_args()
    directories = [args.path] if (args.path/'px4.ulg').exists() else sorted(
        p for p in args.path.iterdir() if (p/'diagnostics/report.json').exists()
        and p.name.startswith(('scurve-', 'sharp3d-', 'line-')))
    reports = [analyze(p, args.output, args.half_window, args.drag_coefficient) for p in directories]
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'campaign.json').write_text(json.dumps(reports, indent=2, allow_nan=False)+'\n')


if __name__ == '__main__':
    main()
