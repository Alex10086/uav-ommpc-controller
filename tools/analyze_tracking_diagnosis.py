#!/usr/bin/env python3
"""Diagnose SITL tracking from native ULog truth, DDS stamps and bagged polynomials.

Source ROS/project overlays and use a Python with pyulog, scipy and matplotlib.
No correlation-derived time shift is applied to any reported tracking error.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from pyulog import ULog
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation, Slerp

from reference_test_publisher import sample_reference, rotation
from tracking_diagnostics import analyze_phase


def summary(values):
    x = np.asarray(values, dtype=float)
    x = x[np.isfinite(x)]
    return {'count': len(x), 'mean': float(np.mean(x)),
            'p5': float(np.percentile(x, 5)), 'p50': float(np.median(x)),
            'p95': float(np.percentile(x, 95)), 'max': float(np.max(x))} if len(x) else None


def rms(x):
    x = np.asarray(x)
    return float(np.sqrt(np.mean(np.sum(x*x, axis=1)))) if len(x) else None


def series(log, name, fields, stamp='timestamp_sample'):
    d = log.get_dataset(name).data
    t = d[stamp].astype(float)*1e-6
    keep = np.r_[np.diff(t) != 0, True]  # retain last duplicate
    t = t[keep]
    if len(t) < 2 or np.any(np.diff(t) <= 0):
        raise ValueError(f'Nonmonotonic or insufficient {name} timestamps')
    return t, np.column_stack([d[f][keep] for f in fields]).astype(float)


def interpolate(t, source_t, values, hold=False):
    t = np.asarray(t)
    if np.any(t < source_t[0]) or np.any(t > source_t[-1]):
        raise ValueError('Refusing time-series extrapolation')
    if hold:
        return values[np.searchsorted(source_t, t, side='right')-1]
    return np.column_stack([np.interp(t, source_t, values[:, i]) for i in range(values.shape[1])])


class ClockMap:
    """DDS estimated_offset = native - ROS; preserve native chronology."""
    def __init__(self, log):
        candidates = [d.data for d in log.data_list if d.name == 'timesync_status']
        rows = []
        for d in candidates:
            mask = (d['source_protocol'] == 2) & (np.abs(d['estimated_offset']) > 1e12)
            rows.extend(zip(d['timestamp'][mask], d['estimated_offset'][mask], d['round_trip_time'][mask]))
        rows = np.asarray(sorted(rows), dtype=float)
        if len(rows) < 3:
            raise ValueError('Need at least three synchronized DDS clock observations')
        self.native_s = rows[:, 0]*1e-6
        self.ros_s = (rows[:, 0]-rows[:, 1])*1e-6
        if np.any(np.diff(self.native_s) <= 0) or np.any(np.diff(self.ros_s) <= 0):
            raise ValueError('Clock mapping is nonmonotonic; cannot align this flight')
        self.quality = {'dds_sync_samples': len(rows), 'round_trip_ms': summary(rows[:, 2]*1e-3),
                        'offset_change_per_observation_ms': summary(np.diff(rows[:, 1])*1e-3),
                        'method': 'piecewise linear DDS native-minus-ROS offset; no optimized time shift',
                        'uncertainty': 'RTT asymmetry, 1 Hz sync sampling and Gazebo bridge callback timing remain'}

    def native(self, ros):
        return interpolate(np.atleast_1d(ros), self.ros_s, self.native_s[:, None])[:, 0]

    def ros(self, native):
        return interpolate(np.atleast_1d(native), self.native_s, self.ros_s[:, None])[:, 0]


def read_bag(directory):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(directory/'bag'), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    selected = ('/uav/state', '/uav/reference_polynomial')
    classes = {name: get_message(types[name]) for name in selected}
    states, references = [], []
    while reader.has_next():
        topic, data, receipt_ns = reader.read_next()
        if topic not in classes:
            continue
        msg = deserialize_message(data, classes[topic])
        if topic == '/uav/state':
            if msg.pose_frame != 1 or msg.velocity_frame != 1:
                raise ValueError('Expected NED estimator pose and velocity')
            states.append((msg.timestamp_sample*1e-6, msg.timestamp*1e-6, receipt_ns*1e-9,
                           *msg.position, *msg.velocity, *msg.q, msg.reset_counter))
        else:
            references.append((receipt_ns*1e-9, msg))
    odom = np.asarray(states, dtype=float)
    if len(odom) < 10 or not references:
        raise ValueError('Missing state/reference bag samples')
    keep = np.r_[np.diff(odom[:, 0]) != 0, True]
    odom = odom[keep]
    if np.any(np.diff(odom[:, 0]) <= 0):
        raise ValueError('Nonmonotonic estimator sample timestamps')
    return odom, references


def validate_frame(local_position, begin_native):
    # Odometry reset_counter sums position, velocity AND quaternion resets.
    # A heading reset is not a translation of the fixed local NED origin.
    mask = local_position['timestamp']*1e-6 >= begin_native
    if np.count_nonzero(mask) < 2:
        raise ValueError('Insufficient local-position frame metadata')
    events = {}
    for field in ('xy_reset_counter', 'z_reset_counter', 'ref_timestamp',
                  'vxy_reset_counter', 'vz_reset_counter', 'heading_reset_counter'):
        indices = np.flatnonzero(np.diff(local_position[field][mask].astype(float)) != 0)+1
        events[field] = (local_position['timestamp'][mask][indices]*1e-6).tolist()
    if any(events[field] for field in ('xy_reset_counter', 'z_reset_counter', 'ref_timestamp')):
        raise ValueError('Position/origin reset after pre-arm calibration requires an explicit frame transform')
    return events


def evaluate_reference(ros, references):
    published = np.array([r[0] for r in references])
    indices = np.searchsorted(published, ros, side='right')-1
    if np.any(indices < 0):
        raise ValueError('No published reference at requested native sample time')
    values = []
    for time_s, index in zip(ros, indices):
        msg = references[index][1]
        start = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        values.append(sample_reference(msg, time_s-start))
    return (np.array([v.position for v in values]), np.array([v.velocity for v in values]),
            np.array([rotation(v.q) for v in values]),
            np.array([v.thrust_acceleration for v in values]))


def rate_metrics(t, command_t, command, gyro_t, gyro):
    actual = interpolate(t, gyro_t, gyro)
    desired = interpolate(t, command_t, command, hold=True)
    result = {'rmse_rad_s': np.sqrt(np.mean((actual-desired)**2, axis=0)).tolist(),
              'command_std_rad_s': np.std(desired, axis=0).tolist(),
              'effective_lag_ms': [], 'note': 'correlation diagnostic only; not pure dead time or alignment correction'}
    for axis in range(3):
        if np.std(desired[:, axis]) < .025:
            result['effective_lag_ms'].append(None)
            continue
        valid = (t > command_t[0]+.16) & (t < command_t[-1]-.16)
        costs = []
        shifts = np.arange(-.150, .1501, .001)
        for lag in shifts:
            shifted = interpolate(t[valid]-lag, command_t, command, hold=True)[:, axis]
            costs.append(np.mean((actual[valid, axis]-shifted)**2))
        result['effective_lag_ms'].append(float(shifts[np.argmin(costs)]*1000))
    return result, actual, desired


def controller_evidence(directory, clock, odom, gt_t, gt_p, origin, begin, end):
    path = directory/'solve_timing.csv'
    if not path.exists():
        return {'available': False}, None
    rows = list(csv.DictReader(path.open()))
    if not rows or 'finished_ros_ns' not in rows[0]:
        return {'available': False, 'reason': 'legacy trace has no controller clock fields'}, None
    curve_rows = [r for r in rows if begin-.2 <= int(r['finished_ros_ns'])*1e-9 <= end+.2]
    curve_use = np.array([int(r['finished_ros_ns'])*1e-9 for r in curve_rows])
    curve_sample = np.array([int(r['odometry_sample_us'])*1e-6 for r in curve_rows])
    rows = [r for r in rows if begin <= int(r['finished_ros_ns'])*1e-9 <= end]
    if not rows:
        return {'available': False, 'reason': 'no phase solve records'}, None
    stamp = np.array([int(r['odometry_sample_us'])*1e-6 for r in rows])
    use = np.array([int(r['finished_ros_ns'])*1e-9 for r in rows])
    receipt = np.array([int(r['odometry_receipt_ros_ns'])*1e-9 for r in rows])
    publication = np.array([int(r['odometry_timestamp_us'])*1e-6 for r in rows])
    native_sample, native_use = clock.native(stamp), clock.native(use)
    sample_truth = interpolate(native_sample, gt_t, gt_p)+origin
    use_truth = interpolate(native_use, gt_t, gt_p)+origin
    at = np.searchsorted(odom[:, 0], stamp)
    at = np.minimum(at, len(odom)-1)
    previous = np.maximum(at-1, 0)
    at = np.where(abs(odom[previous, 0]-stamp) < abs(odom[at, 0]-stamp), previous, at)
    matched = abs(odom[at, 0]-stamp) < 2e-6
    estimation = odom[at, 3:6]-sample_truth
    staleness = sample_truth-use_truth
    total = odom[at, 3:6]-use_truth
    prediction_dt = np.array([float(r.get('state_prediction_dt_s', 0.0)) for r in rows])
    anchor = np.array([int(r.get('reference_anchor_ros_ns', r['odometry_receipt_ros_ns']))*1e-9 for r in rows])
    controller_position = odom[at, 3:6] + prediction_dt[:,None]*odom[at, 6:9]
    anchor_truth = interpolate(clock.native(anchor), gt_t, gt_p)+origin
    input_at_anchor_error = controller_position-anchor_truth
    input_at_use_error = controller_position-use_truth
    evidence = {'available': True, 'samples': len(rows), 'exact_sample_match_fraction': float(np.mean(matched)),
                'sample_to_estimator_publication_ms': summary((publication-stamp)*1000),
                'publication_to_controller_receipt_ms': summary((receipt-publication)*1000),
                'sample_to_controller_receipt_ms': summary((receipt-stamp)*1000),
                'sample_to_solve_finish_ms': summary((use-stamp)*1000),
                'negative_sample_to_receipt_fraction': float(np.mean(receipt < stamp)),
                'controller_wait_ms': summary([float(r['state_wait_us'])*.001 for r in rows]),
                'solve_ms': summary([float(r['solve_us'])*.001 for r in rows]),
                'estimator_at_sample_position_rmse_m': rms(estimation[matched]),
                'stale_motion_position_rmse_m': rms(staleness[matched]),
                'held_state_at_use_position_rmse_m': rms(total[matched]),
                'prediction_dt_ms': summary(prediction_dt*1000),
                'reference_anchor_to_solve_finish_ms': summary((use-anchor)*1000),
                'controller_input_at_anchor_position_rmse_m': rms(input_at_anchor_error[matched]),
                'controller_input_at_use_position_rmse_m': rms(input_at_use_error[matched]),
                'decomposition': 'held_estimate-truth(use) = estimation_error(sample) + truth(sample)-truth(use)',
                'note': 'age uses translated DDS stamps; negative values expose sync uncertainty, not negative physical delay'}
    with (directory/'diagnostics'/f'controller_{begin:.6f}.csv').open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['native_use_s', 'native_sample_s', 'sample_age_ms', 'exact_sample_match',
                    'estimation_error_m', 'stale_motion_m', 'held_state_error_m',
                    'controller_input_error_at_anchor_m','controller_input_error_at_use_m','prediction_dt_s'])
        w.writerows(zip(native_use, native_sample, (use-stamp)*1000, matched,
                        np.linalg.norm(estimation, axis=1), np.linalg.norm(staleness, axis=1),
                        np.linalg.norm(total, axis=1),np.linalg.norm(input_at_anchor_error,axis=1),
                        np.linalg.norm(input_at_use_error,axis=1),prediction_dt))
    return evidence, (clock.native(curve_use), (curve_use-curve_sample)*1000)


def drag_evidence(t, velocity, R, acceleration, reference_velocity):
    # Inertial derivative first, then rotate. d(body velocity)/dt would need omega cross v.
    body_velocity = np.einsum('nji,nj->ni', R, velocity)
    specific = acceleration - [0., 0., 9.8]  # actual default.sdf gravity
    body_force = np.einsum('nji,nj->ni', R, specific)
    lateral = body_force.copy()
    lateral[:, 2] = 0.0  # eliminate unknown thrust magnitude, not estimate axial drag
    residual = np.einsum('nij,nj->ni', R, lateral)
    acceleration_ref = np.gradient(reference_velocity, t, axis=0)
    cruise = ((np.linalg.norm(acceleration_ref, axis=1) < .10) &
              (np.linalg.norm(reference_velocity, axis=1) > .5) &
              (t > t[0]+.15) & (t < t[-1]-.15))
    fit = []
    for axis in range(2):
        x, y = body_velocity[:, axis], body_force[:, axis]
        if np.std(x) < .15:
            fit.append(None)
            continue
        A = np.column_stack([-x, np.ones(len(x))])
        coefficient, bias = np.linalg.lstsq(A, y, rcond=None)[0]
        predicted = A @ np.array([coefficient, bias])
        variance = np.sum((y-np.mean(y))**2)
        fit.append({'effective_drag_s_inverse': float(coefficient), 'bias_m_s2': float(bias),
                    'residual_rmse_m_s2': float(np.sqrt(np.mean((y-predicted)**2))),
                    'r_squared': float(1-np.sum((y-predicted)**2)/variance) if variance > 1e-10 else None})
    return {'definition': 'R^T(a_truth - gravity), body x/y; axial thrust projected out',
            'derivative': '20 ms uniform interpolation; centered 7-point cubic Savitzky-Golay derivative of truth velocity',
            'effective_lateral_fit_xy': fit, 'cruise_samples': int(np.sum(cruise)),
            'cruise_body_velocity_mean_m_s': np.mean(body_velocity[cruise], axis=0).tolist() if np.any(cruise) else None,
            'cruise_lateral_acceleration_mean_m_s2': np.mean(lateral[cruise], axis=0).tolist() if np.any(cruise) else None,
            'lateral_acceleration_rms_m_s2': rms(lateral),
            'limitations': 'finite-differenced truth, sync/attitude interpolation, rotor-link motion; association not causal error attribution'}, residual, body_velocity, body_force


def analyze(directory):
    result = json.loads((directory/'result.json').read_text())
    log = ULog(str(directory/'px4.ulg'))
    clock = ClockMap(log)
    odom, references = read_bag(directory)
    gt_t, gt = series(log, 'vehicle_local_position_groundtruth', ['x','y','z','vx','vy','vz'])
    att_t, quat = series(log, 'vehicle_attitude_groundtruth', [f'q[{i}]' for i in range(4)])
    attitude = Slerp(att_t, Rotation.from_quat(quat[:, [1,2,3,0]]))
    gyro_t, gyro = series(log, 'vehicle_angular_velocity', [f'xyz[{i}]' for i in range(3)])
    command_t, command = series(log, 'vehicle_rates_setpoint', ['roll','pitch','yaw'], 'timestamp')
    arm = result['phases']['request_arm'].get('ros_start_ns')
    if arm is None:
        # Old flights can be analyzed geometrically, but no exact control-use trace exists.
        trace = list(csv.DictReader((directory/'trace.csv').open()))
        arm_rows = [r for r in trace if r['phase'] == 'request_arm']
        arm = (float(arm_rows[0]['reference_stamp_ns']) + float(arm_rows[0]['reference_elapsed_s'])*1e9)
    arm_s = arm*1e-9
    frame_events = validate_frame(log.get_dataset('vehicle_local_position').data,
                                  clock.native(arm_s-1.5)[0])
    resting = (odom[:,0] >= arm_s-1.5) & (odom[:,0] <= arm_s-.2)
    if np.sum(resting) < 20:
        raise ValueError('Insufficient pre-arm observations to fix world-to-estimator origin')
    truth_rest = interpolate(clock.native(odom[resting,0]), gt_t, gt)
    if np.max(np.linalg.norm(truth_rest[:,3:6], axis=1)) > .15:
        raise ValueError('Pre-arm origin window is not stationary')
    differences = odom[resting,3:6]-truth_rest[:,:3]
    origin = np.median(differences, axis=0)
    out = directory/'diagnostics'
    out.mkdir(exist_ok=True)
    report = {'flight_passed': result['passed'], 'clock': clock.quality,
              'ulog_dropouts': len(log.dropouts),
              'estimator_parameters': {key: log.initial_parameters.get(key) for key in
                  ('EKF2_GPS_DELAY','EKF2_GPS_CTRL','EKF2_EV_CTRL','EKF2_TAU_POS','EKF2_TAU_VEL')},
              'frame': {'fixed_prearm_origin_offset_ned_m': origin.tolist(),
                        'origin_offset_std_m': np.std(differences,axis=0).tolist(),
                        'reset_events_native_s': frame_events,
                        'method': 'stationary pre-arm translation only; no flight-error centering or rotation fit'},
              'truth_sample_interval_ms': summary(np.diff(gt_t)*1000), 'phases': {}}
    for phase in result['modes_requested']:
        info = result['phases'][phase]
        if not info.get('completed'):
            report['phases'][phase] = {'completed': False}
            continue
        if 'ros_start_ns' in info:
            begin, end = info['ros_start_ns']*1e-9, info['ros_end_ns']*1e-9
        else:
            rows = [r for r in trace if r['phase'] == phase]
            times = [float(r['reference_stamp_ns'])*1e-9+float(r['reference_elapsed_s']) for r in rows]
            begin, end = min(times), max(times)
        bounds = clock.native([begin,end])
        t = np.arange(bounds[0], bounds[1], .02)
        # Derivatives need real samples on both sides; no edge extrapolation or lag fitting.
        extended = np.r_[t[0]+np.arange(-3,0)*.02,t,t[-1]+np.arange(1,4)*.02]
        truth_ext = interpolate(extended, gt_t, gt)
        acceleration = savgol_filter(truth_ext[:,3:6], 7, 3, deriv=1, delta=.02, axis=0)[3:-3]
        truth = truth_ext[3:-3]
        position, velocity = truth[:,:3]+origin, truth[:,3:6]
        R = attitude(t).as_matrix()
        ros = clock.ros(t)
        ref_p, ref_v, ref_R, _ = evaluate_reference(ros, references)
        rate, actual_rate, desired_rate = rate_metrics(t,command_t,command,gyro_t,gyro)
        control, ages = controller_evidence(directory,clock,odom,gt_t,gt[:,:3],origin,begin,end)
        drag, residual, body_v, body_a = drag_evidence(t,velocity,R,acceleration,ref_v)
        est = interpolate(ros,odom[:,0],odom[:,3:9])
        data = {'time_s':t,'position':position,'velocity':velocity,
                'reference_position':ref_p,'reference_velocity':ref_v,
                'actual_rates':actual_rate,'desired_rates':desired_rate,
                'attitude_error':Rotation.from_matrix(np.einsum('nji,njk->nik',ref_R,R)).as_rotvec(),
                'model_residual':residual,'estimation_error':est[:,:3]-position}
        if ages is not None and ages[0][0] <= t[0] and ages[0][-1] >= t[-1]:
            data['state_age_ms'] = interpolate(t,ages[0],ages[1][:,None],hold=True)[:,0]
        metrics = analyze_phase(data,out/phase)
        peak = metrics['peak']
        peak['seconds_from_phase_start'] = peak['native_time_s']-bounds[0]
        peak['ros_time_ns'] = int(round(clock.ros(peak['native_time_s'])[0]*1e9))
        peak['seconds_from_flight_start'] = info['start_s']+(clock.ros(peak['native_time_s'])[0]-begin)
        metrics.update(controller_timing=control,inner_loop=rate,lateral_dynamics=drag,
                       legacy_receipt_time_position_rmse_m=info['position_error_rmse'],
                       edge_exclusion_s=0.0)
        # Sample-time estimator error is distinct from the held state used by the controller.
        metrics['estimator_at_sample_position_rmse_m'] = rms(est[:,:3]-position)
        metrics['estimator_at_sample_velocity_rmse_m_s'] = rms(est[:,3:]-velocity)
        speed_squared = np.sum(velocity*velocity,axis=1)
        cruise = (np.linalg.norm(np.gradient(ref_v,t,axis=0),axis=1)<.10) & (speed_squared>.25)
        metrics['estimator_equivalent_position_lead_ms_cruise'] = summary(
            1000*np.sum((est[:,:3]-position)[cruise]*velocity[cruise],axis=1)/speed_squared[cruise])
        metrics['estimator_lead_note'] = 'geometric position discrepancy / speed; not a measured transport delay or alignment correction'
        with (out/f'{phase}_dynamics.csv').open('w',newline='') as f:
            writer=csv.writer(f)
            writer.writerow(['native_s','body_vx','body_vy','body_vz','body_ax_minus_g','body_ay_minus_g','body_az_minus_g'])
            writer.writerows(zip(t,*body_v.T,*body_a.T))
        report['phases'][phase] = metrics
    (out/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(directory.name, json.dumps({k:{'rmse':v.get('full',{}).get('position_rmse_m'),
           'peak':v.get('peak',{}).get('position_error_m')} for k,v in report['phases'].items()}),flush=True)
    return report


def main():
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('path',type=Path,help='one flight or campaign root')
    args=cli.parse_args()
    directories=[args.path] if (args.path/'px4.ulg').exists() else sorted(p.parent for p in args.path.glob('*/px4.ulg'))
    if not directories:
        raise ValueError('No completed ULog evidence found')
    reports={directory.name:analyze(directory) for directory in directories}
    if len(directories)>1:
        (args.path/'diagnosis_comparison.json').write_text(json.dumps(reports,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':
    main()
