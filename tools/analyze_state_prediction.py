#!/usr/bin/env python3
"""Offline audit of causal state extrapolation, not a closed-loop benefit claim."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from pyulog import ULog
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation, Slerp

from analyze_tracking_diagnosis import ClockMap, series, interpolate, evaluate_reference, read_bag, rms, summary


def recorded_motion(directory):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(directory/'bag'), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()
             if t.name in ('/uav/state', '/uav/control_input')}
    states, controls = [], []
    while reader.has_next():
        topic, payload, receipt = reader.read_next()
        if topic not in types:
            continue
        m = deserialize_message(payload, types[topic])
        if topic == '/uav/state':
            states.append([m.timestamp_sample*1e-6, *m.position, *m.velocity, *m.q, *m.angular_velocity])
        else:
            controls.append([receipt*1e-9, m.thrust_acceleration])
    states, controls = np.asarray(states), np.asarray(controls)
    keep = np.r_[np.diff(states[:, 0]) != 0, True]
    states = states[keep]
    if np.any(np.diff(states[:, 0]) <= 0):
        raise ValueError('Nonmonotonic odometry samples')
    return states, controls


def analyze(directory):
    report = json.loads((directory/'diagnostics/report.json').read_text())
    phase, diagnostic = next(iter(report['phases'].items()))
    result = json.loads((directory/'result.json').read_text())
    info = result['phases'][phase]
    begin, end = info['ros_start_ns']*1e-9, info['ros_end_ns']*1e-9
    log = ULog(str(directory/'px4.ulg'))
    clock = ClockMap(log)
    rows = list(csv.DictReader((directory/'solve_timing.csv').open()))
    rows = [r for r in rows if begin <= int(r['finished_ros_ns'])*1e-9 <= end]
    sample = np.array([int(r['odometry_sample_us'])*1e-6 for r in rows])
    target = np.array([(int(r['finished_ros_ns'])-float(r['step_us'])*1000)*1e-9 for r in rows])
    age = target-sample
    states, controls = recorded_motion(directory)
    at = np.minimum(np.searchsorted(states[:, 0], sample), len(states)-1)
    previous = np.maximum(at-1, 0)
    at = np.where(abs(states[previous, 0]-sample) < abs(states[at, 0]-sample), previous, at)
    valid = ((abs(states[at, 0]-sample)<2e-6) & (age>=0) & (age<=.08) &
             np.all(np.isfinite(states[at]),axis=1) & (sample>=controls[0,0]))
    coverage = float(np.mean(valid))
    sample, target, age, state = sample[valid], target[valid], age[valid], states[at[valid]]
    t = clock.native(target)
    gt_t, gt = series(log, 'vehicle_local_position_groundtruth', ['x','y','z','vx','vy','vz'])
    truth = interpolate(t, gt_t, gt)
    truth[:,:3] += report['frame']['fixed_prearm_origin_offset_ned_m']
    att_t, att = series(log, 'vehicle_attitude_groundtruth', [f'q[{i}]' for i in range(4)])
    truth_R = Slerp(att_t, Rotation.from_quat(att[:,[1,2,3,0]]))(t)
    measured_R = Rotation.from_quat(state[:,[8,9,10,7]])
    omega = state[:,11:14]
    predicted_R = measured_R * Rotation.from_rotvec(omega*age[:,None])
    p, v = state[:,1:4], state[:,4:7]
    cv_p = p+age[:,None]*v
    # Conservative command availability: bag receipt precedes measurement sample.
    thrust = interpolate(sample,controls[:,0],controls[:,1:2],hold=True)[:,0]
    mid_R = measured_R * Rotation.from_rotvec(omega*age[:,None]*.5)
    acceleration = np.array([0.,0.,9.80665])-thrust[:,None]*mid_R.apply([0.,0.,1.])
    candidates = {'held':(p,v,measured_R),
                  'constant_velocity_measured_rate':(cv_p,v,predicted_R),
                  'past_thrust_measured_rate':(cv_p+.5*age[:,None]**2*acceleration,
                                              v+age[:,None]*acceleration,predicted_R)}
    _, references = read_bag(directory)
    _, ref_v, _, _ = evaluate_reference(target,references)
    moving = np.linalg.norm(ref_v,axis=1)>.05
    peak = diagnostic['peak']['native_time_s']
    masks = {'full':np.ones(len(t),dtype=bool),'moving':moving,'peak_window':abs(t-peak)<=1.}
    output = {'matched_valid_fraction':coverage,'samples':len(t),'age_ms':summary(age*1000),
              'target':'solve start reconstructed from finish ROS stamp minus recorded step duration',
              'assumptions':'Measured body rate held. Model acceleration uses only a command recorded before the state sample; no actual-rate command substitution or future measurements.',
              'limitation':'Offline state errors, not a prediction of closed-loop tracking improvement.', 'candidates':{}}
    columns = {'native_target_s':t,'age_ms':age*1000,'moving':moving.astype(int),'peak_window':masks['peak_window'].astype(int)}
    for name,(position,velocity,R) in candidates.items():
        ep, ev = position-truth[:,:3],velocity-truth[:,3:]
        er = (truth_R.inv()*R).as_rotvec()
        output['candidates'][name]={window:{'samples':int(np.sum(mask)),
            'position_rmse_m':rms(ep[mask]),'velocity_rmse_m_s':rms(ev[mask]),
            'attitude_rmse_rad':rms(er[mask]),'roll_pitch_rmse_rad':rms(er[mask,:2])}
            for window,mask in masks.items()}
        for label,error in [('position',ep),('velocity',ev),('attitude',er)]:
            columns[f'{name}_{label}_error_norm']=np.linalg.norm(error,axis=1)
    out=directory/'diagnostics'
    (out/'state_prediction_audit.json').write_text(json.dumps(output,indent=2,allow_nan=False)+'\n')
    with (out/'state_prediction_audit.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(columns);writer.writerows(zip(*columns.values()))
    print(directory.name,'coverage',coverage,flush=True)
    for name,metrics in output['candidates'].items():
        print(name,'moving',metrics['moving'],'peak',metrics['peak_window'],flush=True)
    return output


if __name__=='__main__':
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('root',type=Path)
    args=cli.parse_args()
    directories=[args.root] if (args.root/'px4.ulg').exists() else sorted(p.parent for p in args.root.glob('*/px4.ulg'))
    if not directories:
        raise ValueError('No flight ULogs')
    reports={p.name:analyze(p) for p in directories}
    if len(directories)>1:
        (args.root/'state_prediction_audit.json').write_text(json.dumps(reports,indent=2,allow_nan=False)+'\n')
