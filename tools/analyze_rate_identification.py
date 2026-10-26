#!/usr/bin/env python3
"""Fit per-axis rate response on PX4 ULog clocks; validate on independent doublets.

Models: K exp(-L*s)/(T*s+1) and K wn^2 exp(-L*s)/(s^2+2*zeta*wn*s+wn^2).
Uses actual vehicle_rates_setpoint (total feedback + excitation), NOT nominal
excitation alone. No attitude differentiation. Fractional delays use interpolated
ZOH command samples. Reported fine-grid fits do not imply sub-sample accuracy.
PX4 DDS translates incoming timestamps and clamps them to receipt time, so L is
an effective setpoint-timestamp-to-filtered-gyro delay, not isolated motor latency.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from pyulog import ULog
from scipy.optimize import least_squares
from scipy.signal import cont2discrete, lfilter

DT = 0.005


def simulate(parameters, kind, u):
    gain, delay, frequency, damping, bias = parameters
    grid = np.arange(len(u))*DT
    delayed = np.interp(grid-delay, grid, u, left=u[0])
    if kind == 'first_order':
        a = np.exp(-DT/frequency)
        response = lfilter([0., 1.-a], [1., -a], delayed)
    elif kind == 'second_order':
        num, den, _ = cont2discrete(([frequency**2], [1., 2*damping*frequency, frequency**2]), DT)
        response = lfilter(num.ravel(), den, delayed)
    else:
        response = delayed
    return gain*response+bias


def fit(u, y, kind):
    # Fit delay independently from the lag dynamics; do not label correlation lag as dead time.
    if kind == 'first_order':
        lower, upper = [.2, 0., .002, .999, -.3], [2., .12, .3, 1.001, .3]
        initial_frequency = .04
    elif kind == 'second_order':
        lower, upper = [.2, 0., 4., .15, -.3], [2., .12, 250., 2., .3]
        initial_frequency = 40.
    else:
        lower, upper = [.2, 0., .999, .999, -.3], [2., .12, 1.001, 1.001, .3]
        initial_frequency = 1.
    mask = slice(200, -100)
    best = None
    for delay in [.0, .02, .05, .09]:
        initial = [1., delay, initial_frequency, .7 if kind == 'second_order' else 1., 0.]
        result = least_squares(lambda p: simulate(p, kind, u)[mask]-y[mask], initial,
                               bounds=(lower, upper), max_nfev=160, diff_step=1e-3)
        if best is None or result.cost < best.cost:
            best = result
    return best.x


def scores(y, predicted):
    y, predicted = y[200:-100], predicted[200:-100]
    error = y-predicted
    centered = np.linalg.norm(y-np.mean(y))
    return {'rmse_rad_s': float(np.sqrt(np.mean(error**2))),
            'fit_percent': float(100*(1-np.linalg.norm(error)/centered)) if centered > 1e-8 else None,
            'output_rms_rad_s': float(np.sqrt(np.mean(y**2)))}


def load_run(directory):
    result = json.loads((directory/'result.json').read_text())
    if not result['passed']:
        raise ValueError(f'Flight failed: {directory}: {result.get("error")}')
    log = ULog(str(directory/'px4.ulg'))
    if log.dropouts:
        raise ValueError(f'ULog dropouts: {directory}: {len(log.dropouts)}')
    command = log.get_dataset('vehicle_rates_setpoint').data
    response = log.get_dataset('vehicle_angular_velocity').data
    timesync = log.get_dataset('timesync_status').data
    mask = timesync['source_protocol'] == 2
    sync_t = timesync['timestamp'][mask]*1e-6
    sync_offset = timesync['estimated_offset'][mask]*1e-6
    if len(sync_t) < 2:
        raise ValueError('Insufficient DDS timesync observations')
    ct = command['timestamp']*1e-6
    yt = response['timestamp_sample']*1e-6
    # Keep last message at duplicate timestamp, preserving the recorded time ordering.
    ckeep = np.r_[np.diff(ct)>0, True]
    ykeep = np.r_[np.diff(yt)>0, True]
    ct, yt = ct[ckeep], yt[ykeep]
    if np.any(np.diff(ct)<=0) or np.any(np.diff(yt)<=0):
        raise ValueError('Nonmonotonic ULog timestamp')
    windows = {}
    for axis, name in enumerate(('roll', 'pitch', 'yaw')):
        for kind in ('chirp', 'doublet'):
            phase = result['phases'][f'{name}_{kind}']
            ros_begin, ros_end = phase['ros_start_ns']*1e-9, phase['ros_end_ns']*1e-9
            estimate = ros_begin+np.median(sync_offset)
            offset = np.interp(estimate, sync_t, sync_offset)
            begin, end = ros_begin+offset, ros_end+offset
            if not (ct[0] < begin-1 and end+.5 < ct[-1] and yt[0] < begin-1 and end+.5 < yt[-1]):
                raise ValueError('Excitation timestamps do not overlap ULog; clock alignment invalid')
            t = np.arange(begin-1, end+.5, DT)
            u = command[name][ckeep][np.clip(np.searchsorted(ct,t,side='right')-1,0,len(ct)-1)]
            y = np.interp(t, yt, response[f'xyz[{axis}]'][ykeep])
            if np.std(u[200:-100]) < .025:
                raise ValueError(f'Insufficient rate excitation: {name}_{kind}')
            windows[(name, kind)] = (u.astype(float), y.astype(float))
            with (directory/f'{name}_{kind}_aligned.csv').open('w', newline='') as handle:
                writer = csv.writer(handle)
                writer.writerow(['px4_time_s', 'setpoint_rad_s', 'measured_rad_s'])
                writer.writerows(zip(t, u, y))
    timing = {'gyro_interval_ms_p5_p50_p95': np.percentile(np.diff(yt)*1000,[5,50,95]).tolist(),
              'setpoint_interval_ms_p5_p50_p95': np.percentile(np.diff(ct)*1000,[5,50,95]).tolist(),
              'gyro_publication_minus_sample_ms_p50_p95': np.percentile(
                  (response['timestamp'].astype(float)-response['timestamp_sample'])*.001,[50,95]).tolist(),
              'timesync_offset_ms_peak_to_peak': float(np.ptp(sync_offset)*1000),
              'dropouts': len(log.dropouts)}
    return windows, timing


def main(root):
    all_runs = {}
    for directory in sorted(root.glob('r[0-9]*')):
        windows, timing = load_run(directory)
        axes = {}
        for axis in ('roll','pitch','yaw'):
            u, y = windows[(axis,'chirp')]
            uv, yv = windows[(axis,'doublet')]
            models = {}
            for kind in ('delay_only', 'first_order', 'second_order'):
                params = fit(u, y, kind)
                models[kind] = {'gain': float(params[0]), 'delay_ms': float(params[1]*1000),
                    'time_constant_s' : float(params[2]) if kind == 'first_order' else None,
                    'natural_frequency_rad_s': float(params[2]) if kind == 'second_order' else None,
                    'damping': float(params[3]) if kind == 'second_order' else None,
                    'bias_rad_s': float(params[4]), 'parameters': params.tolist(),
                    'training': scores(y,simulate(params,kind,u)),
                    'validation': scores(yv,simulate(params,kind,uv))}
                print(directory.name,axis,kind,json.dumps(models[kind]),flush=True)
            axes[axis] = models
        all_runs[directory.name] = {'timing':timing,'axes':axes}
    # Cross-flight transfer: fit on r1, validate without refitting on r2/r3 doublets.
    if len(all_runs)>1:
        first = next(iter(all_runs))
        for run in list(all_runs)[1:]:
            windows, _ = load_run(root/run)
            for axis in ('roll','pitch','yaw'):
                u,y = windows[(axis,'doublet')]
                for kind in ('delay_only','first_order','second_order'):
                    params=all_runs[first]['axes'][axis][kind]['parameters']
                    all_runs[run]['axes'][axis][kind]['cross_run_validation']=scores(y,simulate(params,kind,u))
    (root/'identification.json').write_text(json.dumps(all_runs,indent=2,allow_nan=False)+'\n')
    return 0


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path)
    raise SystemExit(main(parser.parse_args().root))
