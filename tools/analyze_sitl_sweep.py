#!/usr/bin/env python3
"""Summarise SITL flight runs: tracking, lag, tilt, inputs, saturation and solve time.

Usage (sourced ROS env not required):
  python3 tools/analyze_sitl_sweep.py DIR [DIR ...]

Each DIR must contain result.json, trace.csv and control_input.csv from
tools/sitl_ommpc_test.py. Prints one row per run plus per-speed aggregates so
repeatability (mean/std), lag and solve latency are visible without opening CSVs.
"""
import csv
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

THRUST_LIMITS = (1.0, 12.0)
RATE_LIMITS = np.array([3.0, 3.0, 1.5])
MOVING_PHASES = ('hover_to_hover', 'circle', 'figure8', 'helix', 'scurve', 'sharp3d', 'constant_velocity')


def compute_lag_from_trace(directory, phase_name):
    trace_path = Path(directory) / 'trace.csv'
    if not trace_path.is_file():
        return float('nan'), float('nan'), float('nan')
    t_list, p_list, pr_list, vr_list = [], [], [], []
    with trace_path.open() as handle:
        for row in csv.DictReader(handle):
            if row['phase'] != phase_name:
                continue
            try:
                t_list.append(float(row['time_s']))
                p_list.append([float(row['north_m']), float(row['east_m']), float(row['down_m'])])
                pr_list.append([float(row['ref_n_m']), float(row['ref_e_m']), float(row['ref_d_m'])])
                vr_list.append([float(row['ref_vn_m_s']), float(row['ref_ve_m_s']), float(row['ref_vd_m_s'])])
            except (KeyError, ValueError):
                continue
    if len(t_list) < 10:
        return float('nan'), float('nan'), float('nan')
    t_arr = np.array(t_list)
    p_arr = np.array(p_list)
    pr_arr = np.array(pr_list)
    vr_arr = np.array(vr_list)
    vr_norm = np.linalg.norm(vr_arr, axis=1)
    moving = vr_norm > 0.05
    if int(np.sum(moving)) < 10:
        return float('nan'), float('nan'), float('nan')
    p_mov = p_arr[moving]
    pr_mov = pr_arr[moving]
    t_mov = t_arr[moving]
    mov_rmse = float(np.sqrt(np.mean(np.sum((p_mov - pr_mov)**2, axis=1))))
    shifts = np.linspace(-0.35, 0.35, 701)
    best_dt = 0.0
    best_rmse = math.inf
    for dt in shifts:
        shifted = np.column_stack([np.interp(t_mov - dt, t_arr, pr_arr[:, ax]) for ax in range(3)])
        rmse = float(np.sqrt(np.mean(np.sum((p_mov - shifted)**2, axis=1))))
        if rmse < best_rmse:
            best_rmse = rmse
            best_dt = float(dt)
    return best_dt * 1000.0, best_rmse, mov_rmse


def load(directory):
    directory = Path(directory)
    result = json.loads((directory / 'result.json').read_text())
    phases = result.get('phases', {})
    phase_name = next((p for p in MOVING_PHASES if p in phases), 'hover_to_hover')
    phase = phases[phase_name]
    max_thrust = float(result.get('max_thrust_m_s2', THRUST_LIMITS[1]))
    thrust, rates, gaps = [], [], []
    with (directory / 'control_input.csv').open() as handle:
        for row in csv.DictReader(handle):
            if row['phase'] != phase_name:
                continue
            thrust.append(float(row['thrust_acceleration']))
            rates.append([float(row['roll']), float(row['pitch']), float(row['yaw'])])
            gap = float(row['gap_s'])
            if math.isfinite(gap):
                gaps.append(gap)
    thrust = np.array(thrust)
    rates = np.array(rates) if rates else np.zeros((0, 3))
    gaps = np.array(gaps)
    saturated = np.isclose(thrust, THRUST_LIMITS[0], atol=1e-9) | \
        np.isclose(thrust, max_thrust, atol=1e-9)
    if len(rates):
        saturated |= (np.abs(rates) >= RATE_LIMITS - 1e-9).any(axis=1)
    m = re.search(r'(\d+(?:\.\d+)?)ms', directory.name)
    speed = float(m.group(1)) if m else float(phase.get('ref_speed_m_s_max') or 0.0)
    lag_ms = phase.get('tracking_lag_ms')
    lag_comp_rmse = phase.get('lag_compensated_position_rmse')
    mov_rmse = phase.get('moving_only_position_rmse')
    if lag_ms is None or lag_comp_rmse is None:
        lag_ms, lag_comp_rmse, mov_rmse = compute_lag_from_trace(directory, phase_name)
    solve_mean = phase.get('solve_time_ms_mean')
    solve_max = phase.get('solve_time_ms_max')
    return {
        'dir': directory.name,
        'phase': phase_name,
        'speed': speed,
        'passed': bool(result['passed']),
        'pos_rmse': phase['position_error_rmse'],
        'mov_rmse': float('nan') if mov_rmse is None else float(mov_rmse),
        'pos_max': phase['position_error_max'],
        'vel_rmse': phase['velocity_error_rmse'],
        'vel_max': phase['velocity_error_max'],
        'lag_ms': float('nan') if lag_ms is None else float(lag_ms),
        'lag_comp_rmse': float('nan') if lag_comp_rmse is None else float(lag_comp_rmse),
        'tilt_max': phase['tilt_rad_max'],
        'thrust_min': float(thrust.min()) if len(thrust) else float('nan'),
        'thrust_max': phase['input_thrust_m_s2_abs_max'],
        'rate_max': max(phase['input_roll_rad_s_abs_max'], phase['input_pitch_rad_s_abs_max'],
                        phase['input_yaw_rad_s_abs_max']),
        'solve_ms_mean': float('nan') if solve_mean is None else float(solve_mean),
        'solve_ms_max': float('nan') if solve_max is None else float(solve_max),
        'gap_max': max(gaps) if len(gaps) else float('nan'),
        'gaps_over_0_1s': int((gaps > 0.1).sum()),
        'saturated_samples': int(saturated.sum()),
        'saturated_fraction': float(saturated.mean()) if len(saturated) else float('nan'),
        'samples': int(len(thrust)),
        'violations': int(phase['control_bound_violations']),
    }


def main(paths):
    runs = [load(p) for p in paths]
    header = ('run', 'speed', 'pos_rmse', 'pos_max', 'vel_rmse', 'lag_ms', 'lag_rmse',
              'tilt_max', 'thr_max', 'rate_max', 'slv_ms', 'slv_max', 'gap_max', 'ok')
    print('  '.join(f'{h:>10s}' for h in header))
    for r in runs:
        print('  '.join(f'{v:>10}' if isinstance(v, str) else f'{v:>10.4f}'
                        if isinstance(v, float) else f'{v:>10d}'
                        for v in (r['dir'], r['speed'], r['pos_rmse'], r['pos_max'],
                                  r['vel_rmse'], r['lag_ms'], r['lag_comp_rmse'],
                                  r['tilt_max'], r['thrust_max'], r['rate_max'],
                                  r['solve_ms_mean'], r['solve_ms_max'],
                                  r['gap_max'], int(r['passed']))))
    print('\nPer-speed aggregates (n = repeats):')
    print('  '.join(f'{h:>11s}' for h in ('speed', 'n', 'rmse_mean', 'rmse_std', 'max_err',
                                          'lag_ms_mean', 'tilt_max', 'thr_max', 'rate_max',
                                          'solve_ms', 'all_ok')))
    for speed in sorted({r['speed'] for r in runs}):
        group = [r for r in runs if r['speed'] == speed]
        rmse = np.array([g['pos_rmse'] for g in group])
        pmax = np.array([g['pos_max'] for g in group])
        lag = np.array([g['lag_ms'] for g in group])
        tilt = np.array([g['tilt_max'] for g in group])
        thr = np.array([g['thrust_max'] for g in group])
        rate = np.array([g['rate_max'] for g in group])
        slv = np.array([g['solve_ms_mean'] for g in group])
        print(f'{speed:>11.2f}  {len(group):>11d}  {rmse.mean():>11.4f}  {rmse.std(ddof=0):>11.4f}'
              f'  {pmax.max():>11.4f}  {np.nanmean(lag):>11.2f}  {tilt.max():>11.4f}'
              f'  {thr.max():>11.4f}  {rate.max():>11.4f}  {np.nanmean(slv):>11.4f}'
              f'  {int(all(g["passed"] for g in group)):>11d}')
    return 0


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    sys.exit(main(sys.argv[1:]))
