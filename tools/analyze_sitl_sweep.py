#!/usr/bin/env python3
"""Summarise SITL hover_to_hover runs: tracking, tilt, gaps and input saturation.

Usage (sourced ROS env not required):
  python3 tools/analyze_sitl_sweep.py DIR [DIR ...]

Each DIR must contain result.json, trace.csv and control_input.csv from
tools/sitl_ommpc_test.py. Prints one row per run plus per-speed aggregates so
repeatability (mean/std) and saturation are visible without opening the CSVs.
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
PHASE = 'hover_to_hover'


def load(directory):
    directory = Path(directory)
    result = json.loads((directory / 'result.json').read_text())
    phase = result['phases'][PHASE]
    thrust, rates, gaps = [], [], []
    with (directory / 'control_input.csv').open() as handle:
        for row in csv.DictReader(handle):
            if row['phase'] != PHASE:
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
        np.isclose(thrust, THRUST_LIMITS[1], atol=1e-9)
    if len(rates):
        saturated |= (np.abs(rates) >= RATE_LIMITS - 1e-9).any(axis=1)
    return {
        'dir': directory.name,
        'speed': float(re.search(r'(\d+(?:\.\d+)?)ms', directory.name).group(1)),
        'passed': bool(result['passed']),
        'pos_rmse': phase['position_error_rmse'],
        'pos_max': phase['position_error_max'],
        'vel_rmse': phase['velocity_error_rmse'],
        'vel_max': phase['velocity_error_max'],
        'tilt_max': phase['tilt_rad_max'],
        'thrust_max': phase['input_thrust_m_s2_abs_max'],
        'rate_max': max(phase['input_roll_rad_s_abs_max'], phase['input_pitch_rad_s_abs_max'],
                        phase['input_yaw_rad_s_abs_max']),
        'gap_max': max(gaps) if len(gaps) else float('nan'),
        'gaps_over_0_1s': int((gaps > 0.1).sum()),
        'saturated_samples': int(saturated.sum()),
        'saturated_fraction': float(saturated.mean()) if len(saturated) else float('nan'),
        'samples': int(len(thrust)),
        'violations': int(phase['control_bound_violations']),
    }


def main(paths):
    runs = [load(p) for p in paths]
    header = ('run', 'speed', 'pos_rmse', 'pos_max', 'vel_rmse', 'tilt_max', 'thrust_max',
              'rate_max', 'gap_max', 'gaps>0.1', 'sat_frac', 'viol', 'ok')
    print('  '.join(f'{h:>10s}' for h in header))
    for r in runs:
        print('  '.join(f'{v:>10}' if isinstance(v, str) else f'{v:>10.4f}'
                        if isinstance(v, float) else f'{v:>10d}'
                        for v in (r['dir'], r['speed'], r['pos_rmse'], r['pos_max'],
                                  r['vel_rmse'], r['tilt_max'], r['thrust_max'], r['rate_max'],
                                  r['gap_max'], r['gaps_over_0_1s'], r['saturated_fraction'],
                                  r['violations'], int(r['passed']))))
    print('\nPer-speed aggregates (n = repeats):')
    print('  '.join(f'{h:>10s}' for h in ('speed', 'n', 'pos_rmse_mean', 'pos_rmse_std',
                                          'tilt_mean', 'tilt_max', 'sat_frac_max', 'all_ok')))
    for speed in sorted({r['speed'] for r in runs}):
        group = [r for r in runs if r['speed'] == speed]
        rmse = np.array([g['pos_rmse'] for g in group])
        tilt = np.array([g['tilt_max'] for g in group])
        sat = np.array([g['saturated_fraction'] for g in group])
        print(f'{speed:>10.2f}  {len(group):>10d}  {rmse.mean():>13.4f}  {rmse.std(ddof=0):>11.4f}'
              f'  {tilt.mean():>9.4f}  {tilt.max():>8.4f}  {sat.max():>11.4f}'
              f'  {int(all(g["passed"] for g in group)):>6d}')
    return 0


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    sys.exit(main(sys.argv[1:]))
