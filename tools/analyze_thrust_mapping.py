#!/usr/bin/env python3
"""Compare px4_bridge linear vs quadratic thrust mapping from sitl_thrust_mapping_ab.sh runs.

Usage: python3 tools/analyze_thrust_mapping.py [RESULTS_ROOT]

Reads thrmap-{linear,quadratic}-<case>-r<k> directories. For every run it reports
the moving-phase tracking metrics plus thrust-specific evidence:
  - vertical (down) RMSE and mean bias over the moving phase,
  - measured specific-thrust gain: slope of a_meas.b3 vs commanded thrust,
    where a_meas is differentiated odometry velocity (1.0 means exact mapping).
"""
import csv
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

G = 9.80665
PHASES = ('helix', 'sharp3d', 'scurve', 'hover_to_hover')


def rotation(q):
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def thrust_metrics(directory, phase):
    rows = [r for r in csv.DictReader((directory/'trace.csv').open()) if r['phase'] == phase]
    t = np.array([int(r['px4_timestamp_us'])*1e-6 for r in rows])
    p = np.array([[float(r[k]) for k in ('north_m', 'east_m', 'down_m')] for r in rows])
    pr = np.array([[float(r[k]) for k in ('ref_n_m', 'ref_e_m', 'ref_d_m')] for r in rows])
    v = np.array([[float(r[k]) for k in ('vn_m_s', 've_m_s', 'vd_m_s')] for r in rows])
    vr = np.array([[float(r[k]) for k in ('ref_vn_m_s', 'ref_ve_m_s', 'ref_vd_m_s')] for r in rows])
    cmd = np.array([float(r['input_thrust_m_s2']) for r in rows])
    b3 = np.array([rotation([float(r[k]) for k in ('qw', 'qx', 'qy', 'qz')])[:, 2] for r in rows])
    moving = np.linalg.norm(vr, axis=1) > 0.05
    valid = moving & np.isfinite(cmd)
    # Body -Z specific thrust from measured acceleration (NED, gravity +down).
    accel = np.gradient(v, t, axis=0)
    measured = np.sum((np.array([0., 0., G]) - accel) * b3, axis=1)
    slope = float(np.polyfit(cmd[valid] - G, measured[valid] - G, 1)[0])
    ez = p[moving, 2] - pr[moving, 2]
    return {'z_rmse': float(np.sqrt(np.mean(ez**2))), 'z_bias': float(np.mean(ez)),
            'gain': slope}


def load(directory):
    result = json.loads((directory/'result.json').read_text())
    phase = next((k for k in PHASES if k in result['phases']), None)
    if phase is None:
        return {'passed': bool(result['passed']), 'missing_phase': True,
                'error': result.get('error', 'Requested trajectory was not reached')}
    m = result['phases'][phase]
    out = {'passed': bool(result['passed']),
           'rmse': m['position_error_rmse'], 'max': m['position_error_max'],
           'vel_rmse': m['velocity_error_rmse'], 'tilt': m['tilt_rad_max'],
           'thr_max': m['input_thrust_m_s2_max'], 'throttle_max': m['bridge_thrust_z_abs_max'],
           'solve_ms': m['solve_time_ms_mean']}
    out.update(thrust_metrics(directory, phase))
    return out


def main(root):
    groups = defaultdict(list)
    for d in sorted(root.glob('thrmap-*-r*')):
        mapping, case = d.name.split('-', 2)[1], d.name.split('-', 2)[2].rsplit('-r', 1)[0]
        groups[(case, mapping)].append(load(d))
    keys = ('rmse', 'max', 'vel_rmse', 'z_rmse', 'z_bias', 'gain', 'tilt', 'thr_max', 'throttle_max', 'solve_ms')
    print(f"{'case':16s} {'mapping':9s} {'n':>2s} {'ok':>3s} " + ' '.join(f'{k:>12s}' for k in keys))
    for case in sorted({c for c, _ in groups}):
        means = {}
        for mapping in ('linear', 'quadratic'):
            runs = groups.get((case, mapping), [])
            if not runs:
                continue
            observed = [r for r in runs if not r.get('missing_phase')]
            for r in runs:
                if r.get('missing_phase'):
                    print(f"{case} {mapping}: no trajectory samples: {r['error']}")
            ok = sum(r['passed'] for r in runs)
            if not observed:
                print(f'{case} {mapping}: {ok}/{len(runs)} passed; no trajectory metrics')
                continue
            mean = {k: float(np.mean([r[k] for r in observed])) for k in keys}
            std = {k: float(np.std([r[k] for r in observed])) for k in keys}
            means[mapping] = mean
            cells = ' '.join(f'{mean[k]:.4f}±{std[k]:.4f}' for k in keys)
            print(f'{case:16s} {mapping:9s} {len(runs):2d} {ok:3d} {cells} (metrics n={len(observed)})')
        if len(means) == 2:
            gain = lambda k: 100.0*(means['linear'][k]-means['quadratic'][k])/means['linear'][k]
            print(f"{'':16s} {'improve%':9s}        rmse {gain('rmse'):+.1f}%  max {gain('max'):+.1f}%  "
                  f"vel {gain('vel_rmse'):+.1f}%  z_rmse {gain('z_rmse'):+.1f}%")
    return 0


if __name__ == '__main__':
    sys.exit(main(Path(sys.argv[1]) if len(sys.argv) > 1 else
                  Path.home()/'uav_control_ws/ommpc-flight-results'))
