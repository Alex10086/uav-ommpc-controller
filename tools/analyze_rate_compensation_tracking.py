#!/usr/bin/env python3
"""Summarize fixed-parameter OFF/ON high-speed flights, retaining failures and saturation.
Usage: python3 tools/analyze_rate_compensation_tracking.py RESULTS_ROOT
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def load(directory):
    result=json.loads((directory/'result.json').read_text())
    manifest=json.loads((directory/'manifest.json').read_text())
    mode=result['modes_requested'][0]
    m=result['phases'].get(mode)
    base={'directory':directory.name,'passed':result['passed'],'error':result.get('error'),
          'case':manifest['case'],'enabled':manifest['rate_compensation_enabled'],
          'controller_sha256':manifest['controller_sha256']}
    if m is None:
        return base
    rows=[r for r in csv.DictReader((directory/'bridge_rates.csv').open()) if r['phase']==mode]
    applied=np.array([[float(r[k]) for k in ['roll','pitch','yaw']] for r in rows])
    bounds=np.array([3.,3.,1.5])
    base.update({
        'phase_complete':m['completed'],
        'position_rmse':m['position_error_rmse'],
        'moving_position_rmse':m['moving_only_position_rmse'],
        'position_max':m['position_error_max'],
        'velocity_rmse':m['velocity_error_rmse'],
        'tracking_lag_ms':m['tracking_lag_ms'],
        'tilt_max_rad':m['tilt_rad_max'],
        'reference_speed_max':m['ref_speed_m_s_max'],
        'actual_speed_max':m['speed_m_s_max'],
        'thrust_min':m['input_thrust_m_s2_min'],
        'thrust_max':m['input_thrust_m_s2_max'],
        'applied_rate_abs_max':np.max(np.abs(applied),axis=0).tolist() if len(applied) else None,
        'applied_rate_saturation_samples':int(np.sum(np.any(np.abs(applied)>=bounds-1e-6,axis=1))) if len(applied) else 0,
        'applied_rate_samples':len(applied),
        'solve_mean_ms':m['solve_time_ms_mean'],
        'solve_p95_ms':m['solve_time_ms_p95'],
        'solve_max_ms':m['solve_time_ms_max'],
        'recorded_command_gap_max_s':m['control_max_gap_s'],
        'bounds_violations':m['control_bound_violations']+m['rates_bound_violations']})
    return base


def main(root):
    runs=[load(p.parent) for p in sorted(root.glob('*/manifest.json'))]
    metrics=['position_rmse','moving_position_rmse','position_max','velocity_rmse',
             'tracking_lag_ms','tilt_max_rad','thrust_max','solve_mean_ms']
    groups=[]
    for case in sorted({r['case'] for r in runs}):
        for enabled in [False,True]:
            group=[r for r in runs if r['case']==case and r['enabled']==enabled]
            observed=[r for r in group if 'position_rmse' in r]
            summary={'case':case,'enabled':enabled,'attempts':len(group),
                     'passed':sum(r['passed'] for r in group),'metrics_samples':len(observed)}
            for field in metrics:
                values=[r[field] for r in observed if r[field] is not None]
                summary[field]={'mean':float(np.mean(values)) if values else None,
                    'sample_std':float(np.std(values,ddof=1)) if len(values)>1 else None}
            summary['worst_position_error']=max((r['position_max'] for r in observed),default=None)
            summary['rate_saturation_samples']=sum(r['applied_rate_saturation_samples'] for r in observed)
            groups.append(summary)
            print(json.dumps(summary),flush=True)
    (root/'tracking_comparison.json').write_text(json.dumps({'runs':runs,'groups':groups},indent=2,allow_nan=False)+'\n')


if __name__=='__main__':
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('root',type=Path)
    main(cli.parse_args().root)
