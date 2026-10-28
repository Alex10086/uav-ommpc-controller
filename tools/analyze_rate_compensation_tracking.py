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
          'source_directory':str(directory.resolve()),
          'case':manifest['case'],'enabled':manifest['rate_compensation_enabled'],
          'state_prediction_enabled':manifest.get('state_prediction_enabled',False),
          'gps_delay_ms':manifest.get('gps_delay_ms'),
          'controller_sha256':manifest['controller_sha256']}
    if m is None:
        return base
    rows=[r for r in csv.DictReader((directory/'bridge_rates.csv').open()) if r['phase']==mode]
    applied=np.array([[float(r[k]) for k in ['roll','pitch','yaw']] for r in rows])
    bounds=np.array([3.,3.,1.5])
    base.update({
        'phase_complete':m['completed'],
        'error_time_basis':'legacy ROS receipt-time estimator comparison; not ground truth',
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
    diagnostic_path=directory/'diagnostics/report.json'
    if diagnostic_path.exists():
        diagnostic=json.loads(diagnostic_path.read_text())
        base['gps_delay_ms']=diagnostic.get('estimator_parameters',{}).get('EKF2_GPS_DELAY',base['gps_delay_ms'])
        aligned=diagnostic['phases'].get(mode,{})
        if 'peak' in aligned:
            base.update({
                'native_truth_position_rmse':aligned['full']['position_rmse_m'],
                'native_truth_moving_position_rmse':aligned['moving']['position_rmse_m'],
                'native_truth_velocity_rmse':aligned['full']['velocity_rmse_m_s'],
                'native_truth_position_max':aligned['peak']['position_error_m'],
                'native_truth_tangent_rmse':aligned['moving']['tangent_rmse_m'],
                'native_truth_cross_track_rmse':aligned['moving']['cross_track_rmse_m'],
                'native_truth_peak':aligned['peak'],
                'diagnostic_outputs':aligned['outputs']})
    return base


def main(root, additional_roots=()):
    roots=[root,*additional_roots]
    directories=[p.parent.resolve() for source in roots for p in sorted(source.glob('*/manifest.json'))]
    if len(set(directories)) != len(directories):
        raise ValueError('Repeated evidence directory would double-count flights')
    runs=[load(directory) for directory in directories]
    if len({r['controller_sha256'] for r in runs}) > 1:
        raise ValueError('Fixed-parameter comparison cannot pool different controller configurations')
    if len({r['gps_delay_ms'] for r in runs if r['gps_delay_ms'] is not None}) > 1:
        raise ValueError('Comparison cannot pool different GPS measurement-delay configurations')
    metrics=['position_rmse','moving_position_rmse','position_max','velocity_rmse',
             'tracking_lag_ms','tilt_max_rad','thrust_max','solve_mean_ms',
             'native_truth_position_rmse','native_truth_position_max',
             'native_truth_moving_position_rmse','native_truth_velocity_rmse',
             'native_truth_tangent_rmse','native_truth_cross_track_rmse']
    groups=[]
    keys=sorted({(r['case'],r['enabled'],r['state_prediction_enabled']) for r in runs})
    for case,enabled,predicted in keys:
        group=[r for r in runs if (r['case'],r['enabled'],r['state_prediction_enabled'])==(case,enabled,predicted)]
        observed=[r for r in group if 'position_rmse' in r]
        summary={'case':case,'enabled':enabled,'state_prediction_enabled':predicted,'attempts':len(group),
                 'passed':sum(r['passed'] for r in group),'metrics_samples':len(observed)}
        for field in metrics:
            values=[r[field] for r in observed if r.get(field) is not None]
            summary[field]={'mean':float(np.mean(values)) if values else None,
                'sample_std':float(np.std(values,ddof=1)) if len(values)>1 else None,
                'samples':len(values)}
        summary['worst_position_error']=max((r['position_max'] for r in observed),default=None)
        summary['worst_native_truth_position_error']=max(
            (r['native_truth_position_max'] for r in observed if 'native_truth_position_max' in r),default=None)
        summary['rate_saturation_samples']=sum(r['applied_rate_saturation_samples'] for r in observed)
        groups.append(summary)
        print(json.dumps(summary),flush=True)
    startup_failures=[{'root':str(source),**json.loads((source/'startup_failure.json').read_text())}
                      for source in roots if (source/'startup_failure.json').exists()]
    (root/'tracking_comparison.json').write_text(json.dumps({
        'roots':[str(source) for source in roots], 'runs':runs,'groups':groups,
        'reported_startup_failures':startup_failures},indent=2,allow_nan=False)+'\n')


if __name__=='__main__':
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('root',type=Path)
    cli.add_argument('--additional-root',type=Path,action='append',default=[],
                     help='Merge disjoint interrupted/recovery campaigns with the same controller config')
    args=cli.parse_args()
    main(args.root,args.additional_root)
