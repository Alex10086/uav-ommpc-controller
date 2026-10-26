#!/usr/bin/env python3
"""Compare desired (PRE-compensation) rates against actual gyro on OFF/ON runs.

Never use compensated PX4 setpoints as the desired rate: doing so would identify
the unchanged inner loop, rather than measure the precompensator's benefit.
Desired timestamps are relay ROS publish stamps mapped through PX4 timesync;
reported delays include bridge scheduling and clock uncertainty.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from pyulog import ULog


def distribution(values):
    values = np.asarray([x for x in values if x is not None], dtype=float)
    return {'n': len(values), 'mean': float(np.mean(values)) if len(values) else None,
            'p95': float(np.percentile(values,95)) if len(values) else None,
            'max': float(np.max(values)) if len(values) else None}


def crossing(t, y, level):
    hits=np.flatnonzero(y>=level)
    if not len(hits):
        return None
    i=int(hits[0])
    if i==0 or y[i]==y[i-1]:
        return float(t[i])
    return float(t[i-1]+(level-y[i-1])*(t[i]-t[i-1])/(y[i]-y[i-1]))


def analyze(directory, at_bridge=False):
    result=json.loads((directory/'result.json').read_text())
    if not result['passed']:
        return {'passed':False,'error':result.get('error')}
    log=ULog(str(directory/'px4.ulg'))
    gyro=log.get_dataset('vehicle_angular_velocity').data
    sync=log.get_dataset('timesync_status').data
    mask=sync['source_protocol']==2
    st=sync['timestamp'][mask]*1e-6
    offset=sync['estimated_offset'][mask]*1e-6
    rows=list(csv.DictReader((directory/'excitation.csv').open()))
    ros=np.array([int(r['ros_ns'])*1e-9 for r in rows])
    approx=ros+np.median(offset)
    ct=ros+np.interp(approx,st,offset)
    keep=np.r_[np.diff(ct)>0,True]
    ct=ct[keep];rows=[r for r,k in zip(rows,keep) if k]
    if at_bridge:
        matched=list(csv.DictReader((directory/'bridge_native_reference.csv').open()))
        ct=np.array([float(r['px4_s']) for r in matched])
        rows=[{'phase':r['phase'], **{'sent_'+a:r['desired_'+a] for a in ('roll','pitch','yaw')}}
              for r in matched]
    yt=gyro['timestamp_sample']*1e-6
    gkeep=np.r_[np.diff(yt)>0,True]
    yt=yt[gkeep]
    report={'passed':True,'dropouts':len(log.dropouts),
            'timebase':'matched_bridge_native' if at_bridge else 'relay_source_ros_mapped','axes':{}}
    for axis,name in enumerate(('roll','pitch','yaw')):
        desired=np.array([float(r['sent_'+name]) for r in rows])
        measured=gyro[f'xyz[{axis}]'][gkeep]
        which=np.array([r['phase']==name+'_chirp' for r in rows])
        start,end=ct[which][0],ct[which][-1]
        t=np.arange(start+1,end-1,.005)
        u=desired[np.clip(np.searchsorted(ct,t,side='right')-1,0,len(ct)-1)]
        y=np.interp(t,yt,measured)
        lags=np.arange(-.1,.2001,.001)
        errors=[]
        for lag in lags:
            shifted=desired[np.clip(np.searchsorted(ct,t-lag,side='right')-1,0,len(ct)-1)]
            errors.append(np.mean((y-shifted)**2))
        lag=float(lags[int(np.argmin(errors))])
        shifted=desired[np.clip(np.searchsorted(ct,t-lag,side='right')-1,0,len(ct)-1)]
        fit_gain=float(np.dot(shifted,y)/np.dot(shifted,shifted))
        data={'chirp_rmse_rad_s':float(np.sqrt(np.mean((y-u)**2))),
              'desired_rms_rad_s':float(np.sqrt(np.mean(u*u))),
              'normalized_rmse':float(np.sqrt(np.mean((y-u)**2)/np.mean(u*u))),
              'chirp_effective_lag_ms':lag*1000,
              'chirp_rms_gain':float(np.sqrt(np.mean(y*y)/np.mean(u*u))),
              'lag_aligned_gain':fit_gain}
        events=[]
        amplitude=(.22,.22,.18)[axis]
        for i,r in enumerate(rows):
            if r['phase']!=name+'_doublet':
                continue
            target=desired[i]
            # Exact plateaus exist only with --step-validation; no synthetic step assumption.
            if abs(abs(target)-amplitude)>1e-6:
                continue
            if i>0 and rows[i-1]['phase']==r['phase'] and abs(desired[i-1]-target)<1e-6:
                continue
            j=i+1
            while j<len(rows) and rows[j]['phase']==r['phase'] and abs(desired[j]-target)<1e-6:
                j+=1
            stop=ct[j] if j<len(ct) else ct[j-1]
            if stop-ct[i]<.22:
                continue
            before=(yt>=ct[i]-.025)&(yt<ct[i])
            baseline=float(np.mean(measured[before]))
            delta=target-baseline
            if abs(delta)<.08:
                continue
            response=(yt>=ct[i])&(yt<stop-.005)
            ts=yt[response]-ct[i]
            normalized=(measured[response]-baseline)/delta
            t10,t50,t90=(crossing(ts,normalized,x) for x in [.1,.5,.9])
            overshoot=max(0.,float(np.max(normalized)-1))*100
            plateau=(yt>=stop-.05)&(yt<stop-.005)
            plateau_error=100*(float(np.mean(measured[plateau]))/target-1)
            events.append({'start_px4_s':float(ct[i]),'target_rad_s':float(target),
                'baseline_rad_s':baseline,'t10_ms':None if t10 is None else t10*1000,
                't50_ms':None if t50 is None else t50*1000,
                't90_ms':None if t90 is None else t90*1000,
                'rise10_90_ms':None if t10 is None or t90 is None else (t90-t10)*1000,
                'overshoot_percent':overshoot,
                'plateau_relative_error_percent':plateau_error})
        data['steps']=events
        for field in ['t10_ms','t50_ms','t90_ms','rise10_90_ms','overshoot_percent','plateau_relative_error_percent']:
            data[field]=distribution([r[field] for r in events])
        report['axes'][name]=data
    return report


def main(root, at_bridge=False):
    reports={}
    for directory in sorted(root.glob('*/r[0-9]*')):
        key=str(directory.relative_to(root))
        reports[key]=analyze(directory, at_bridge)
        for axis,m in reports[key].get('axes',{}).items():
            print(key,axis,'RMSE',round(m['chirp_rmse_rad_s'],4),
                  'lag_ms',round(m['chirp_effective_lag_ms'],1),
                  'gain',round(m['lag_aligned_gain'],3),
                  'step_t50_ms',m['t50_ms']['mean'],
                  'step_t90_ms',m['t90_ms']['mean'],
                  'overshoot_pct',m['overshoot_percent']['mean'],flush=True)
    filename='comparison_bridge.json' if at_bridge else 'comparison.json'
    (root/filename).write_text(json.dumps(reports,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('root',type=Path)
    cli.add_argument('--at-bridge',action='store_true',
                     help='Use native-time matched bridge_reference data to exclude bridge scheduling wait')
    args=cli.parse_args()
    main(args.root,args.at_bridge)
