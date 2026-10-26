#!/usr/bin/env python3
"""SITL-only alternating OFF/ON comparisons with fixed tuned OMMPC and quadratic thrust.

Use --suite rates for per-axis chirps/direct doublets and --suite tracking for
Helix, Sharp3D, S-curve and hover-to-hover. Results are never overwritten.
Requires existing simulator-only DDS/GCS tmux windows and sourced ROS overlays.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

from run_rate_identification import CONFIG, LOG_ROOT, PX4, SRC, nodes, stop_sim

CASES = {
    'helix-3.0': ['--mode','helix','--speed','3.0','--radius','1.6','--z-amp','0.40','--loops','2.0'],
    'sharp3d-3.85': ['--mode','sharp3d','--speed','3.85','--radius','1.65','--z-amp','0.60'],
    'scurve-3.9': ['--mode','scurve','--speed','3.9','--radius','1.6','--loops','2.0'],
    'h2h-4.0': ['--mode','hover_to_hover','--speed','4.0','--offset','4.5','0','0'],
}


def track(out, case, enabled):
    if out.exists():
        raise FileExistsError(out)
    nodes(False, enabled)
    before=set(LOG_ROOT.rglob('*.ulg'))
    px4_log = out.parent / (out.name + '.px4.log')
    flight_log_path = out.parent / (out.name + '.flight.log')
    with px4_log.open('x') as log:
        sim=subprocess.Popen(['bash','-lc',
            'source ~/.venvs/px4-v1.17/bin/activate && HEADLESS=1 make -j4 px4_sitl gz_super'],
            cwd=PX4,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+100
        while time.monotonic()<deadline:
            if 'Ready for takeoff' in px4_log.read_text(errors='replace'):
                break
            if sim.poll() is not None:
                raise RuntimeError('Simulator startup failed')
            time.sleep(1)
        else:
            raise RuntimeError('Simulator startup timeout')
        time.sleep(5)
        with flight_log_path.open('x') as flight_log:
            rc=subprocess.run(['python3',str(SRC/'tools/sitl_ommpc_test.py'),
                '--confirm-sitl','--vehicle','super',*CASES[case],
                '--abort-tilt','0.9','--max-pos-rmse','0.35','--max-vel-rmse','0.30',
                '--record-bag','--output-dir',str(out)],stdout=flight_log,stderr=subprocess.STDOUT).returncode
        result=json.loads((out/'result.json').read_text())
        if not result.get('landing',{}).get('landed_and_disarmed'):
            raise RuntimeError('Landing/disarm unconfirmed; leaving simulator and routing intact')
        time.sleep(3)
        logs=set(LOG_ROOT.rglob('*.ulg'))-before
        if len(logs)!=1:
            raise RuntimeError(f'Expected one ULog, found {len(logs)}')
        shutil.copy2(logs.pop(),out/'px4.ulg')
        shutil.copy2(CONFIG,out/'controller.yaml')
        (out/'manifest.json').write_text(json.dumps({
            'case':case,'rate_compensation_enabled':enabled,
            'gain':[1.08,1.08],'plant_time_constant_s':[.0275,.0275],
            'filter_time_constant_s':.020,'quadratic_thrust_model':True,
            'controller_sha256':hashlib.sha256(CONFIG.read_bytes()).hexdigest(),
            'returncode':rc},indent=2)+'\n')
        stop_sim()
        try:
            sim.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(sim.pid,signal.SIGTERM)
    print(out.name,'exit',rc,flush=True)
    if rc:
        raise RuntimeError('Flight failed; evidence retained, stopping campaign')


def main():
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--confirm-sitl',required=True,action='store_true')
    cli.add_argument('--suite',choices=['rates','tracking'],required=True)
    cli.add_argument('--output-root',type=Path,required=True)
    cli.add_argument('--repeats',type=int,default=3)
    cli.add_argument('--first-repeat',type=int,default=1)
    args=cli.parse_args()
    if args.repeats<1 or args.first_repeat<1:
        cli.error('repeat counts must be positive')
    args.output_root.mkdir(parents=True,exist_ok=True)
    for i in range(args.first_repeat,args.first_repeat+args.repeats):
        # Reverse order on alternate repeats to reduce ordering/load bias.
        for enabled in ([False,True] if i%2 else [True,False]):
            label='on' if enabled else 'off'
            if args.suite=='rates':
                cmd=['python3',str(SRC/'tools/run_rate_identification.py'),
                     '--confirm-sitl','--repeats','1','--step-validation',
                     '--output-root',str(args.output_root/f'{label}-{i}')]
                if enabled:
                    cmd.append('--rate-compensation')
                subprocess.run(cmd,check=True)
            else:
                for case in CASES:
                    track(args.output_root/f'{case}-{label}-r{i}',case,enabled)
    nodes(False,False)


if __name__=='__main__':
    main()
