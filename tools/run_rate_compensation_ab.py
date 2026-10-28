#!/usr/bin/env python3
"""SITL-only alternating OFF/ON comparisons with fixed tuned OMMPC and quadratic thrust.

Use --suite rates for per-axis chirps/direct doublets, --suite tracking for
OFF/ON trajectories, or --suite diagnostics for fixed-OFF timing diagnosis.
Diagnostics also supports selected --cases and a restored --gps-delay-ms trial.
Results are never overwritten.
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

DIAGNOSTIC_CASES = {
    'line-1.0': ['--mode', 'constant_velocity', '--speed', '1.0'],
    'line-3.9': ['--mode', 'constant_velocity', '--speed', '3.9'],
    'sharp3d-3.85': CASES['sharp3d-3.85'],
    'scurve-3.9': CASES['scurve-3.9'],
}


def track(out, case, enabled, arguments=None, gps_delay_ms=None):
    if out.exists():
        raise FileExistsError(out)
    nodes(False, enabled)
    before=set(LOG_ROOT.rglob('*.ulg'))
    px4_log = out.parent / (out.name + '.px4.log')
    flight_log_path = out.parent / (out.name + '.flight.log')
    saved_parameters = {}
    if gps_delay_ms is not None:
        if subprocess.run(['pgrep', '-x', 'px4'], capture_output=True).returncode == 0:
            raise RuntimeError('Cannot back up SITL parameters while PX4 is running')
        backup = out.parent / (out.name + '.parameters-before')
        backup.mkdir()
        for name in ('parameters.bson', 'parameters_backup.bson'):
            path = LOG_ROOT.parent / name
            saved_parameters[path] = path.exists()
            if path.exists():
                shutil.copy2(path, backup / name)
    with px4_log.open('x') as log:
        sim=subprocess.Popen(['bash',str(SRC/'tools/start_super_sitl.sh'),
            str(int(gps_delay_ms)) if gps_delay_ms is not None else '0'],
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
                '--confirm-sitl','--vehicle','super',*(CASES[case] if arguments is None else arguments),
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
            'trajectory_arguments':CASES[case] if arguments is None else arguments,
            'gps_delay_override_ms':gps_delay_ms,
            'gps_delay_ms':0.0 if gps_delay_ms is None else gps_delay_ms,
            'gain':[1.08,1.08],'plant_time_constant_s':[.0275,.0275],
            'filter_time_constant_s':.020,'quadratic_thrust_model':True,
            'controller_sha256':hashlib.sha256(CONFIG.read_bytes()).hexdigest(),
            'returncode':rc},indent=2)+'\n')
        stop_sim()
        try:
            sim.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(sim.pid,signal.SIGTERM)
            sim.wait(timeout=10)
    if saved_parameters:
        if subprocess.run(['pgrep', '-x', 'px4'], capture_output=True).returncode == 0:
            raise RuntimeError('PX4 still running; preserving parameter backup without restoring live files')
        for path, existed in saved_parameters.items():
            if existed:
                shutil.copy2(backup / path.name, path)
                if path.read_bytes() != (backup / path.name).read_bytes():
                    raise RuntimeError('SITL parameter restoration failed')
            else:
                path.unlink(missing_ok=True)
        manifest=json.loads((out/'manifest.json').read_text())
        manifest['parameter_store_restored']=True
        manifest['parameter_backup_directory']=str(backup)
        (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(out.name,'exit',rc,flush=True)
    if rc:
        raise RuntimeError('Flight failed; evidence retained, stopping campaign')


def main():
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--confirm-sitl',required=True,action='store_true')
    cli.add_argument('--suite',choices=['rates','tracking','diagnostics'],required=True)
    cli.add_argument('--output-root',type=Path,required=True)
    cli.add_argument('--repeats',type=int,default=3)
    cli.add_argument('--first-repeat',type=int,default=1)
    cli.add_argument('--cases', choices=tuple(DIAGNOSTIC_CASES), nargs='+',
                     help='Diagnostic cases only; default runs all four')
    cli.add_argument('--gps-delay-ms', type=float, choices=[0.0,110.0],
                     help='Diagnostic-only override of normal 0 ms; restores prior SITL parameters after shutdown')
    args=cli.parse_args()
    if args.repeats<1 or args.first_repeat<1:
        cli.error('repeat counts must be positive')
    if (args.cases is not None or args.gps_delay_ms is not None) and args.suite != 'diagnostics':
        cli.error('--cases and --gps-delay-ms require --suite diagnostics')
    args.output_root.mkdir(parents=True,exist_ok=True)
    if args.suite == 'diagnostics':
        for i in range(args.first_repeat, args.first_repeat + args.repeats):
            for case in (args.cases or DIAGNOSTIC_CASES):
                arguments = DIAGNOSTIC_CASES[case]
                extra = ['--direction', str(1 if i % 2 else -1), '0', '0'] if case.startswith('line-') else []
                track(args.output_root / f'{case}-r{i}', case, False, arguments + extra, args.gps_delay_ms)
        nodes(False, False)
        return
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
