#!/usr/bin/env python3
"""Run three simulator-only rate identification flights; preserve bags and PX4 ULogs.
Source ROS/project overlays first. Requires existing sitl tmux DDS/GCS windows.
Do not run concurrently with other PX4 simulations or aircraft on this ROS domain.
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

HOME = Path.home()
SRC = Path(__file__).resolve().parent.parent
PX4 = HOME/'PX4-Autopilot'
CONFIG = SRC/'uav_ommpc_ros/config/ommpc_super_sitl.yaml'
LOG_ROOT = PX4/'build/px4_sitl_default/rootfs/log'
ENV = 'source ~/ros2-px4-env.bash && source ~/uav_control_ws/install/setup.bash && '


def nodes(experiment, rate_compensation=False, rate_filter=0.020, rate_gain=1.08):
    for name in ['bridge', 'ommpc']:
        subprocess.run(['tmux', 'kill-window', '-t', 'sitl:'+name], capture_output=True)
    commands = {
        'bridge': 'ros2 run uav_ommpc_ros px4_bridge --ros-args -p hover_thrust:=0.309128 -p quadratic_thrust_model:=true' +
                  f' -p rate_compensation_enabled:={str(rate_compensation).lower()}' +
                  f' -p rate_compensation_filter_time_constant:={rate_filter}' +
                  f' -p rate_compensation_gain:=\"[{rate_gain}, {rate_gain}]\"',
        'ommpc': f'ros2 run uav_ommpc_ros ommpc_node --ros-args --params-file {CONFIG}' +
                 (' -r /uav/control_input:=/uav/sysid/raw_control' if experiment else '')}
    for name, cmd in commands.items():
        subprocess.run(['tmux', 'new-window', '-t', 'sitl', '-n', name, ENV+cmd], check=True)
    time.sleep(3)


def stop_sim():
    subprocess.run(['pkill', '-INT', '-x', 'px4'], capture_output=True)
    time.sleep(2)
    subprocess.run(['pkill', '-TERM', '-f', '[g]z sim'], capture_output=True)
    time.sleep(2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--confirm-sitl', required=True, action='store_true')
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--rate-compensation', action='store_true')
    parser.add_argument('--step-validation', action='store_true')
    parser.add_argument('--rate-filter', type=float, default=0.020)
    parser.add_argument('--rate-gain', type=float, default=1.08)
    args = parser.parse_args()
    args.output_root.mkdir(parents=True, exist_ok=False)
    safe_to_restore = True
    try:
        for i in range(1, args.repeats+1):
            case = args.output_root/f'r{i}'
            nodes(True, args.rate_compensation, args.rate_filter, args.rate_gain)
            before = set(LOG_ROOT.rglob('*.ulg'))
            with (args.output_root/f'px4-r{i}.log').open('w') as log:
                sim = subprocess.Popen(['bash', '-lc',
                    'source ~/.venvs/px4-v1.17/bin/activate && HEADLESS=1 make -j4 px4_sitl gz_super'],
                    cwd=PX4, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                deadline = time.monotonic()+100
                while time.monotonic() < deadline:
                    if 'Ready for takeoff' in (args.output_root/f'px4-r{i}.log').read_text(errors='replace'):
                        break
                    if sim.poll() is not None:
                        raise RuntimeError('Simulator exited before ready')
                    time.sleep(1)
                else:
                    raise RuntimeError('Simulator startup timeout')
                time.sleep(5)
                safe_to_restore = False
                with (args.output_root/f'flight-r{i}.log').open('w') as flight_log:
                    result = subprocess.run(['python3', str(SRC/'tools/sitl_rate_identification.py'),
                        '--confirm-sitl', '--output-dir', str(case)] +
                        (['--step-validation'] if args.step_validation else []),
                        stdout=flight_log, stderr=subprocess.STDOUT)
                flight_result = json.loads((case/'result.json').read_text())
                safe_to_restore = bool(flight_result.get('landing', {}).get('landed_and_disarmed'))
                if not safe_to_restore:
                    raise RuntimeError('Landing/disarm not confirmed; leaving simulator and routing intact for recovery')
                time.sleep(3)
                logs = set(LOG_ROOT.rglob('*.ulg'))-before
                if len(logs) != 1:
                    raise RuntimeError(f'Expected one new ULog, found {len(logs)}')
                source = logs.pop()
                shutil.copy2(source, case/'px4.ulg')
                shutil.copy2(CONFIG, case/'controller.yaml')
                manifest = {'ulog_source': str(source), 'controller_sha256': hashlib.sha256(CONFIG.read_bytes()).hexdigest(),
                    'quadratic_thrust_model': True, 'hover_thrust': 0.309128,
                    'rate_compensation_enabled': args.rate_compensation,
                    'rate_compensation_filter_time_constant': args.rate_filter,
                    'rate_compensation_gain': [args.rate_gain, args.rate_gain],
                    'raw_control_topic': '/uav/sysid/raw_control', 'flight_returncode': result.returncode}
                (case/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
                print(json.dumps({'run': i, **manifest}), flush=True)
                stop_sim()
                try:
                    sim.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(sim.pid, signal.SIGTERM)
                if result.returncode:
                    raise RuntimeError('Identification flight failed; evidence retained, no automatic retry')
    finally:
        if safe_to_restore:
            nodes(False)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
