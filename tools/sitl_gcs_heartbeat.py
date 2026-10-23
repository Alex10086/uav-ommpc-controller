#!/usr/bin/env python3
"""SITL-only GCS presence fixture. Sends no commands or parameter changes.

Run separately before the flight test:
  ~/.venvs/px4-v1.17/bin/python tools/sitl_gcs_heartbeat.py
The real heartbeat satisfies PX4's GCS-present preflight requirement without
changing any safety checks. Stop with Ctrl-C after the flight has landed.
"""
import time
from pymavlink import mavutil


def main():
    connection = mavutil.mavlink_connection(
        'udpout:127.0.0.1:18570', source_system=255, source_component=190)
    print('SITL test GCS heartbeat -> 127.0.0.1:18570 at 1 Hz; no commands/params', flush=True)
    try:
        while True:
            connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        connection.close()


if __name__ == '__main__':
    main()
