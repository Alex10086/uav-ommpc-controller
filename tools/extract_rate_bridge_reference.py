#!/usr/bin/env python3
"""Match PRE-compensation desired rates to PX4-native bridge output events.

Requires sourced ROS overlays plus pyulog/numpy. Only applies to this experiment:
yaw bypasses the Roll/Pitch compensator, so its float32 payload identifies the
source command. OFF runs additionally match all three rates. Applied triples
then match ULog setpoint payloads; timesync is only a nearby-event search hint.
Repeated identical payloads are ambiguous; preserve matching-quality diagnostics.
"""
import argparse
import bisect
import csv
import json
from pathlib import Path

import numpy as np
from pyulog import ULog
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

AXES=('roll','pitch','yaw')


def key(values):
    return np.asarray(values,dtype=np.float32).tobytes()


def extract(directory):
    enabled=json.loads((directory/'manifest.json').read_text())['rate_compensation_enabled']
    source_axes=('yaw',) if enabled else AXES
    injections={}
    for row in csv.DictReader((directory/'excitation.csv').open()):
        k=key([float(row['sent_'+a]) for a in source_axes])
        injections.setdefault(k,[]).append(row)
    stamps={k:[int(r['ros_ns']) for r in rows] for k,rows in injections.items()}
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(directory/'bag'),storage_id=''),
                rosbag2_py.ConverterOptions('',''))
    message_type=get_message('px4_msgs/msg/VehicleRatesSetpoint')
    matched=[]
    count=0
    ambiguous={}
    while reader.has_next():
        topic,data,_=reader.read_next()
        if topic!='/fmu/in/vehicle_rates_setpoint':
            continue
        m=deserialize_message(data,message_type)
        count+=1
        k=key([getattr(m,a) for a in source_axes])
        candidates=injections.get(k,[])
        clock=stamps.get(k,[])
        i=bisect.bisect_right(clock,int(m.timestamp)*1000+1000000)-1
        if i<0:
            continue
        r=candidates[i]
        age=(int(m.timestamp)*1000-int(r['ros_ns']))*1e-6
        if not -.999<age<100:
            continue
        first=bisect.bisect_left(clock,int(m.timestamp)*1000-100000000)
        distinct={key([float(c['sent_'+a]) for a in AXES]) for c in candidates[first:i+1]}
        if len(distinct)>1:
            ambiguous[r['phase']]=ambiguous.get(r['phase'],0)+1
        matched.append({'bridge_ros_us':int(m.timestamp),'source_ros_ns':int(r['ros_ns']),
            'phase':r['phase'],**{'desired_'+a:float(r['sent_'+a]) for a in AXES},
            **{'applied_'+a:float(getattr(m,a)) for a in AXES}})
    log=ULog(str(directory/'px4.ulg'))
    sp=log.get_dataset('vehicle_rates_setpoint').data
    sync=log.get_dataset('timesync_status').data
    mask=sync['source_protocol']==2
    t=sp['timestamp']*1e-6
    lookup={}
    for i in range(len(t)):
        lookup.setdefault(key([sp[a][i] for a in AXES]),[]).append(i)
    native=[]
    last=-1
    maximum_match_difference=0.
    for row in matched:
        ros=row['bridge_ros_us']*1e-6
        offset=np.interp(ros+np.median(sync['estimated_offset'][mask]*1e-6),
                         sync['timestamp'][mask]*1e-6,sync['estimated_offset'][mask]*1e-6)
        guess=ros+offset
        choices=lookup.get(key([row['applied_'+a] for a in AXES]),[])
        if not choices:
            continue
        i=min(choices,key=lambda index:abs(t[index]-guess))
        difference=abs(t[i]-guess)
        if difference>.04 or i<=last:
            continue
        maximum_match_difference=max(maximum_match_difference,difference)
        last=i
        native.append({'px4_s':float(t[i]),**row})
    if not native:
        raise ValueError('No exact-payload ULog matches; cannot estimate bridge-aligned response')
    for name,rows in [('bridge_reference.csv',matched),('bridge_native_reference.csv',native)]:
        with (directory/name).open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]))
            writer.writeheader();writer.writerows(rows)
    quality={'bag_rate_setpoints':count,'source_matches':len(matched),'native_matches':len(native),
             'native_match_fraction':len(native)/count,'ambiguous_source_by_phase':ambiguous,
             'maximum_native_match_hint_difference_ms':maximum_match_difference*1000,
             'note':'Matching removes bridge scheduling wait for diagnostics, not a replacement for end-to-end latency. Identical payloads and skipped logger samples limit event correspondence.'}
    (directory/'bridge_matching_quality.json').write_text(json.dumps(quality,indent=2)+'\n')
    print(str(directory),json.dumps(quality),flush=True)


if __name__=='__main__':
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('root',type=Path)
    for directory in sorted(cli.parse_args().root.glob('*/r[0-9]*')):
        extract(directory)
