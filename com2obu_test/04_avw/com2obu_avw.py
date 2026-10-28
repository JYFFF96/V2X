#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AVW RSM-only feasibility test. Does not inject BSM or event2hmi."""
import argparse
import math
import sys
import time
from copy import deepcopy
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import (MQTTClient, OBU_HOST, OBU_PORT, OBU_USER, OBU_PASSWORD,
    RSU_MQTT_HOST, RSU_MQTT_PORT, SEND_HZ, bearing_deg, com2obu_payload,
    distance_m, dump_json, load_json, rsu_topic, set_participant)
from fcw_monitor import OBUCapture

ORIGIN=(30.4630088,114.1230776)
DIRECTION_POINT=(30.4626818,114.1225879)
JSON_FILE=Path(__file__).with_name("avw.json")
RSM_REQID="983966a4-59a2-4431-bd62-965d7ec8ee24"


def advance(origin,heading,meters):
    angle=math.radians(heading)
    return (origin[0]+meters*math.cos(angle)/111195.0,
            origin[1]+meters*math.sin(angle)/(111195.0*math.cos(math.radians(origin[0]))))


def main():
    parser=argparse.ArgumentParser(description="AVW static/stop RSM test")
    parser.add_argument("--mode",choices=["static","stop"],default="static",
                        help="static: parked vehicle, stop: moving vehicle stops")
    parser.add_argument("--gap",type=float,default=55.0,help="initial vehicle separation in metres")
    parser.add_argument("--ego-speed",type=float,default=30.0)
    parser.add_argument("--target-speed",type=float,default=12.0,
                        help="initial target speed for stop mode (km/h)")
    parser.add_argument("--duration",type=float,default=13.0)
    parser.add_argument("--min-gap",type=float,default=8.0)
    parser.add_argument("--log-dir",default="avw_logs")
    args=parser.parse_args()
    if args.gap<=args.min_gap or args.min_gap<=0 or args.ego_speed<=0 or args.duration<=0:
        parser.error("Require gap > min-gap > 0, ego-speed > 0 and duration > 0")
    if args.target_speed<0:parser.error("target-speed must be nonnegative")
    heading=bearing_deg(ORIGIN,DIRECTION_POINT)
    base=load_json(JSON_FILE)
    assert base["value"]["participants"][0]["ptcType"]==1
    monitor=OBUCapture(Path(args.log_dir)/("avw_"+datetime.now().strftime("%Y%m%d_%H%M%S")+".jsonl"))
    obu=MQTTClient(OBU_HOST,OBU_PORT,OBU_USER,OBU_PASSWORD,"avw_ego")
    rsu=MQTTClient(RSU_MQTT_HOST,RSU_MQTT_PORT,client_id="avw_target")
    monitor.start()
    last=ORIGIN
    try:
        obu.connect();rsu.connect()
        print(f"[AVW] mode={args.mode} gap={args.gap:.1f}m ego={args.ego_speed:.1f}km/h target={args.target_speed if args.mode=='stop' else 0:.1f}km/h")
        print("[NOTE] 10Hz RSM + com2obu. Stop other publishers first.")
        print("[NOTE] RSM-only AVW may not trigger if firmware requires BSM vehicle events.")
        print(f"[LOG] {monitor.output}")
        dt=1.0/SEND_HZ;count=int(args.duration*SEND_HZ)
        ego_s=0.;target_s=args.gap;ego_ms=args.ego_speed/3.6
        start=time.monotonic()
        for i in range(count):
            t=i*dt
            target_ms=(args.target_speed/3.6 if args.mode=="stop" and t<3.0 else 0.0)
            target=advance(ORIGIN,heading,target_s)
            ego=advance(ORIGIN,heading,ego_s);last=ego
            gap=target_s-ego_s
            # Preserve a minimum simulation separation; no real braking model implied.
            ego_now=min(ego_ms,max(0.0,target_ms+(gap-args.min_gap)/dt)) if gap>args.min_gap else min(ego_ms,target_ms)
            msg=deepcopy(base);msg["value"]["msgCnt"]=i%128
            p=msg["value"]["participants"][0]
            p["secMark"]=int(time.time()*1000)%60000
            set_participant(p,target[0],target[1],target_ms*3.6,heading,1)
            rsu.publish(rsu_topic("rsm",RSM_REQID),dump_json(msg))
            obu.publish("com2obu",dump_json(com2obu_payload(*ego,heading,ego_now*3.6)))
            closing=max(0.,ego_now-target_ms)
            if i%int(SEND_HZ)==0:
                ttc=gap/closing if closing>0.01 else float("inf")
                print(f"[t={t:4.1f}s] gap={gap:5.1f}m distance={distance_m(ego,target):5.1f}m "
                      f"ego={ego_now*3.6:5.1f}km/h target={target_ms*3.6:5.1f}km/h TTC={ttc:5.2f}s")
            ego_s+=ego_now*dt;target_s+=target_ms*dt
            wait=start+(i+1)*dt-time.monotonic()
            if wait>0:time.sleep(wait)
        obu.publish("com2obu",dump_json(com2obu_payload(*last,heading,0)))
    except KeyboardInterrupt:
        print("\n[STOP] interrupted")
    finally:
        obu.close();rsu.close();monitor.close()


if __name__=="__main__":
    main()
