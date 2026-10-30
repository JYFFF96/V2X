#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SLW exploratory MAP lane-limit test using a vendor-provided map.json.

Requires the original complete MAP JSON; does not invent RTS sign types.
MAP speedLimits.speed is encoded in 0.02m/s (40km/h ~= 556).
"""
import argparse
import sys
import time
from copy import deepcopy
from datetime import datetime
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from common import (MQTTClient, OBU_HOST, OBU_PORT, OBU_USER, OBU_PASSWORD,
    RSU_MQTT_HOST, RSU_MQTT_PORT, SEND_HZ, route_frames, speed_raw,
    com2obu_payload, dump_json, load_json, rsu_topic)
from fcw_monitor import OBUCapture

REQID="983966a4-59a2-4431-bd62-965d7ec8ee29"


def choose_lane(map_message, link_index=2, lane_id=4):
    nodes=map_message["value"]["nodes"]
    links=nodes[0]["inLinks"]
    if link_index>=len(links):raise ValueError("Invalid link index")
    candidates=[x for x in links[link_index]["lanes"] if x.get("laneID")==lane_id]
    if not candidates:raise ValueError("Lane does not exist")
    lane=candidates[0]
    points=[]
    for p in lane["points"]:
        pos=p["posOffset"]["offsetLL"]["position_LatLon"]
        points.append((pos["lat"]/1e7,pos["long"]/1e7))
    if len(points)<2:raise ValueError("MAP lane needs >=2 points")
    return lane,points


def main():
    parser=argparse.ArgumentParser(description="SLW via real MAP lane speedLimits + simulated ego")
    parser.add_argument("--map",required=True,help="path to the user's original map.json")
    parser.add_argument("--limit",type=float,default=40.0,help="test lane speed limit in km/h")
    parser.add_argument("--speed",type=float,default=60.0,help="OBU speed km/h; 30 for below-limit, 60 for over-limit")
    parser.add_argument("--link-index",type=int,default=2,help="zero-based inLinks index of first node")
    parser.add_argument("--lane-id",type=int,default=4)
    parser.add_argument("--loops",type=int,default=2)
    parser.add_argument("--log-dir",default="slw_logs")
    args=parser.parse_args()
    if min(args.limit,args.speed)<=0 or args.loops<1:parser.error("speed/limit/loops must be positive")
    data=load_json(args.map)
    if data.get("type","").lower()!="map":raise ValueError("Expected map message")
    lane,points=choose_lane(data,args.link_index,args.lane_id)
    if not lane.get("speedLimits"):
        lane["speedLimits"]=[{"type":5,"speed":speed_raw(args.limit)}]
    else:
        for item in lane["speedLimits"]:item["speed"]=speed_raw(args.limit)
    frames=route_frames(points,args.speed,SEND_HZ)
    logpath=Path(args.log_dir)/("slw_"+datetime.now().strftime("%Y%m%d_%H%M%S")+".jsonl")
    capture=OBUCapture(logpath);capture.start()
    obu=MQTTClient(OBU_HOST,OBU_PORT,OBU_USER,OBU_PASSWORD,"slw_ego")
    rsu=MQTTClient(RSU_MQTT_HOST,RSU_MQTT_PORT,client_id="slw_map")
    print(f"[SLW] lane={args.lane_id} link={args.link_index} points={len(points)} frames={len(frames)}")
    print(f"[SLW] limit={args.limit}km/h raw={speed_raw(args.limit)} ego={args.speed}km/h")
    print(f"[LOG] {logpath}")
    print("[NOTE] MAP speedLimits are broadcast through the RSU; SLW activation is firmware-dependent.")
    try:
        rsu.connect();obu.connect()
        rsu.publish(rsu_topic("map",REQID),dump_json(data))
        time.sleep(2)
        for loop in range(args.loops):
            started=time.monotonic()
            print(f"[LOOP] {loop+1}/{args.loops}")
            for i,(lat,lon,hd) in enumerate(frames):
                obu.publish("com2obu",dump_json(com2obu_payload(lat,lon,hd,args.speed)))
                if i%int(SEND_HZ)==0:print(f"[t={i/SEND_HZ:5.1f}s] lat={lat:.7f} lon={lon:.7f} heading={hd:.1f}")
                wait=started+(i+1)/SEND_HZ-time.monotonic()
                if wait>0:time.sleep(wait)
            time.sleep(2)
    except KeyboardInterrupt:
        print("[STOP] interrupted")
    finally:
        obu.close();rsu.close();capture.close()


if __name__=="__main__":
    main()
