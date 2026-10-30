#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SLW exploratory MAP lane-limit test using a vendor-provided map.json.

Requires the original complete MAP JSON; does not invent RTS sign types.
MAP speedLimits.speed is encoded in 0.02m/s (40km/h ~= 556).
"""
import argparse
import json
import socket
import struct
import threading
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from common import (MQTTClient, OBU_HOST, OBU_PORT, OBU_USER, OBU_PASSWORD,
    RSU_MQTT_HOST, RSU_MQTT_PORT, SEND_HZ, route_frames, speed_raw,
    com2obu_payload, dump_json, load_json, rsu_topic, mqtt_len, mqtt_str)
from fcw_monitor import OBUCapture

REQID="983966a4-59a2-4431-bd62-965d7ec8ee29"
ACK_TOPIC="command///res/+/200"


def choose_lane(map_message, link_index=2, lane_id=3):
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
    if not lane.get("maneuvers",{}).get("maneuverStraightAllowed"):
        print("[WARN] Selected lane does not advertise straight movement")
    return lane,points


def update_all_speed_limits(data, limit):
    """Change all existing MAP lane speed limits; don't add speculative fields."""
    raw=speed_raw(limit)
    lanes=0
    changed=0
    for node in data["value"]["nodes"]:
        for link in node.get("inLinks", []):
            for lane in link.get("lanes", []):
                lanes+=1
                for item in lane.get("speedLimits") or []:
                    item["speed"]=raw
                    changed+=1
    return lanes,changed


class RSUAckMonitor:
    """Read-only subscriber on local broker, records responses without interpreting as PC5 delivery."""
    def __init__(self, path):
        self.path=Path(path)
        self.stop=threading.Event()
        self.thread=threading.Thread(target=self.run,daemon=True)
        self.responses=0
        self.matched=0
        self.ready=threading.Event()

    @staticmethod
    def recv_exact(sock,n):
        out=bytearray()
        while len(out)<n:
            b=sock.recv(n-len(out))
            if not b:raise ConnectionError("socket closed")
            out.extend(b)
        return bytes(out)

    @classmethod
    def packet(cls,sock):
        head=cls.recv_exact(sock,1)[0]
        length=0;shift=0
        while True:
            b=cls.recv_exact(sock,1)[0]
            length+=(b&127)<<shift
            if not b&128:break
            shift+=7
            if shift>21:raise ValueError("MQTT remaining length invalid")
        return head,cls.recv_exact(sock,length)

    def start(self):
        self.thread.start()

    def run(self):
        self.path.parent.mkdir(parents=True,exist_ok=True)
        with self.path.open("w",encoding="utf-8") as out:
            try:
                with socket.create_connection((RSU_MQTT_HOST,RSU_MQTT_PORT),timeout=5) as sock:
                    sock.settimeout(5.0)
                    vh=mqtt_str("MQTT")+b"\\x04\\x02"+struct.pack("!H",60)
                    payload=mqtt_str("slw_rsu_ack_monitor")
                    sock.sendall(b"\\x10"+mqtt_len(len(vh)+len(payload))+vh+payload)
                    kind,body=self.packet(sock)
                    if kind>>4!=2 or len(body)<2 or body[1]!=0:raise RuntimeError("RSU broker CONNACK failed")
                    sub=struct.pack("!H",1)+mqtt_str(ACK_TOPIC)+b"\\x00"
                    sock.sendall(b"\\x82"+mqtt_len(len(sub))+sub)
                    self.ready.set()
                    last_ping=time.monotonic()
                    while not self.stop.is_set():
                        try:
                            kind,body=self.packet(sock)
                        except socket.timeout:
                            kind=0;body=b""
                        if kind>>4==3 and len(body)>=2:
                            n=int.from_bytes(body[:2],"big")
                            if len(body)<n+2:continue
                            topic=body[2:n+2].decode("utf-8","replace")
                            raw=body[n+2:].decode("utf-8","replace")
                            try:msg=json.loads(raw)
                            except ValueError:msg=None
                            reqid=msg.get("reqid") if isinstance(msg,dict) else None
                            self.responses+=1
                            matched=(reqid==REQID)
                            if matched:self.matched+=1
                            out.write(json.dumps({"time":time.time(),"topic":topic,"payload":raw,
                                "matches_slw_reqid":matched},ensure_ascii=False)+"\\n")
                            out.flush()
                            if matched:print("[RSU ACK]",raw,flush=True)
                        if time.monotonic()-last_ping>20:
                            sock.sendall(b"\\xc0\\x00");last_ping=time.monotonic()
            except (OSError,RuntimeError,ValueError,ConnectionError) as exc:
                print("[RSU ACK] monitor unavailable:",exc,flush=True)
                self.ready.set()

    def close(self):
        self.stop.set()
        self.thread.join(timeout=3)
        print(f"[RSU ACK] responses={self.responses} matched={self.matched} log={self.path}")


def main():
    parser=argparse.ArgumentParser(description="SLW via real MAP lane speedLimits + simulated ego")
    parser.add_argument("--map",required=True,help="path to the user's original map.json")
    parser.add_argument("--limit",type=float,default=40.0,help="test lane speed limit in km/h")
    parser.add_argument("--speed",type=float,default=60.0,help="OBU speed km/h; 30 for below-limit, 60 for over-limit")
    parser.add_argument("--link-index",type=int,default=2,help="zero-based inLinks index of first node")
    parser.add_argument("--lane-id",type=int,default=3)
    parser.add_argument("--loops",type=int,default=2)
    parser.add_argument("--log-dir",default="slw_logs")
    parser.add_argument("--map-interval",type=float,default=1.0,help="MAP publish interval in seconds")
    args=parser.parse_args()
    if min(args.limit,args.speed,args.map_interval)<=0 or args.loops<1:parser.error("speed/limit/loops must be positive")
    data=load_json(args.map)
    if data.get("type","").lower()!="map":raise ValueError("Expected map message")
    lane,points=choose_lane(data,args.link_index,args.lane_id)
    total_lanes,changed=update_all_speed_limits(data,args.limit)
    if not changed:raise ValueError("MAP has no existing speedLimits entries to update")
    frames=route_frames(points,args.speed,SEND_HZ)
    logpath=Path(args.log_dir)/("slw_"+datetime.now().strftime("%Y%m%d_%H%M%S")+".jsonl")
    capture=OBUCapture(logpath);capture.start()
    ack=RSUAckMonitor(logpath.with_name(logpath.stem+"_rsu_ack.jsonl"));ack.start()
    ack.ready.wait(timeout=2.0)
    sent_log=logpath.with_name(logpath.stem+"_map_sent.jsonl")
    obu=MQTTClient(OBU_HOST,OBU_PORT,OBU_USER,OBU_PASSWORD,"slw_ego")
    rsu=MQTTClient(RSU_MQTT_HOST,RSU_MQTT_PORT,client_id="slw_map")
    print(f"[SLW] lane={args.lane_id} link={args.link_index} points={len(points)} frames={len(frames)} straight={lane.get('maneuvers',{}).get('maneuverStraightAllowed',False)}")
    print("[SLW] SpeedLimitType 5 = vehicleMaxSpeed (general vehicle maximum); units 0.02m/s")
    print(f"[SLW] MAP lanes={total_lanes} limit_entries_updated={changed}")
    print(f"[SLW] limit={args.limit}km/h raw={speed_raw(args.limit)} ego={args.speed}km/h")
    print(f"[LOG] {logpath}")
    print("[NOTE] MAP speedLimits are broadcast through the RSU; SLW activation is firmware-dependent.")
    try:
        rsu.connect();obu.connect()
        with sent_log.open("w",encoding="utf-8") as sent:
            def publish_map():
                data["value"]["msgCnt"]=(data["value"].get("msgCnt",0)+1)%128
                payload=dump_json(data)
                rsu.publish(rsu_topic("map",REQID),payload)
                sent.write(json.dumps({"time":time.time(),"reqid":REQID,"msgCnt":data["value"]["msgCnt"],
                    "limit_kmh":args.limit,"lanes":total_lanes,"entries":changed},ensure_ascii=False)+"\\n")
                sent.flush()
            publish_map()
            next_map=time.monotonic()+args.map_interval
            time.sleep(2)
            for loop in range(args.loops):
                started=time.monotonic()
                print(f"[LOOP] {loop+1}/{args.loops}")
                for i,(lat,lon,hd) in enumerate(frames):
                    if time.monotonic()>=next_map:
                        publish_map()
                        next_map=time.monotonic()+args.map_interval
                    obu.publish("com2obu",dump_json(com2obu_payload(lat,lon,hd,args.speed)))
                    if i%int(SEND_HZ)==0:
                        print(f"[t={i/SEND_HZ:5.1f}s] lat={lat:.7f} lon={lon:.7f} heading={hd:.1f}")
                    wait=started+(i+1)/SEND_HZ-time.monotonic()
                    if wait>0:time.sleep(wait)
                time.sleep(2)
    except KeyboardInterrupt:
        print("[STOP] interrupted")
    finally:
        obu.close();rsu.close();capture.close();ack.close()
        print("[NOTE] RSU MQTT 200 does not prove MAP delivery to OBU.")


if __name__=="__main__":
    main()
