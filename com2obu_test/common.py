#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Shared helpers for static C-V2X OBU/RSU event tests."""
import json, math, socket, struct, time
from copy import deepcopy
from pathlib import Path

OBU_HOST="192.168.66.113"; OBU_PORT=1883; OBU_USER="dctek"; OBU_PASSWORD="123456"
RSU_MQTT_HOST="127.0.0.1"; RSU_MQTT_PORT=1883
RSU_VENDOR="dachuan"; RSU_DEVICE="DC887-002047"
SEND_HZ=10.0; EARTH_RADIUS_M=6371000.0

def mqtt_len(n):
    b=bytearray()
    while True:
        d=n%128; n//=128
        if n: d|=0x80
        b.append(d)
        if not n: return bytes(b)

def mqtt_str(s):
    b=s.encode("utf-8"); return struct.pack("!H",len(b))+b

class MQTTClient:
    def __init__(self,host,port=1883,user=None,password=None,client_id="cv2x_test"):
        self.host=host; self.port=port; self.user=user; self.password=password; self.client_id=client_id; self.sock=None
    def connect(self):
        self.close(); self.sock=socket.create_connection((self.host,self.port),timeout=5)
        flags=0x02 | (0x80 if self.user else 0) | (0x40 if self.password else 0)
        vh=mqtt_str("MQTT")+b"\x04"+bytes([flags])+struct.pack("!H",60)
        pl=mqtt_str(self.client_id)
        if self.user: pl+=mqtt_str(self.user)
        if self.password: pl+=mqtt_str(self.password)
        self.sock.sendall(b"\x10"+mqtt_len(len(vh)+len(pl))+vh+pl)
        r=self.sock.recv(4)
        if len(r)<4 or r[0]!=0x20 or r[3]!=0: raise RuntimeError("MQTT connect failed: %r"%r)
        self.sock.settimeout(None)
    def publish(self,topic,payload):
        if isinstance(payload,str): payload=payload.encode("utf-8")
        vh=mqtt_str(topic); pkt=b"\x30"+mqtt_len(len(vh)+len(payload))+vh+payload
        try: self.sock.sendall(pkt)
        except OSError:
            self.connect(); self.sock.sendall(pkt)
    def close(self):
        if self.sock:
            try:self.sock.sendall(b"\xe0\x00")
            except Exception:pass
            try:self.sock.close()
            except Exception:pass
            self.sock=None

def load_json(path):
    with open(path,"r",encoding="utf-8") as f:return json.load(f)

def dump_json(data):
    return json.dumps(data,ensure_ascii=False,separators=(",",":"))

def speed_raw(kmh): return int(round((kmh/3.6)/0.02))
def heading_raw(deg): return int(round((deg%360)/0.0125))

def distance_m(a,b):
    lat1,lon1=a; lat2,lon2=b
    p1=math.radians(lat1); p2=math.radians(lat2); dp=p2-p1; dl=math.radians(lon2-lon1)
    x=math.sin(dp/2)**2+math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return 2*EARTH_RADIUS_M*math.asin(math.sqrt(x))

def bearing_deg(a,b):
    p1=math.radians(a[0]); p2=math.radians(b[0]); dl=math.radians(b[1]-a[1])
    x=math.sin(dl)*math.cos(p2); y=math.cos(p1)*math.sin(p2)-math.sin(p1)*math.cos(p2)*math.cos(dl)
    return (math.degrees(math.atan2(x,y))+360)%360

def interpolate(a,b,ratio):
    return (a[0]+(b[0]-a[0])*ratio,a[1]+(b[1]-a[1])*ratio)

def route_frames(points,speed_kmh,hz=SEND_HZ):
    speed_ms=speed_kmh/3.6; step=speed_ms/hz; out=[]
    for i in range(len(points)-1):
        a,b=points[i],points[i+1]; dist=max(distance_m(a,b),0.001); n=max(1,int(math.ceil(dist/step))); hd=bearing_deg(a,b)
        for j in range(n):
            p=interpolate(a,b,j/n); out.append((p[0],p[1],hd))
    hd=bearing_deg(points[-2],points[-1]); out.append((points[-1][0],points[-1][1],hd))
    return out

def com2obu_payload(lat,lon,heading,speed_kmh):
    return {"longitude":int(round(lon*1e7)),"latitude":int(round(lat*1e7)),"heading":heading_raw(heading),
            "speed":speed_raw(speed_kmh),"LeftTurnLamp":0,"RightTurnLamp":0,"ABS_lamp":0,"E_lamp":0,
            "brakeBoost":0,"tcs":0,"scs":0,"source_gps":1}

def rsu_topic(kind,reqid):
    return f"command/{RSU_VENDOR}/{RSU_DEVICE}/req/{reqid}/{kind}"

def publish_rsu_once(json_path,kind,reqid):
    data=load_json(json_path); c=MQTTClient(RSU_MQTT_HOST,RSU_MQTT_PORT,client_id=f"test_{kind}")
    c.connect(); c.publish(rsu_topic(kind,reqid),dump_json(data)); c.close()
    print(f"[RSU] sent {kind}: {json_path}")

def active_path_from_rsi(data):
    value=data["value"]
    if value.get("rtes"): item=value["rtes"][0]
    elif value.get("rtss"): item=value["rtss"][0]
    else: raise ValueError("JSON has no rtes/rtss")
    pts=[]
    for x in item["referencePaths"][0]["activePath"]:
        p=x["offsetLL"]["position_LatLon"]; pts.append((p["lat"]/1e7,p.get("long",p.get("lng"))/1e7))
    return pts

def run_rsi(json_path,kind,reqid,speed_kmh=36.0,loops=3):
    data=load_json(json_path); publish_rsu_once(json_path,kind,reqid); pts=active_path_from_rsi(data)
    frames=route_frames(pts,speed_kmh); obu=MQTTClient(OBU_HOST,OBU_PORT,OBU_USER,OBU_PASSWORD,"com2obu_"+kind); obu.connect()
    print(f"[TEST] {kind.upper()} points={len(pts)} frames={len(frames)} speed={speed_kmh}km/h")
    try:
        for loop in range(loops):
            for i,(lat,lon,hd) in enumerate(frames):
                obu.publish("com2obu",dump_json(com2obu_payload(lat,lon,hd,speed_kmh)))
                print(f"\r[OBU] loop={loop+1}/{loops} frame={i+1}/{len(frames)} lat={lat:.7f} lon={lon:.7f} hd={hd:.1f}",end="",flush=True)
                time.sleep(1/SEND_HZ)
            print(); time.sleep(1)
    except KeyboardInterrupt: pass
    finally: obu.close()

def set_participant(p,lat,lon,speed_kmh,heading,ptc_type=None):
    if ptc_type is not None:p["ptcType"]=ptc_type
    pos=p["pos"]["offsetLL"]["position_LatLon"]; pos["lat"]=int(round(lat*1e7)); pos["long"]=int(round(lon*1e7))
    p["speed"]=speed_raw(speed_kmh); p["heading"]=heading_raw(heading)

def run_rsm(json_path,reqid,ego_points,ego_speed_kmh,duration_s,update_target,repeat=1):
    base=load_json(json_path); topic=rsu_topic("rsm",reqid)
    obu=MQTTClient(OBU_HOST,OBU_PORT,OBU_USER,OBU_PASSWORD,"com2obu_rsm"); rsu=MQTTClient(RSU_MQTT_HOST,RSU_MQTT_PORT,client_id="rsm_scenario")
    obu.connect(); rsu.connect(); frames=route_frames(ego_points,ego_speed_kmh); max_frames=max(1,int(duration_s*SEND_HZ))
    try:
        for loop in range(repeat):
            for i in range(max_frames):
                frame=frames[min(i,len(frames)-1)]; lat,lon,hd=frame; t=i/SEND_HZ
                data=deepcopy(base); update_target(data,t,(lat,lon,hd),i,max_frames)
                rsu.publish(topic,dump_json(data)); obu.publish("com2obu",dump_json(com2obu_payload(lat,lon,hd,ego_speed_kmh)))
                p=data["value"]["participants"][0]; pp=p["pos"]["offsetLL"]["position_LatLon"]
                print(f"\r[TEST] t={t:5.1f}s ego=({lat:.7f},{lon:.7f}) target=({pp['lat']/1e7:.7f},{pp['long']/1e7:.7f}) targetSpeed={p['speed']*0.02*3.6:.1f}km/h",end="",flush=True)
                time.sleep(1/SEND_HZ)
            print(); time.sleep(1)
    except KeyboardInterrupt: pass
    finally: obu.close(); rsu.close()
