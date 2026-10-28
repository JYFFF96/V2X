#!/usr/bin/env python3
"""Read-only MQTT monitor for FCW tests; no third-party Python packages."""
import json
import socket
import struct
import threading
import time
from pathlib import Path
from common import OBU_HOST, OBU_PORT, OBU_USER, OBU_PASSWORD, mqtt_len, mqtt_str

class OBUCapture:
    def __init__(self, output):
        self.output=Path(output);self.stop=threading.Event();self.thread=None
        self.counts={"local":0,"event2hmi":0,"fcw":0}

    @staticmethod
    def exact(sock,n):
        buf=bytearray()
        while len(buf)<n:
            data=sock.recv(n-len(buf))
            if not data: raise ConnectionError("MQTT stream closed")
            buf.extend(data)
        return bytes(buf)

    @classmethod
    def packet(cls,sock):
        head=cls.exact(sock,1)[0];shift=0;length=0
        while True:
            b=cls.exact(sock,1)[0];length+=(b&127)<<shift
            if not b&128:break
            shift+=7
            if shift>21:raise ValueError("invalid MQTT remaining length")
        return head,cls.exact(sock,length)

    def run(self):
        self.output.parent.mkdir(parents=True,exist_ok=True)
        with self.output.open("w",encoding="utf-8") as fp:
            while not self.stop.is_set():
                try:
                    with socket.create_connection((OBU_HOST,OBU_PORT),timeout=5) as sock:
                        sock.settimeout(1)
                        flags=0xc2
                        vh=mqtt_str("MQTT")+b"\x04"+bytes([flags])+struct.pack("!H",60)
                        pl=mqtt_str("fcw_capture")+mqtt_str(OBU_USER)+mqtt_str(OBU_PASSWORD)
                        sock.sendall(b"\x10"+mqtt_len(len(vh)+len(pl))+vh+pl)
                        sock.settimeout(5)
                        kind,payload=self.packet(sock)
                        if kind>>4!=2 or len(payload)<2 or payload[1]!=0:
                            raise RuntimeError("monitor MQTT CONNACK rejected")
                        for packet_id,topic in enumerate(("local","event2hmi"),1):
                            body=struct.pack("!H",packet_id)+mqtt_str(topic)+b"\x00"
                            sock.sendall(b"\x82"+mqtt_len(len(body))+body)
                        sock.settimeout(1)
                        last_ping=time.monotonic()
                        while not self.stop.is_set():
                            try:
                                kind,payload=self.packet(sock)
                            except socket.timeout:
                                kind=None;payload=b""
                            if kind is not None and kind>>4==3 and len(payload)>=2:
                                n=struct.unpack("!H",payload[:2])[0]
                                if len(payload)<2+n:continue
                                topic=payload[2:2+n].decode("utf-8","replace")
                                body=payload[2+n:]
                                if topic not in self.counts:continue
                                raw=body.decode("utf-8","replace")
                                try:decoded=json.loads(raw)
                                except ValueError:decoded=None
                                self.counts[topic]+=1
                                event=decoded.get("data",{}) if isinstance(decoded,dict) else {}
                                if topic=="event2hmi" and isinstance(event,dict):
                                    if event.get("category") in ("fFCW","FCW"):
                                        self.counts["fcw"]+=1
                                        print("[FCW EVENT]",raw,flush=True)
                                    elif event:
                                        print("[OTHER EVENT]",raw,flush=True)
                                fp.write(json.dumps({"recv_time":time.time(),"topic":topic,"raw":raw,
                                    "data":decoded},ensure_ascii=False)+"\n")
                                fp.flush()
                            if time.monotonic()-last_ping>20:
                                sock.sendall(b"\xc0\x00");last_ping=time.monotonic()
                except (OSError,RuntimeError,ValueError,ConnectionError) as exc:
                    if not self.stop.is_set():
                        print("[CAPTURE] reconnect:",exc,flush=True)
                        self.stop.wait(2)

    def start(self):
        self.thread=threading.Thread(target=self.run,name="fcw_mqtt_capture",daemon=True)
        self.thread.start()

    def close(self):
        self.stop.set()
        if self.thread:self.thread.join(timeout=3)
        print(f"[CAPTURE] {self.output} counts={self.counts}")
