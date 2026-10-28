#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""FCW: 10Hz simulated ego (com2obu) approaches a slower lead car (RSM).

Sends both streams. Stop other RSM/RSI scenario publishers before testing.
No guarantee of an FCW event: final trigger depends on OBU firmware and configuration.
"""
import math
import sys
import time
from copy import deepcopy
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import (
    MQTTClient, OBU_HOST, OBU_PORT, OBU_USER, OBU_PASSWORD,
    RSU_MQTT_HOST, RSU_MQTT_PORT, SEND_HZ, bearing_deg, com2obu_payload,
    distance_m, dump_json, load_json, rsu_topic, set_participant
)

ORIGIN = (30.4630088, 114.1230776)
DIRECTION_POINT = (30.4626818, 114.1225879)
EGO_SPEED_KMH = 50.0
LEAD_SPEED_KMH = 15.0
INITIAL_GAP_M = 65.0
MIN_GAP_M = 7.0
DURATION_S = 10.0
REPEAT = 3
PAUSE_S = 2.0
RSM_REQID = "983966a4-59a2-4431-bd62-965d7ec8ee26"
JSON_FILE = Path(__file__).with_name("fcw.json")


def advance(origin, heading, meters):
    lat, lon = origin
    rad = math.radians(heading)
    east = meters * math.sin(rad)
    north = meters * math.cos(rad)
    return lat + north / 111195.0, lon + east / (111195.0 * math.cos(math.radians(lat)))


def run():
    base = load_json(JSON_FILE)
    if base["type"].lower() != "rsm" or base["value"]["participants"][0]["ptcType"] != 1:
        raise ValueError("fcw.json must contain a motor vehicle RSM participant (ptcType=1)")
    heading = bearing_deg(ORIGIN, DIRECTION_POINT)
    ego_ms = EGO_SPEED_KMH / 3.6
    lead_ms = LEAD_SPEED_KMH / 3.6
    closing_ms = ego_ms - lead_ms
    stop_time = (INITIAL_GAP_M - MIN_GAP_M) / closing_ms
    print("[FCW] dynamic vehicle following: 10Hz RSM + com2obu")
    print(f"[EGO] {EGO_SPEED_KMH}km/h, [LEAD] {LEAD_SPEED_KMH}km/h, heading={heading:.2f}deg")
    print(f"[GAP] initial={INITIAL_GAP_M}m min={MIN_GAP_M}m initial TTC={(INITIAL_GAP_M-MIN_GAP_M)/closing_ms:.2f}s")
    print("[NOTE] Stop any OTHER publisher sending RSM, com2obu or old scenario data.")
    print("[EXPECT] category=fFCW, event_sort=0 (OBU algorithm dependent)")
    obu = MQTTClient(OBU_HOST, OBU_PORT, OBU_USER, OBU_PASSWORD, "fcw_ego")
    rsu = MQTTClient(RSU_MQTT_HOST, RSU_MQTT_PORT, client_id="fcw_lead")
    obu.connect()
    last_ego = ORIGIN
    try:
        rsu.connect()
        try:
            period = 1 / SEND_HZ
            for loop in range(REPEAT):
                print(f"\n[LOOP] {loop + 1}/{REPEAT}")
                loop_begin = time.monotonic()
                for i in range(int(DURATION_S * SEND_HZ)):
                    elapsed = i * period
                    lead_progress = INITIAL_GAP_M + lead_ms * elapsed
                    ego_progress = min(ego_ms * elapsed, ego_ms * stop_time)
                    gap = lead_progress - ego_progress
                    ego_speed = EGO_SPEED_KMH if elapsed < stop_time else 0.0
                    ego = advance(ORIGIN, heading, ego_progress)
                    lead = advance(ORIGIN, heading, lead_progress)
                    last_ego = ego

                    msg = deepcopy(base)
                    msg["value"]["msgCnt"] = i % 128
                    p = msg["value"]["participants"][0]
                    p["secMark"] = int(time.time() * 1000) % 60000
                    set_participant(p, lead[0], lead[1], LEAD_SPEED_KMH, heading, 1)
                    rsu.publish(rsu_topic("rsm", RSM_REQID), dump_json(msg))
                    obu.publish("com2obu", dump_json(com2obu_payload(*ego, heading, ego_speed)))
                    if i % int(SEND_HZ) == 0:
                        ttc = (gap / closing_ms) if ego_speed > 0 and closing_ms > 0 else float("inf")
                        print(f"[t={elapsed:4.1f}s] gap={gap:5.1f}m TTC={ttc:4.1f}s "
                              f"relative={closing_ms:4.1f}m/s distance={distance_m(ego,lead):5.1f}m")
                    wait = loop_begin + (i + 1) * period - time.monotonic()
                    if wait > 0:
                        time.sleep(wait)
                for _ in range(int(PAUSE_S * SEND_HZ)):
                    obu.publish("com2obu", dump_json(com2obu_payload(*last_ego, heading, 0.0)))
                    time.sleep(period)
        except KeyboardInterrupt:
            print("\n[STOP] Ctrl+C")
        finally:
            try:
                obu.publish("com2obu", dump_json(com2obu_payload(*last_ego, heading, 0.0)))
            except OSError:
                pass
            rsu.close()
    finally:
        obu.close()


if __name__ == "__main__":
    run()
