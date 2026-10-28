#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Dynamic pedestrian crossing: publish RSM to RSU and com2obu to OBU, both at 10Hz."""
import math
import sys
import time
from copy import deepcopy
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import (
    MQTTClient, OBU_HOST, OBU_PORT, OBU_USER, OBU_PASSWORD,
    RSU_MQTT_HOST, RSU_MQTT_PORT, SEND_HZ, bearing_deg,
    com2obu_payload, distance_m, dump_json, heading_raw, load_json,
    rsu_topic, set_participant
)

# Intersection coordinate is the same observed pedestrian location in local.rinfo.
CROSSING = (30.4626818, 114.1225879)
PREVIOUS_EGO = (30.4630088, 114.1230776)
EGO_SPEED_KMH = 30.0
PEDESTRIAN_SPEED_MS = 1.5
EGO_INITIAL_DISTANCE_M = 45.0
PEDESTRIAN_INITIAL_SIDE_M = 7.5
STOP_DISTANCE_M = 2.5
RUN_SECONDS = 12.0
REPEAT = 3
PAUSE_SECONDS = 3.0
JSON_PATH = Path(__file__).with_name("vrucw.json")
RSM_REQID = "983966a4-59a2-4431-bd62-965d7ec8ee27"


def offset_m(origin, east_m, north_m):
    lat, lon = origin
    return (
        lat + north_m / 111195.0,
        lon + east_m / (111195.0 * math.cos(math.radians(lat)))
    )


def run():
    base = load_json(JSON_PATH)
    target = base["value"]["participants"][0]
    if target["ptcType"] != 3:
        raise ValueError("Expected ptcType=3 pedestrian RSM")

    ego_heading = bearing_deg(PREVIOUS_EGO, CROSSING)
    h = math.radians(ego_heading)
    forward_e, forward_n = math.sin(h), math.cos(h)
    right_e, right_n = math.cos(h), -math.sin(h)
    start = offset_m(CROSSING, -forward_e * EGO_INITIAL_DISTANCE_M,
                     -forward_n * EGO_INITIAL_DISTANCE_M)
    ped_start = offset_m(CROSSING, -right_e * PEDESTRIAN_INITIAL_SIDE_M,
                         -right_n * PEDESTRIAN_INITIAL_SIDE_M)
    ped_heading = (ego_heading + 90) % 360
    ego_speed_ms = EGO_SPEED_KMH / 3.6
    ego_ttc_start = EGO_INITIAL_DISTANCE_M / ego_speed_ms
    ped_ttc_start = PEDESTRIAN_INITIAL_SIDE_M / PEDESTRIAN_SPEED_MS

    print("[VRUCW] dynamic crossing: 10 Hz RSM + com2obu")
    print(f"[EGO] speed={EGO_SPEED_KMH}km/h heading={ego_heading:.1f} start={start}")
    print(f"[PED] speed={PEDESTRIAN_SPEED_MS}m/s heading={ped_heading:.1f} start={ped_start}")
    print(f"[CROSS] {CROSSING}; initial time-to-cross: ego={ego_ttc_start:.1f}s ped={ped_ttc_start:.1f}s")
    print("[INFO] Stop any other RSM publishers before running this test.")
    print("[NOTE] No RSI neutralizer: previous JAM may still appear in event2hmi.")

    obu = MQTTClient(OBU_HOST, OBU_PORT, OBU_USER, OBU_PASSWORD, "vrucw_ego")
    rsu = MQTTClient(RSU_MQTT_HOST, RSU_MQTT_PORT, client_id="vrucw_ped")
    obu.connect()
    try:
        rsu.connect()
        try:
            period = 1.0 / SEND_HZ
            last_ego = start
            for iteration in range(REPEAT):
                print(f"\n[LOOP] {iteration + 1}/{REPEAT}")
                start_tick = time.monotonic()
                for i in range(int(RUN_SECONDS * SEND_HZ)):
                    t = i * period
                    traveled = min(ego_speed_ms * t, EGO_INITIAL_DISTANCE_M - STOP_DISTANCE_M)
                    ego_pos = offset_m(CROSSING,
                                       forward_e * (traveled - EGO_INITIAL_DISTANCE_M),
                                       forward_n * (traveled - EGO_INITIAL_DISTANCE_M))
                    last_ego = ego_pos
                    ped_traveled = PEDESTRIAN_SPEED_MS * t
                    ped_side = min(ped_traveled - PEDESTRIAN_INITIAL_SIDE_M, PEDESTRIAN_INITIAL_SIDE_M)
                    ped_pos = offset_m(CROSSING, right_e * ped_side, right_n * ped_side)
                    ego_speed = EGO_SPEED_KMH if traveled < EGO_INITIAL_DISTANCE_M - STOP_DISTANCE_M else 0.0
                    ped_speed = PEDESTRIAN_SPEED_MS if ped_side < PEDESTRIAN_INITIAL_SIDE_M else 0.0

                    msg = deepcopy(base)
                    msg["value"]["msgCnt"] = i % 128
                    p = msg["value"]["participants"][0]
                    p["secMark"] = int((time.time() * 1000) % 60000)
                    set_participant(p, ped_pos[0], ped_pos[1], ped_speed * 3.6, ped_heading, 3)
                    rsu.publish(rsu_topic("rsm", RSM_REQID), dump_json(msg))
                    obu.publish("com2obu", dump_json(com2obu_payload(*ego_pos, ego_heading, ego_speed)))

                    if i % int(SEND_HZ) == 0:
                        ego_to_cross = max(0.0, EGO_INITIAL_DISTANCE_M - traveled)
                        ped_to_cross = max(0.0, PEDESTRIAN_INITIAL_SIDE_M - ped_traveled)
                        ego_ttc = ego_to_cross / ego_speed_ms if ego_speed else float("inf")
                        ped_ttc = ped_to_cross / PEDESTRIAN_SPEED_MS if ped_speed else float("inf")
                        delta = abs(ego_ttc - ped_ttc) if math.isfinite(ego_ttc) and math.isfinite(ped_ttc) else float("inf")
                        print(f"[t={t:4.1f}s] distance={distance_m(ego_pos,ped_pos):5.1f}m "
                              f"ego_to_cross={ego_to_cross:4.1f}m ped_to_cross={ped_to_cross:4.1f}m "
                              f"TTC(ego/ped)={ego_ttc:4.1f}/{ped_ttc:4.1f}s delta={delta:4.1f}s")
                    remain = start_tick + (i + 1) * period - time.monotonic()
                    if remain > 0:
                        time.sleep(remain)
                # Keep the ego stopped during the inter-loop pause.
                for _ in range(int(PAUSE_SECONDS * SEND_HZ)):
                    obu.publish("com2obu", dump_json(com2obu_payload(*last_ego, ego_heading, 0)))
                    time.sleep(period)
        except KeyboardInterrupt:
            print("\n[STOP] Ctrl+C")
        finally:
            try:
                obu.publish("com2obu", dump_json(com2obu_payload(*last_ego, ego_heading, 0)))
            except OSError:
                pass
            rsu.close()
    finally:
        obu.close()


if __name__ == "__main__":
    run()
