#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""VRUCW test: move only the OBU ego position toward a pedestrian already received in local.rinfo.

IMPORTANT: This script does NOT publish any RSM/RSI. Keep the known-good original
example/rsm.json publishing separately at 10Hz so its pedestrian remains visible.
"""
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import (
    MQTTClient, OBU_HOST, OBU_PORT, OBU_USER, OBU_PASSWORD,
    SEND_HZ, bearing_deg, com2obu_payload, distance_m, dump_json, interpolate
)

EGO_START = (30.4630088, 114.1230776)
PEDESTRIAN = (30.4626818, 114.1225879)
EGO_SPEED_KMH = 30.0
STOP_BEFORE_PED_M = 6.0
HOLD_SECONDS = 5.0
LOOPS = 3


def run():
    full_distance = distance_m(EGO_START, PEDESTRIAN)
    drive_distance = max(0.0, full_distance - STOP_BEFORE_PED_M)
    heading = bearing_deg(EGO_START, PEDESTRIAN)
    step_m = (EGO_SPEED_KMH / 3.6) / SEND_HZ
    frames = max(1, int(drive_distance / step_m) + 1)

    print("[VRUCW] OBU ego-only motion (no RSM/RSI publication)")
    print(f"[PED] lat={PEDESTRIAN[0]:.7f} lon={PEDESTRIAN[1]:.7f}")
    print(f"[EGO] start={EGO_START}, heading={heading:.1f} deg, speed={EGO_SPEED_KMH} km/h")
    print(f"[ROUTE] distance={full_distance:.1f} m, stop={STOP_BEFORE_PED_M:.1f} m before pedestrian")
    print("[NOTE] Keep publishing the original example/rsm.json at 10Hz in another terminal.")
    print("[EXPECT] event2hmi category=VRUCW, event_sort=10 (firmware dependent)")

    obu = MQTTClient(OBU_HOST, OBU_PORT, OBU_USER, OBU_PASSWORD, "com2obu_vrucw_ego")
    obu.connect()
    last = EGO_START
    try:
        for loop in range(LOOPS):
            print(f"\n[LOOP] {loop + 1}/{LOOPS}")
            for i in range(frames):
                traveled = min(i * step_m, drive_distance)
                ratio = traveled / full_distance if full_distance else 0.0
                last = interpolate(EGO_START, PEDESTRIAN, ratio)
                obu.publish("com2obu", dump_json(com2obu_payload(*last, heading, EGO_SPEED_KMH)))
                if i % int(SEND_HZ) == 0 or i == frames - 1:
                    print(f"  distance_to_ped={distance_m(last, PEDESTRIAN):.1f}m")
                time.sleep(1.0 / SEND_HZ)
            stop_ticks = int(HOLD_SECONDS * SEND_HZ)
            for _ in range(stop_ticks):
                obu.publish("com2obu", dump_json(com2obu_payload(*last, heading, 0.0)))
                time.sleep(1.0 / SEND_HZ)
    except KeyboardInterrupt:
        print("\n[STOP] Interrupted")
    finally:
        try:
            for _ in range(3):
                obu.publish("com2obu", dump_json(com2obu_payload(*last, heading, 0.0)))
                time.sleep(0.1)
        except OSError:
            pass
        obu.close()


if __name__ == "__main__":
    run()
