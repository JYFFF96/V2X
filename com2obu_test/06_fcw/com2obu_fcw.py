#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""FCW OBU test: one scenario or a 3x3 distance/speed sweep.

Runs RSM (lead vehicle) and com2obu (ego) simultaneously at 10 Hz.
No other com2obu or RSM publishers should be running. This only tests whether
the OBU firmware generates fFCW/event_sort=0; it never injects an event2hmi.
"""
import argparse
import math
import sys
import time
from datetime import datetime
from fcw_monitor import OBUCapture
from copy import deepcopy
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import (MQTTClient, OBU_HOST, OBU_PORT, OBU_USER, OBU_PASSWORD,
    RSU_MQTT_HOST, RSU_MQTT_PORT, SEND_HZ, bearing_deg, distance_m,
    com2obu_payload, dump_json, load_json, rsu_topic, set_participant)

ORIGIN = (30.4630088, 114.1230776)
DIRECTION_POINT = (30.4626818, 114.1225879)
RSM_REQID = "983966a4-59a2-4431-bd62-965d7ec8ee26"
JSON_FILE = Path(__file__).with_name("fcw.json")
DEFAULT_EGO_KMH = 50.0
SAFE_GAP_M = 8.0
BRAKING_START_GAP_M = 28.0
RISK_BRAKE_GAP_M = 14.0
PAUSE_S = 3.0


def advance(origin, heading, meters):
    rad = math.radians(heading)
    return (origin[0] + meters * math.cos(rad) / 111195.0,
            origin[1] + meters * math.sin(rad) /
            (111195.0 * math.cos(math.radians(origin[0]))))


def test_one(obu, rsu, template, heading, initial_gap, lead_kmh, ego_kmh, number, mode):
    dt = 1.0 / SEND_HZ
    lead_ms = lead_kmh / 3.6
    ego_ms = ego_kmh / 3.6
    relative = ego_ms - lead_ms
    if relative <= 0:
        raise ValueError("Ego must be faster than lead vehicle")
    brake_gap = BRAKING_START_GAP_M if mode == "normal" else RISK_BRAKE_GAP_M
    available = max(1.0, brake_gap - SAFE_GAP_M)
    brake_decel = max(2.0, relative * relative / (2.0 * available))
    duration = min(18.0, max(8.0, (initial_gap - brake_gap) / relative + 5.0))
    frames = int(math.ceil(duration * SEND_HZ))
    ego_s = 0.0
    lead_s = initial_gap
    current_ego_ms = ego_ms
    print(f"\n[CASE {number}] gap={initial_gap:.0f}m lead={lead_kmh:.0f}km/h "
          f"ego={ego_kmh:.0f}km/h mode={mode} heading={heading:.2f}deg "
          f"initial TTC={initial_gap/relative:.2f}s")
    started = time.monotonic()
    for i in range(frames):
        t = i * dt
        gap = lead_s - ego_s
        if gap <= brake_gap:
            current_ego_ms = max(lead_ms, current_ego_ms - brake_decel * dt)
        else:
            current_ego_ms = ego_ms
        # The scripted ego never crosses the lead vehicle.
        if gap - (current_ego_ms - lead_ms) * dt < SAFE_GAP_M:
            current_ego_ms = lead_ms + max(0.0, gap - SAFE_GAP_M) / dt
        lead_pos = advance(ORIGIN, heading, lead_s)
        ego_pos = advance(ORIGIN, heading, ego_s)
        msg = deepcopy(template)
        msg["value"]["msgCnt"] = i % 128
        participant = msg["value"]["participants"][0]
        participant["secMark"] = int(time.time() * 1000) % 60000
        set_participant(participant, lead_pos[0], lead_pos[1], lead_kmh, heading, 1)
        rsu.publish(rsu_topic("rsm", RSM_REQID), dump_json(msg))
        obu.publish("com2obu", dump_json(com2obu_payload(
            ego_pos[0], ego_pos[1], heading, current_ego_ms * 3.6)))
        closing = max(0.0, current_ego_ms - lead_ms)
        ttc = gap / closing if closing > 0.01 else float("inf")
        if i % int(SEND_HZ) == 0:
            print(f"[t={t:4.1f}s] gap={gap:5.1f}m "
                  f"distance={distance_m(ego_pos,lead_pos):5.1f}m "
                  f"ego={current_ego_ms*3.6:4.1f}km/h lead={lead_kmh:4.1f}km/h "
                  f"relative={closing:4.1f}m/s TTC={ttc:5.2f}s")
        lead_s += lead_ms * dt
        ego_s += current_ego_ms * dt
        wait = started + (i + 1) * dt - time.monotonic()
        if wait > 0:
            time.sleep(wait)
    # Publish the final speed/position; stay at that position while paused.
    final_ego = advance(ORIGIN, heading, ego_s)
    print(f"[CASE {number}] complete; gap={lead_s-ego_s:.1f}m")
    for _ in range(int(PAUSE_S * SEND_HZ)):
        obu.publish("com2obu", dump_json(com2obu_payload(
            final_ego[0], final_ego[1], heading, 0)))
        time.sleep(dt)
    return final_ego, heading


def main():
    parser = argparse.ArgumentParser(description="Dynamic FCW RSM + com2obu test")
    parser.add_argument("--gap", type=float, choices=[30.0, 50.0, 70.0], default=50.0)
    parser.add_argument("--lead-speed", type=float, choices=[0.0, 10.0, 20.0], default=10.0)
    parser.add_argument("--ego-speed", type=float, default=DEFAULT_EGO_KMH)
    parser.add_argument("--sweep", action="store_true", help="test all 9 combinations")
    parser.add_argument("--mode", choices=["normal", "risk"], default="normal")
    parser.add_argument("--log-dir", default="fcw_logs", help="directory for captured MQTT JSONL")
    args = parser.parse_args()
    if args.ego_speed <= max(20.0 if args.sweep else args.lead_speed, 0):
        parser.error("ego speed must exceed lead speed")
    template = load_json(JSON_FILE)
    assert template["value"]["participants"][0]["ptcType"] == 1
    heading = bearing_deg(ORIGIN, DIRECTION_POINT)
    combos = ([(g, s) for g in (30., 50., 70.) for s in (0., 10., 20.)]
              if args.sweep else [(args.gap, args.lead_speed)])
    print(f"[FCW] cases={len(combos)}  RSM + com2obu @ {SEND_HZ} Hz")
    print("[INFO] Stop other RSM/com2obu publishers. Expect fFCW/event_sort=0 if supported.")
    obu = MQTTClient(OBU_HOST, OBU_PORT, OBU_USER, OBU_PASSWORD, "fcw_ego")
    rsu = MQTTClient(RSU_MQTT_HOST, RSU_MQTT_PORT, client_id="fcw_lead")
    capture_path = Path(args.log_dir) / ("fcw_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".jsonl")
    capture = OBUCapture(capture_path)
    capture.start()
    print(f"[CAPTURE] local + event2hmi -> {capture_path}")
    obu.connect()
    last = None
    try:
        rsu.connect()
        try:
            for n, (gap, lead_speed) in enumerate(combos, 1):
                last = test_one(obu, rsu, template, heading, gap, lead_speed, args.ego_speed, n, args.mode)
        except KeyboardInterrupt:
            print("\n[STOP] interrupted")
        finally:
            if last:
                pos, hd = last
                try:
                    obu.publish("com2obu", dump_json(com2obu_payload(pos[0], pos[1], hd, 0)))
                except OSError:
                    pass
            rsu.close()
    finally:
        obu.close()
        capture.close()


if __name__ == "__main__":
    main()
