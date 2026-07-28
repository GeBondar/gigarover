"""
rover_setup_web — first-time setup tool for a 4-motor VESC/CAN skid-steer rover.

This is a STANDALONE Flask app that owns the CAN bus directly (via python-can).
Run it BEFORE the ROS 2 driver (they cannot share the serial adapter). It lets
you:
  * scan the CAN bus for VESC controller IDs,
  * spin each raw ID at 5% to identify which physical wheel it is,
  * assign each ID a wheel position (FL/FR/RL/RR) and its "forward" direction,
  * enter the platform's kinematic parameters,
  * live-test differential (skid-steer) driving + telemetry charts,
  * export `motors.yaml` and `kinematics.yaml` consumed by rover_vesc_driver.

The CAN owner is a single background thread; HTTP handlers only mutate shared
state under a lock. A client heartbeat deadman stops the motors if the browser
goes away.
"""
import collections
import json
import os
import struct
import threading
import time

import can
from flask import Flask, jsonify, request, send_from_directory

try:
    from serial.tools import list_ports
except Exception:
    list_ports = None

# ---- configuration ---------------------------------------------------------
CH340_VID_PID = (0x1A86, 0x7523)          # CH340 USB-serial (seeed USB-CAN)
BITRATE = int(os.environ.get("VESC_BITRATE", "500000"))
LOOP_HZ = 50.0
IDENTIFY_DUTY = 0.05
DEADMAN_S = 0.5
IDENTIFY_MAX_S = 30.0
SCAN_S = 2.0
DEFAULT_MAX_ERPM = 1000
ERPM_LIMIT = 6000
HIST_HZ = 10.0
HIST_LEN = int(HIST_HZ * 40)

# VESC CAN packet types
P_SET_DUTY, P_SET_CURRENT, P_SET_RPM = 0, 1, 3
P_STATUS, P_STATUS_2, P_STATUS_4, P_STATUS_5 = 9, 14, 16, 27

HERE = os.path.dirname(os.path.abspath(__file__))
STATIC = os.path.join(HERE, "static")
# Default output: rover_bringup/config next to this package, else ./generated
_BRINGUP = os.path.normpath(os.path.join(HERE, "..", "..", "rover_bringup", "config"))
OUTPUT_DIR = os.environ.get(
    "ROVER_CONFIG_DIR", _BRINGUP if os.path.isdir(os.path.dirname(_BRINGUP)) else
    os.path.join(HERE, "generated"))
STATE_PATH = os.path.join(HERE, "setup_state.json")

POSITIONS = ["FL", "FR", "RL", "RR"]
POS_KEY = {"FL": "front_left", "FR": "front_right",
           "RL": "rear_left", "RR": "rear_right"}

app = Flask(__name__, static_folder=None)

lock = threading.Lock()
state = {
    "mode": "idle",           # idle | identify | drive | scan
    "identify_id": None,
    "throttle": 0.0,
    "steer": 0.0,
    "max_erpm": DEFAULT_MAX_ERPM,
    "last_beat": 0.0,
    "known_ids": [],
    "scan_result": None,
    "config": {},             # id(str) -> {"position","forward_sign"}
    "kinematics": {
        "wheel_radius": 0.0825,
        "wheel_separation": 0.40,
        "wheelbase": 0.50,
        "gear_ratio": 3.0,
        "motor_pole_pairs": 7,
        "max_wheel_speed": 3.0,
        "battery_cells": 6,
    },
    "bus_error": None,
    "channel": None,
    "tel": {},
}
history = collections.deque(maxlen=HIST_LEN)


# ---- port autodetect -------------------------------------------------------
def detect_channel():
    forced = os.environ.get("VESC_PORT")
    if forced:
        return forced
    if list_ports is not None:
        for p in list_ports.comports():
            if p.vid == CH340_VID_PID[0] and p.pid == CH340_VID_PID[1]:
                return p.device
        for p in list_ports.comports():           # fallback: any CH340 by name
            if "CH340" in (p.description or ""):
                return p.device
    return None                                    # no adapter present yet


# ---- persistence -----------------------------------------------------------
def load_state():
    if os.path.exists(STATE_PATH):
        try:
            with open(STATE_PATH, encoding="utf-8") as f:
                d = json.load(f)
            with lock:
                state["config"] = d.get("config", {})
                state["max_erpm"] = d.get("max_erpm", DEFAULT_MAX_ERPM)
                state["kinematics"].update(d.get("kinematics", {}))
        except Exception as e:
            print("state load failed:", e)


def save_state():
    with lock:
        d = {"config": state["config"], "max_erpm": state["max_erpm"],
             "kinematics": state["kinematics"]}
    tmp = STATE_PATH + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(d, f, indent=2)
    os.replace(tmp, STATE_PATH)


# ---- CAN senders -----------------------------------------------------------
def _send(bus, ptype, cid, payload):
    bus.send(can.Message(arbitration_id=(ptype << 8) | cid,
                         is_extended_id=True, data=payload))


def send_duty(bus, cid, duty):
    _send(bus, P_SET_DUTY, cid, struct.pack(">i", int(duty * 100000.0)))


def send_current(bus, cid, amps):
    _send(bus, P_SET_CURRENT, cid, struct.pack(">i", int(amps * 1000.0)))


def send_rpm(bus, cid, erpm):
    _send(bus, P_SET_RPM, cid, struct.pack(">i", int(erpm)))


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


# ---- telemetry decode ------------------------------------------------------
def decode_frame(msg):
    if not msg.is_extended_id:
        return
    ptype = (msg.arbitration_id >> 8) & 0xFF
    cid = str(msg.arbitration_id & 0xFF)
    d = msg.data
    upd = None
    try:
        if ptype == P_STATUS and len(d) >= 8:
            upd = {"erpm": struct.unpack(">i", d[0:4])[0],
                   "current": struct.unpack(">h", d[4:6])[0] / 10.0,
                   "duty": struct.unpack(">h", d[6:8])[0] / 1000.0}
        elif ptype == P_STATUS_2 and len(d) >= 4:
            upd = {"amp_hours": struct.unpack(">i", d[0:4])[0] / 1e4}
        elif ptype == P_STATUS_4 and len(d) >= 6:
            upd = {"temp_fet": struct.unpack(">h", d[0:2])[0] / 10.0,
                   "temp_motor": struct.unpack(">h", d[2:4])[0] / 10.0,
                   "current_in": struct.unpack(">h", d[4:6])[0] / 10.0}
        elif ptype == P_STATUS_5 and len(d) >= 6:
            upd = {"v_in": struct.unpack(">h", d[4:6])[0] / 10.0}
    except struct.error:
        return
    if upd:
        with lock:
            state["tel"].setdefault(cid, {}).update(upd)
            state["tel"][cid]["t"] = time.time()


def sample_history():
    now = time.time()
    with lock:
        tel = {k: dict(v) for k, v in state["tel"].items()}
    vs = [v["v_in"] for v in tel.values() if "v_in" in v]
    iins = [v["current_in"] for v in tel.values() if "current_in" in v]
    ahs = [v["amp_hours"] for v in tel.values() if "amp_hours" in v]
    history.append({
        "t": now,
        "v": round(sum(vs) / len(vs), 2) if vs else None,
        "i": round(sum(iins), 2) if iins else None,
        "ah": round(sum(ahs), 4) if ahs else None,
        "m": {c: {"i": v.get("current"), "rpm": v.get("erpm")}
              for c, v in tel.items()},
    })


# ---- CAN owner thread ------------------------------------------------------
def open_bus():
    """Block until the CH340 adapter is found and the bus opens. Re-detects the
    port each attempt so a replug on a new COM number is picked up."""
    while True:
        channel = detect_channel()
        with lock:
            state["channel"] = channel
        if channel is None:
            with lock:
                state["bus_error"] = "no CH340 adapter found"
            time.sleep(2.0)
            continue
        try:
            bus = can.Bus(interface="seeedstudio", channel=channel,
                          bitrate=BITRATE, timeout=0.1)
            with lock:
                state["bus_error"] = None
            return bus
        except Exception as e:
            with lock:
                state["bus_error"] = repr(e)
            time.sleep(2.0)


def can_thread():
    bus = open_bus()
    dt = 1.0 / LOOP_HZ
    identify_started = 0.0
    last_hist = 0.0
    while True:
        t = time.time()
        with lock:
            mode = state["mode"]
            iid = state["identify_id"]
            throttle, steer = state["throttle"], state["steer"]
            max_erpm = state["max_erpm"]
            last_beat = state["last_beat"]
            cfg = dict(state["config"])
            known = list(state["known_ids"])

        if mode in ("identify", "drive") and (t - last_beat) > DEADMAN_S:
            with lock:
                state["mode"] = "idle"
            mode = "idle"

        try:
            if mode != "scan":
                for _ in range(10):
                    m = bus.recv(timeout=0)
                    if m is None:
                        break
                    decode_frame(m)

            if mode == "scan":
                seen = {}
                t0 = time.time()
                while time.time() - t0 < SCAN_S:
                    m = bus.recv(timeout=0.1)
                    if m is not None and m.is_extended_id:
                        seen[m.arbitration_id & 0xFF] = 1
                        decode_frame(m)
                with lock:
                    state["known_ids"] = sorted(seen.keys())
                    state["scan_result"] = state["known_ids"]
                    state["mode"] = "idle"

            elif mode == "identify":
                if identify_started == 0.0:
                    identify_started = t
                if t - identify_started > IDENTIFY_MAX_S:
                    with lock:
                        state["mode"] = "idle"
                    identify_started = 0.0
                else:
                    if iid is not None:
                        send_duty(bus, iid, IDENTIFY_DUTY)
                    for cid in known:
                        if cid != iid:
                            send_current(bus, cid, 0.0)

            elif mode == "drive":
                identify_started = 0.0
                left = clamp(throttle + steer, -1.0, 1.0)
                right = clamp(throttle - steer, -1.0, 1.0)
                for id_str, mc in cfg.items():
                    side = mc.get("position", "FL")[1]
                    base = left if side == "L" else right
                    send_rpm(bus, int(id_str),
                             mc.get("forward_sign", 1) * base * max_erpm)

            else:
                identify_started = 0.0
                for cid in set(int(k) for k in cfg) | set(known):
                    send_current(bus, cid, 0.0)

            if t - last_hist >= 1.0 / HIST_HZ:
                sample_history()
                last_hist = t
        except Exception as e:
            # Stale handle / adapter re-enumerated (e.g. "Access denied" on
            # write) — close and reopen so the site self-heals instead of
            # freezing on a dead port.
            with lock:
                state["bus_error"] = repr(e)
            try:
                bus.shutdown()
            except Exception:
                pass
            bus = open_bus()
            identify_started = 0.0

        el = time.time() - t
        if el < dt:
            time.sleep(dt - el)


# ---- YAML export -----------------------------------------------------------
def wheels_yaml():
    """Return {wheel_key: {can_id, invert}} from the assignment config."""
    out = {}
    with lock:
        cfg = dict(state["config"])
    for id_str, mc in cfg.items():
        key = POS_KEY.get(mc.get("position"))
        if key:
            out[key] = {"can_id": int(id_str),
                        "invert": mc.get("forward_sign", 1) < 0}
    return out


def export_configs():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    wheels = wheels_yaml()
    with lock:
        ch = state["channel"] or ""
        k = dict(state["kinematics"])
        max_erpm = state["max_erpm"]
    missing = [p for p in POSITIONS if POS_KEY[p] not in wheels]

    order = ["front_left", "front_right", "rear_left", "rear_right"]
    lines = [
        "# motors.yaml - generated by rover_setup_web",
        "# Maps each wheel position to its VESC CAN id and forward direction.",
        "can:",
        "  interface: seeedstudio",
        f"  channel: {ch}    # override on the robot, e.g. /dev/ttyUSB0",
        f"  bitrate: {BITRATE}",
        "wheels:",
    ]
    for key in order:
        w = wheels.get(key)
        if w:
            lines.append(f"  {key}: {{ can_id: {w['can_id']}, "
                         f"invert: {str(w['invert']).lower()} }}")
    motors_path = os.path.join(OUTPUT_DIR, "motors.yaml")
    with open(motors_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    kin = [
        "# kinematics.yaml - generated by rover_setup_web",
        "# Loaded as ROS 2 params by rover_vesc_driver.",
        "/rover_vesc_driver:",
        "  ros__parameters:",
        f"    wheel_radius: {k['wheel_radius']}",
        f"    wheel_separation: {k['wheel_separation']}",
        f"    wheelbase: {k['wheelbase']}",
        f"    gear_ratio: {k['gear_ratio']}",
        f"    motor_pole_pairs: {int(k['motor_pole_pairs'])}",
        f"    max_wheel_speed: {k['max_wheel_speed']}",
        f"    battery_cells: {int(k['battery_cells'])}",
        f"    max_erpm: {int(max_erpm)}",
        "    cmd_vel_timeout: 0.5",
        "    use_stamped: true",
        '    motors_config: "motors.yaml"',
    ]
    kin_path = os.path.join(OUTPUT_DIR, "kinematics.yaml")
    with open(kin_path, "w", encoding="utf-8") as f:
        f.write("\n".join(kin) + "\n")

    return {"motors_yaml": motors_path, "kinematics_yaml": kin_path,
            "wheels": wheels, "missing_positions": missing}


# ---- HTTP API --------------------------------------------------------------
@app.route("/")
def index():
    return send_from_directory(STATIC, "index.html")


@app.route("/api/state")
def api_state():
    with lock:
        return jsonify({
            "mode": state["mode"], "identify_id": state["identify_id"],
            "known_ids": state["known_ids"], "config": state["config"],
            "max_erpm": state["max_erpm"], "kinematics": state["kinematics"],
            "bus_error": state["bus_error"], "channel": state["channel"],
            "tel": state["tel"], "positions": POSITIONS,
        })


@app.route("/api/history")
def api_history():
    return jsonify({"now": time.time(), "samples": list(history)})


@app.route("/api/scan", methods=["POST"])
def api_scan():
    with lock:
        state["scan_result"] = None
        state["mode"] = "scan"
    t0 = time.time()
    while time.time() - t0 < SCAN_S + 2.0:
        with lock:
            r = state["scan_result"]
        if r is not None:
            return jsonify({"ids": r})
        time.sleep(0.05)
    return jsonify({"ids": [], "error": "scan timeout"}), 504


@app.route("/api/identify", methods=["POST"])
def api_identify():
    cid = int(request.get_json(force=True)["id"])
    with lock:
        state["identify_id"] = cid
        state["mode"] = "identify"
        state["last_beat"] = time.time()
        if cid not in state["known_ids"]:
            state["known_ids"].append(cid)
    return jsonify({"ok": True})


@app.route("/api/stop", methods=["POST"])
def api_stop():
    with lock:
        state["mode"] = "idle"
        state["identify_id"] = None
        state["throttle"] = state["steer"] = 0.0
    return jsonify({"ok": True})


@app.route("/api/drive", methods=["POST"])
def api_drive():
    d = request.get_json(force=True)
    with lock:
        state["throttle"] = clamp(float(d.get("throttle", 0.0)), -1.0, 1.0)
        state["steer"] = clamp(float(d.get("steer", 0.0)), -1.0, 1.0)
        state["mode"] = "drive"
        state["last_beat"] = time.time()
    return jsonify({"ok": True})


@app.route("/api/heartbeat", methods=["POST"])
def api_heartbeat():
    with lock:
        state["last_beat"] = time.time()
    return jsonify({"ok": True})


@app.route("/api/assign", methods=["POST"])
def api_assign():
    d = request.get_json(force=True)
    cid, position = str(int(d["id"])), d["position"]
    fs = 1 if int(d.get("forward_sign", 1)) >= 0 else -1
    if position not in POSITIONS:
        return jsonify({"error": "bad position"}), 400
    with lock:
        for k, v in list(state["config"].items()):
            if v.get("position") == position and k != cid:
                del state["config"][k]
        state["config"][cid] = {"position": position, "forward_sign": fs}
    save_state()
    return jsonify({"ok": True, "config": state["config"]})


@app.route("/api/unassign", methods=["POST"])
def api_unassign():
    cid = str(int(request.get_json(force=True)["id"]))
    with lock:
        state["config"].pop(cid, None)
    save_state()
    return jsonify({"ok": True, "config": state["config"]})


@app.route("/api/set_speed", methods=["POST"])
def api_set_speed():
    v = int(clamp(int(request.get_json(force=True).get("max_erpm",
             DEFAULT_MAX_ERPM)), 100, ERPM_LIMIT))
    with lock:
        state["max_erpm"] = v
    save_state()
    return jsonify({"ok": True, "max_erpm": v})


@app.route("/api/kinematics", methods=["POST"])
def api_kinematics():
    d = request.get_json(force=True)
    with lock:
        for key in state["kinematics"]:
            if key in d:
                try:
                    state["kinematics"][key] = float(d[key])
                except (TypeError, ValueError):
                    pass
    save_state()
    with lock:
        return jsonify({"ok": True, "kinematics": state["kinematics"]})


@app.route("/api/export", methods=["POST"])
def api_export():
    try:
        return jsonify({"ok": True, **export_configs()})
    except Exception as e:
        return jsonify({"ok": False, "error": repr(e)}), 500


def main():
    load_state()
    threading.Thread(target=can_thread, daemon=True).start()
    print("rover_setup_web on http://127.0.0.1:5000")
    print("  output dir:", OUTPUT_DIR)
    app.run(host="127.0.0.1", port=5000, threaded=True, use_reloader=False)


if __name__ == "__main__":
    main()
