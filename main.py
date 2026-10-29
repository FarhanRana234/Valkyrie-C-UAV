"""C-UAV v2: Dual-Sensor Single-Target Tracker Web Interface

Transforms C-UAV into a web application powered by Flask and Flask-SocketIO.
Supports multi-device remote control, real-time telemetry, and live video streaming.
"""
import torch
import csv
import json
import math
import os
import struct
import sys
import threading
import time
from collections import deque

import cv2
import numpy as np
from flask import Flask, render_template, Response, jsonify
from flask_socketio import SocketIO, emit
from ultralytics import YOLO

# Load config
config_path = sys.argv[1] if len(sys.argv) > 1 else "config.json"
with open(config_path) as f:
    cfg = json.load(f)

SENS = cfg["sensors"]
KEYS = ("ir", "rgb")
DW, DH = cfg["display"]["width"], cfg["display"]["height"]

# --- SMART DEVICE AUTO-DETECTION ---
requested_device = cfg.get("device", "cpu").lower()

if requested_device.startswith("cuda"):
    if torch.cuda.is_available():
        DEVICE = requested_device
        gpu_name = torch.cuda.get_device_name(0)
        print(f"[INIT] CUDA GPU detected: {gpu_name}. Running inference on GPU.")
    else:
        print("[WARN] CUDA requested in config, but no NVIDIA GPU/CUDA runtime found. Falling back to CPU.")
        DEVICE = "cpu"
else:
    DEVICE = "cpu"
    print("[INIT] Running inference on CPU as configured.")

# Initialize Flask & SocketIO
app = Flask(__name__)
app.config['SECRET_KEY'] = 'valkyrie_cuav_secret'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

class SourceError(Exception):
    """A configured sensor could not be started."""

def sensor_enabled(key):
    sc = SENS[key]
    src = sc.get("source")
    return bool(sc.get("enabled", True)) and src is not None and str(src).strip() != ""

# Logging Buffer
LOG_BUFFER = []
MAX_LOG_LINES = 10

def log_message(msg, console=True):
    if console:
        print(msg)
    entry = f"{time.strftime('[%H:%M:%S]')} {msg}"
    LOG_BUFFER.append(entry)
    if len(LOG_BUFFER) > MAX_LOG_LINES:
        LOG_BUFFER.pop(0)
    socketio.emit('log_event', {'message': entry})

# ---------------------------------------------------------
# PERFORMANCE TIMING & CSV LOGGING HELPERS
# ---------------------------------------------------------
def init_timing_csv(csv_path):
    """Initializes the timing log CSV file with headers if it doesn't exist."""
    if not csv_path:
        return
    file_exists = os.path.exists(csv_path)
    if not file_exists:
        try:
            with open(csv_path, mode='w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(["timestamp", "sensor", "seq", "inference_ms", "measured_fps"])
        except Exception as e:
            print(f"[LOG] Failed to initialize timing CSV: {e}")

def log_timing_metric(csv_path, sensor_key, seq, inference_ms, current_fps):
    """Appends an inference timing record to the CSV log."""
    if not csv_path:
        return
    try:
        with open(csv_path, mode='a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
                sensor_key,
                seq,
                f"{inference_ms:.2f}",
                f"{current_fps:.2f}"
            ])
    except Exception:
        pass  # Prevent file write bottlenecks from affecting worker threads

class Stats:
    def __init__(self, window=100, skip=5):
        self.window, self.skip = window, skip
        self.d, self.n = {}, {}

    def add(self, key, ms):
        self.n[key] = self.n.get(key, 0) + 1
        if self.n[key] > self.skip:
            self.d.setdefault(key, deque(maxlen=self.window)).append(ms)

    def avg(self, key):
        v = self.d.get(key)
        return sum(v) / len(v) if v else 0.0

    def mx(self, key):
        v = self.d.get(key)
        return max(v) if v else 0.0

def largest_group_size(dets, eps):
    n = len(dets)
    if n < 2:
        return n
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i in range(n):
        for j in range(i + 1, n):
            if math.hypot(dets[i]["cx"] - dets[j]["cx"], dets[i]["cy"] - dets[j]["cy"]) <= eps:
                parent[find(i)] = find(j)
    sizes = {}
    for i in range(n):
        r = find(i)
        sizes[r] = sizes.get(r, 0) + 1
    return max(sizes.values())

class KalmanCV:
    def __init__(self, accel_var, meas_sigma):
        self.F = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], float)
        G = np.array([[0.5, 0], [0, 0.5], [1, 0], [0, 1]], float)
        self.Q = accel_var * (G @ G.T)
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], float)
        self.R = np.eye(2) * meas_sigma ** 2
        self.x = np.zeros(4)
        self.P = np.eye(4)

    def reset(self, cx, cy):
        self.x = np.array([cx, cy, 0.0, 0.0])
        self.P = np.diag([25.0, 25.0, 400.0, 400.0])

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, cx, cy):
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (np.array([cx, cy]) - self.H @ self.x)
        self.P = (np.eye(4) - K @ self.H) @ self.P

class StreamTracker:
    def __init__(self, key):
        self.key = key
        self.w, self.h = SENS[key]["width"], SENS[key]["height"]
        self.diag = math.hypot(self.w, self.h)
        self.eps = self.p("group_eps")
        self.manual = False
        self.dets, self.ranked, self.group = [], [], 0
        self.history, self.spawn = {}, {}
        self.kf = KalmanCV(self.p("kalman_accel_var"), self.p("kalman_meas_sigma"))
        self.locked, self.locked_id = False, None
        self.locked_conf, self.locked_age = 0.0, 0
        self.missing, self.challenge = 0, 0
        self.last_wh = (0.0, 0.0)
        self.status = "SEARCHING"
        self.box, self.center = None, None

    def p(self, name):
        sc = SENS[self.key]
        return sc[name] if name in sc else cfg[name]

    def filter_detections(self, boxes):
        dets, seen = [], set()
        if boxes is not None and len(boxes) > 0:
            xyxy = boxes.xyxy.cpu().numpy()
            confs = boxes.conf.cpu().numpy()
            ids = boxes.id.cpu().numpy() if boxes.id is not None else None
            for i in range(len(xyxy)):
                x1, y1, x2, y2 = (float(v) for v in xyxy[i][:4])
                w, h = x2 - x1, y2 - y1
                if w < self.p("min_box_width") or h < self.p("min_box_height"):
                    continue
                ar = w / (h + 1e-5)
                if ar < self.p("min_aspect_ratio") or ar > self.p("max_aspect_ratio"):
                    continue
                tid = int(ids[i]) if ids is not None else -(i + 1)
                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                seen.add(tid)
                hist = self.history.setdefault(tid, [])
                self.spawn.setdefault(tid, (cx, cy))
                hist.append((cx, cy))
                if len(hist) > 30:
                    hist.pop(0)
                if (self.p("static_gate_enabled") and tid != self.locked_id
                        and len(hist) >= self.p("static_gate_frames")):
                    sx, sy = self.spawn[tid]
                    if math.hypot(cx - sx, cy - sy) < self.p("static_gate_px"):
                        continue
                dets.append(dict(tid=tid, cx=cx, cy=cy, w=w, h=h, conf=float(confs[i]),
                                 age=len(hist), box=(x1, y1, x2, y2)))
        for k in [k for k in self.history if k not in seen]:
            self.history.pop(k, None)
            self.spawn.pop(k, None)
        return dets

    def score(self, d):
        w = cfg["priority_weights"]
        size = min(math.sqrt(d["w"] * d["h"]) / (cfg["priority_size_ref"] * self.diag), 1.0)
        centre = 1 - math.hypot(d["cx"] - self.w / 2, d["cy"] - self.h / 2) / (self.diag / 2)
        age = min(d["age"] / 30.0, 1.0)
        return w["size"] * size + w["center"] * centre + w["conf"] * d["conf"] + w["age"] * age

    def _locked_status(self):
        return "LOCKED (MANUAL)" if self.manual else "LOCKED"

    def _lock(self, d):
        self.kf.reset(d["cx"], d["cy"])
        self.locked, self.locked_id = True, d["tid"]
        self.locked_conf = d["conf"]
        self.locked_age = d["age"]
        self.last_wh = (d["w"], d["h"])
        self.missing, self.challenge = 0, 0
        self.status = self._locked_status()

    def manual_lock(self, det):
        self._lock(det)
        self.manual = True
        self.status = self._locked_status()
        self._publish()

    def _release(self):
        self.locked, self.locked_id = False, None
        self.locked_conf, self.locked_age = 0.0, 0
        self.manual = False
        self.missing, self.challenge = 0, 0
        self.status = "SEARCHING"

    def step(self, dets):
        if self.locked:
            self.kf.predict()
            px, py = self.kf.x[0], self.kf.x[1]
            match = next((d for d in dets if d["tid"] == self.locked_id), None)
            if match is None and dets:
                near = min(dets, key=lambda d: math.hypot(d["cx"] - px, d["cy"] - py))
                if math.hypot(near["cx"] - px, near["cy"] - py) <= self.p("gate_px"):
                    match = near
            if match is not None:
                self.kf.update(match["cx"], match["cy"])
                self.locked_id = match["tid"]
                self.locked_conf = match["conf"]
                self.locked_age = match["age"]
                self.last_wh = (match["w"], match["h"])
                self.missing, self.status = 0, self._locked_status()
                others = [d for d in dets if d is not match]
                best = max(others, key=self.score) if others else None
                if (not self.manual and best
                        and self.score(best) > self.score(match) * (1 + cfg["switch_margin"])):
                    self.challenge += 1
                else:
                    self.challenge = 0
                if not self.manual and best and self.challenge >= cfg["switch_hold_frames"]:
                    self._lock(best)
            else:
                self.missing += 1
                self.kf.x[2:] *= 0.9
                if self.missing > self.p("max_coast_frames"):
                    self._release()
                else:
                    self.status = f"COASTING ({self.missing})"
        elif dets:
            self._lock(max(dets, key=self.score))
        self.dets = dets
        self.ranked = sorted(((self.score(d), d) for d in dets), key=lambda t: t[0], reverse=True)
        self.group = largest_group_size(dets, self.eps)
        self._publish()

    def _publish(self):
        if not self.locked:
            self.box = self.center = None
            return
        lead = cfg["lead_frames"]
        cx = self.kf.x[0] + self.kf.x[2] * lead
        cy = self.kf.x[1] + self.kf.x[3] * lead
        w, h = self.last_wh
        self.box = (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)
        self.center = (cx, cy)

STATUS_PAYLOAD_LEN = {0x02: 1}

class Esp32Link:
    def __init__(self, c):
        self.ser, self.seq = None, 0
        self.buf = bytearray()
        self.status, self.status_t = None, 0.0
        if c.get("enabled"):
            try:
                import serial
                self.ser = serial.serial_for_url(c["port"], baudrate=c["baud"], timeout=0, do_not_open=True)
                self.ser.dtr = False
                self.ser.rts = False
                self.ser.open()
            except Exception as e:
                log_message(f"[ESP32] Could not open {c['port']}: {e}. Running without ESP32.")

    def send(self, pan_rad, tilt_rad, valid, fire=False, reload=False):
        if self.ser is None:
            return
        flags = (1 if valid else 0) | (2 if fire else 0) | (4 if reload else 0)
        body = struct.pack("<BHhhB", 0x01, self.seq & 0xFFFF,
                           int(pan_rad * 1000), int(tilt_rad * 1000), flags)
        crc = 0
        for b in body:
            crc ^= b
        try:
            self.ser.write(bytes([0xA5]) + body + bytes([crc]))
        except Exception as e:
            log_message(f"[ESP32] Write failed: {e}")
            self.ser = None
            return
        self.seq += 1

    def poll(self):
        if self.ser is None:
            return
        try:
            n = self.ser.in_waiting
            if n:
                self.buf += self.ser.read(n)
        except Exception as e:
            log_message(f"[ESP32] Read failed: {e}")
            self.ser = None
            return
        b = self.buf
        while b:
            if b[0] != 0x5A:
                del b[0]
                continue
            if len(b) < 2:
                return
            plen = STATUS_PAYLOAD_LEN.get(b[1])
            if plen is None:
                del b[0]
                continue
            if len(b) < 3 + plen:
                return
            x = b[1]
            for v in b[2:2 + plen]:
                x ^= v
            if x != b[2 + plen]:
                del b[0]
                continue
            fl = b[2]
            del b[:3 + plen]
            self.status = {"dart": bool(fl & 1), "sensor_ok": bool(fl & 2), "busy": bool(fl & 4)}
            self.status_t = time.time()

    def close(self):
        if self.ser:
            self.ser.close()

def px_to_angles(cx, cy, key, box_w=None):
    sc = SENS[key]
    w, h = sc["width"], sc["height"]
    off = sc.get("offset", {})
    half = math.radians(sc["hfov_deg"]) / 2
    span = h if sc.get("fov_across", "width") == "height" else w
    fisheye = sc.get("lens", "rectilinear") == "fisheye"
    f_pixels = (span / 2) / (half if fisheye else math.tan(half))

    dx, dy = cx - w / 2, cy - h / 2
    if "known_target_width_inches" in off and ("x_inches" in off or "y_inches" in off):
        if box_w and box_w > 0:
            dist = off["known_target_width_inches"] * f_pixels / box_w
        else:
            dist = off.get("default_distance_inches", 590.0)
        dx += off.get("x_inches", 0.0) * f_pixels / dist
        dy += off.get("y_inches", 0.0) * f_pixels / dist
    else:
        dx += off.get("x", 0)
        dy += off.get("y", 0)

    if fisheye:
        r = math.hypot(dx, dy)
        if r == 0:
            vx, vy, vz = 0.0, 0.0, 1.0
        else:
            theta = r / f_pixels
            vx, vy, vz = math.sin(theta) * dx / r, math.sin(theta) * dy / r, math.cos(theta)
    else:
        vx, vy, vz = dx, dy, f_pixels
    pan = math.atan2(vx, vz)
    tilt = math.atan2(vy, math.hypot(vx, vz))
    return pan, (-tilt if cfg["invert_tilt"] else tilt)

def aim_error_deg(tr):
    box_w = tr.last_wh[0] if tr.last_wh else None
    pan, tilt = px_to_angles(tr.center[0], tr.center[1], tr.key, box_w=box_w)
    return math.degrees(math.hypot(pan, tilt))

def auto_block_reason(state, src, now, last_fire):
    if src is None or not src.locked:
        return "no target", "no target locked"
    if cfg.get("require_arm", True) and not state["armed"]:
        return "disarmed", "DISARMED"
    am = ammo_state(state, now)
    if am == "reloading":
        return "reloading", f"reloading ({state['reload_until'] - now:.0f}s left)"
    if am == "unknown":
        return "unknown", "dart state unknown"
    if am == "empty":
        if state.get("auto_reload") and state.get("reload_due") is not None:
            return "reloading", "automatic reload queued"
        return "empty", "no dart loaded"
    if src.missing > 0:
        return "coasting", "target is coasting"
    thr = cfg.get("auto_shoot_confidence", 0.65)
    if src.locked_conf < thr:
        return "confidence", f"confidence {src.locked_conf:.2f} < {thr:.2f}"
    need = cfg.get("auto_shoot_min_age", 15)
    if src.locked_age < need:
        return "age", f"lock age {src.locked_age} < {need} frames"
    err, lim = aim_error_deg(src), cfg.get("auto_shoot_max_error_deg", 2.0)
    if err > lim:
        return "off crosshair", f"target {err:.1f} deg off crosshair"
    if now - last_fire < cfg.get("fire_cooldown", 2.0):
        return "cooldown", "cooldown"
    return None

def _ammo_from_timer(state, now):
    state["ammo_src"] = "timer"
    if state["loaded"]:
        return "loaded"
    if state["reloading"]:
        if now >= state["reload_until"]:
            state["loaded"], state["reloading"] = True, False
            log_message("[RELOAD] Reload timeout finished: dart assumed loaded.")
            return "loaded"
        return "reloading"
    return "empty"

def _ammo_from_sensor(state, st, now):
    state["ammo_src"] = "sensor"
    present, busy = st["dart"], st["busy"]
    if state["reloading"]:
        if busy or now - state["reload_sent_at"] < cfg.get("reload_grace_s", 0.5):
            return "reloading"
        if present:
            state["loaded"], state["reloading"] = True, False
            log_message("[RELOAD] Sensor confirms: dart loaded.")
            return "loaded"
        if now >= state["reload_until"]:
            state["loaded"], state["reloading"] = False, False
            log_message("[RELOAD] FAILED: timeout reached.")
            return "empty"
        return "reloading"
    if now - state.get("fired_at", -1e9) < cfg.get("sensor_settle_s", 0.6):
        return "empty"
    state["loaded"] = present and not busy
    return "loaded" if state["loaded"] else ("reloading" if busy else "empty")

def ammo_state(state, now):
    mode = cfg.get("ammo_source", "auto")
    st = state.get("esp_status")
    if mode != "timer" and st is not None and st["sensor_ok"]:
        return _ammo_from_sensor(state, st, now)
    if mode == "sensor":
        state["ammo_src"] = "sensor"
        return "unknown"
    return _ammo_from_timer(state, now)

def note_fired(state, now):
    state["loaded"], state["reloading"] = False, False
    state["fired_at"] = now
    state["shots"] = state.get("shots", 0) + 1
    if state.get("auto_reload"):
        state["reload_due"] = now + cfg.get("auto_reload_delay_s", 0.5)

def _start_reload(state, now):
    state["loaded"], state["reloading"] = False, True
    state["reload_sent_at"] = now
    state["reload_until"] = now + cfg.get("reload_timeout_s", 6.0)
    state["shots"] = 0
    state["reload_due"] = None

def decide_reload(state, now, last_reload, tag=""):
    if state.get("manual_reload_trigger"):
        state["manual_reload_trigger"] = False
        if now - last_reload < cfg.get("reload_min_interval_s", 1.0):
            log_message("[RELOAD] Ignored: reload command was just sent.")
            return False, last_reload
        _start_reload(state, now)
        log_message(f"[RELOAD] Manual reload command sent." + tag)
        return True, now
    due = state.get("reload_due")
    if due is not None and now >= due:
        state["reload_due"] = None
        if state.get("auto_reload"):
            _start_reload(state, now)
            log_message(f"[RELOAD] Auto reload command sent." + tag)
            return True, now
    return False, last_reload

def decide_fire(state, src, now, last_fire, tag=""):
    cooldown_ok = (now - last_fire) >= cfg.get("fire_cooldown", 2.0)
    armed_ok = state["armed"] or not cfg.get("require_arm", True)

    if state["manual_fire_trigger"]:
        state["manual_fire_trigger"] = False
        if not armed_ok:
            log_message("[SAFE] Fire blocked: DISARMED.")
            return False, last_fire
        am = ammo_state(state, now)
        if am == "reloading" or am == "unknown" or am == "empty":
            log_message(f"[SAFE] Fire blocked: state is {am.upper()}.")
            return False, last_fire
        if src and cooldown_ok:
            log_message("[ACTION] Manual Interception Fired!" + tag)
            note_fired(state, now)
            return True, now
        return False, last_fire

    if state["auto_shoot"]:
        why = auto_block_reason(state, src, now, last_fire)
        if why is None:
            log_message(f"[AUTO-SHOOT] Interception Fired!{tag}")
            state["auto_cat"], state["auto_msg"] = "fired", "fired"
            note_fired(state, now)
            return True, now
        cat, msg = why
        if cat != state.get("auto_cat"):
            state["auto_cat"] = cat
            if cat not in {"no target", "cooldown", "reloading"} and now - state.get("auto_log_t", 0.0) > 1.5:
                log_message(f"[AUTO] Holding fire: {msg}")
                state["auto_log_t"] = now
        state["auto_msg"] = msg
    else:
        state["auto_cat"], state["auto_msg"] = None, ""
    return False, last_fire

def to_proc(fr, w, h, crop=None):
    if crop:
        H0, W0 = fr.shape[:2]
        x0, y0 = max(0, int(round(crop["x"] * W0))), max(0, int(round(crop["y"] * H0)))
        x1 = min(W0, x0 + max(1, int(round(crop["w"] * W0))))
        y1 = min(H0, y0 + max(1, int(round(crop["h"] * H0))))
        fr = fr[y0:y1, x0:x1]
    if fr.shape[1] == w and fr.shape[0] == h:
        return fr
    shrinking = fr.shape[1] * fr.shape[0] > w * h
    return cv2.resize(fr, (w, h), interpolation=cv2.INTER_AREA if shrinking else cv2.INTER_LINEAR)

class LiveCapture(threading.Thread):
    def __init__(self, cap):
        super().__init__(name="camera-reader", daemon=True)
        self.cap = cap
        self.cond = threading.Condition()
        self.frame, self.t, self.seq = None, 0.0, 0
        self.alive, self.stopping = True, False

    def run(self):
        fails = 0
        while not self.stopping:
            ok, fr = self.cap.read()
            if not ok:
                fails += 1
                if fails > 30:
                    break
                time.sleep(0.01)
                continue
            fails = 0
            with self.cond:
                self.frame, self.t, self.seq = fr, time.time(), self.seq + 1
                self.cond.notify_all()

    def get(self, last_seq, timeout=2.0):
        end = time.time() + timeout
        with self.cond:
            while self.seq == last_seq and self.alive:
                left = end - time.time()
                if left <= 0:
                    return None
                self.cond.wait(left)
            if self.seq == last_seq:
                return None
            return self.frame, self.t, self.seq

BACKENDS = {"ANY": cv2.CAP_ANY, "V4L2": cv2.CAP_V4L2, "DSHOW": cv2.CAP_DSHOW,
            "MSMF": cv2.CAP_MSMF, "GSTREAMER": cv2.CAP_GSTREAMER, "FFMPEG": cv2.CAP_FFMPEG}

class OpenCVSource:
    """Video file, camera index, stream URL or GStreamer pipeline (anything cv2.VideoCapture opens)."""

    def __init__(self, key, sc):
        self.key, self.w, self.h = key, sc["width"], sc["height"]
        self.crop, self._crop_checked = sc.get("crop"), False
        self.src_path = sc["source"]
        self.is_file = os.path.isfile(str(self.src_path))
        pipeline = isinstance(self.src_path, str) and "!" in self.src_path
        is_url = isinstance(self.src_path, str) and "://" in self.src_path
        backend = BACKENDS[sc.get("backend", "GSTREAMER" if pipeline else "ANY").upper()]
        
        self.cap = cv2.VideoCapture(int(self.src_path) if str(self.src_path).isdigit() else self.src_path, backend)
        if not self.cap.isOpened():
            raise SourceError(f"could not open source: {self.src_path}")
        self.iv = 1.0 / (self.cap.get(cv2.CAP_PROP_FPS) or 30)
        self.realtime = cfg.get("realtime_files", True)
        self.idx, self.t0 = 0, time.time()
        self.live, self.last_seq = None, 0
        if not self.is_file:
            cam = {} if is_url else sc.get("camera", {})
            if cam.get("fourcc"):
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*cam["fourcc"]))
            if cam.get("width"):
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam["width"])
            if cam.get("height"):
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam["height"])
            if cam.get("fps"):
                self.cap.set(cv2.CAP_PROP_FPS, cam["fps"])
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if sc.get("camera", {}).get("latest_only", True):
                self.live = LiveCapture(self.cap)
                self.live.start()

    def proc(self, fr):
        return to_proc(fr, self.w, self.h, self.crop)

    def read(self):
        if self.live is not None:
            item = self.live.get(self.last_seq)
            if item is None:
                return None
            fr, t, self.last_seq = item
            return self.proc(fr), t
        if self.is_file:
            due = int((time.time() - self.t0) / self.iv)
            if self.realtime:
                while self.idx < due:
                    if not self.cap.grab():
                        return self._eof()
                    self.idx += 1
            wait = self.t0 + self.idx * self.iv - time.time()
            if wait > 0:
                time.sleep(wait)
        ok, fr = self.cap.read()
        if not ok:
            return self._eof()
        self.idx += 1
        return self.proc(fr), time.time()

    def _eof(self):
        if cfg.get("loop_videos", False) and self.is_file:
            try:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, fr = self.cap.read()
                if ok:
                    self.idx = 1
                    self.t0 = time.time()
                    return self.proc(fr), self.t0
            except Exception:
                pass
            # Fallback: completely re-open file capture if pointer reset fails
            try:
                self.cap.release()
                self.cap = cv2.VideoCapture(self.src_path)
                ok, fr = self.cap.read()
                if ok:
                    self.idx = 1
                    self.t0 = time.time()
                    return self.proc(fr), self.t0
            except Exception as e:
                log_message(f"[CAM] Failed to loop video {self.key}: {e}")
        return None

    def release(self):
        if self.live is not None:
            self.live.stopping = True
        self.cap.release()

SOURCE_TYPES = {"opencv": OpenCVSource}

def make_source(key):
    kind = SENS[key].get("type", "opencv")
    return SOURCE_TYPES[kind](key, SENS[key])

def load_model(key):
    sc = SENS[key]
    path = sc["model_path"]
    imgsz = sc.get("imgsz", 480)
    return YOLO(path, task="detect"), imgsz

def infer(model, imgsz, frame, conf):
    return model.track(frame, conf=conf, persist=True, tracker="bytetrack.yaml",
                       imgsz=imgsz, verbose=False, device=DEVICE)[0].boxes

# ---------------------------------------------------------
# OPTIMIZED SENSOR WORKER WITH TIMING & CSV LOGGING
# ---------------------------------------------------------
class SensorWorker(threading.Thread):
    def __init__(self, key, source, model, imgsz, state, trackers):
        super().__init__(name=f"{key}-worker", daemon=True)
        self.key, self.source, self.model, self.imgsz = key, source, model, imgsz
        self.conf = SENS[key]["confidence"]
        self.state, self.trackers = state, trackers
        self.lock = threading.Lock()
        self.frame, self.result = None, None
        self.seq = self.seen = 0
        self.read_count = 0
        self.stop_flag = threading.Event()
        self.eof, self.error = False, None
        self.last_pub = time.time()
        self.peers = {}
        
        # FPS and timing tracking
        self.fps_counter = 0
        self.last_fps_time = time.time()
        self.current_fps = 0.0

    def _should_infer(self):
        n = cfg.get("secondary_every_n", 1)
        aim = self.state["aim_sensor"]
        if n <= 1 or aim == self.key or aim not in self.peers:
            return True
        if cfg.get("secondary_full_rate_when_unlocked", True):
            aim_stale = time.time() - self.peers[aim].last_pub > cfg.get("stale_result_ms", 500) / 1000.0
            if not self.trackers[aim].locked or aim_stale:
                return True
        return self.read_count % n == 0

    def run(self):
        timing_csv_path = cfg.get("timing_csv", "timing_log.csv")
        init_timing_csv(timing_csv_path)

        try:
            while not self.stop_flag.is_set():
                item = self.source.read()
                if item is None:
                    if getattr(self.source, "is_file", False) and cfg.get("loop_videos", False):
                        time.sleep(0.01)
                        continue
                    break
                
                fr, t_cap = item
                self.read_count += 1
                
                with self.lock:
                    self.frame = fr

                # Calculate live measured FPS
                now = time.time()
                self.fps_counter += 1
                if now - self.last_fps_time >= 1.0:
                    self.current_fps = self.fps_counter / (now - self.last_fps_time)
                    self.fps_counter = 0
                    self.last_fps_time = now

                # Run model inference if loaded and enabled
                inf_ms = 0.0
                boxes = []
                if self.model is not None and cfg.get("device") != "none":
                    if self._should_infer():
                        t_inf_start = time.time()
                        boxes = infer(self.model, self.imgsz, fr, self.conf)
                        inf_ms = (time.time() - t_inf_start) * 1000.0
                
                with self.lock:
                    self.seq += 1
                    self.last_pub = time.time()
                    self.result = {
                        "boxes": boxes,
                        "t_cap": t_cap,
                        "ms": inf_ms,
                        "seq": self.seq,
                        "fps": self.current_fps
                    }

                # Log to timing_log.csv
                log_timing_metric(timing_csv_path, self.key, self.seq, inf_ms, self.current_fps)

        except Exception as e:
            self.error = e
            log_message(f"[WORKER ERROR] Sensor {self.key}: {e}")
        finally:
            self.eof = True

    def take(self):
        with self.lock:
            if self.result is not None and self.result["seq"] != self.seen:
                self.seen = self.result["seq"]
                return self.result
        return None

    def get_frame(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

def pick_source(state, trackers, fresh):
    active = state["aim_sensor"]
    other = "ir" if active == "rgb" else "rgb"
    for need_manual in (True, False):
        for key in (active, other):
            tr = trackers[key]
            if tr.locked and (tr.manual or not need_manual) and fresh(key):
                return key
    return None

def render_overlay(frame, tr):
    h, w = frame.shape[:2]
    cx0, cy0 = int(w / 2), int(h / 2)
    cv2.line(frame, (cx0 - 15, cy0), (cx0 + 15, cy0), (0, 255, 0), 1)
    cv2.line(frame, (cx0, cy0 - 15), (cx0, cy0 + 15), (0, 255, 0), 1)

    for rank, (_, d) in enumerate(tr.ranked, 1):
        if tr.locked and d["tid"] == tr.locked_id:
            continue
        x1, y1, x2, y2 = map(int, d["box"])
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 1)
        cv2.putText(frame, f"T{rank}", (x1, max(y1 - 5, 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

    if tr.box is not None:
        x1, y1, x2, y2 = map(int, tr.box)
        color = (0, 0, 255) if tr.status.startswith("LOCKED") else (0, 165, 255)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        c = (int(tr.center[0]), int(tr.center[1]))
        cv2.circle(frame, c, 5, (0, 255, 0), -1)
        cv2.putText(frame, f"LOCK: ({c[0]},{c[1]})", (x1, max(y1 - 10, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    return frame

# Shared Global App State
APP_STATE = {
    "aim_sensor": cfg.get("aim_sensor", "rgb").lower(),
    "auto_shoot": cfg.get("auto_shoot_enabled", False),
    "manual_fire_trigger": False,
    "armed": False,
    "auto_reload": cfg.get("auto_reload_enabled", False),
    "manual_reload_trigger": False,
    "reload_due": None,
    "loaded": cfg.get("start_loaded", False),
    "reloading": False,
    "reload_until": 0.0,
    "reload_sent_at": 0.0, 
    "fired_at": -1e9,
    "esp_status": None,
    "ammo_src": "timer",
    "shots": 0,
}

trackers = {k: StreamTracker(k) for k in KEYS}

# Generator for MJPEG Web Video Streaming with "SENSOR MISSING" Fallback
def generate_camera_stream(sensor_key):
    while True:
        fr = None
        if sensor_key in workers and workers[sensor_key].is_alive():
            fr = workers[sensor_key].get_frame()
            if fr is not None:
                fr = render_overlay(fr, trackers[sensor_key])

        if fr is None:
            fr = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.rectangle(fr, (20, 20), (620, 460), (30, 40, 50), 2)
            cv2.putText(fr, f"SENSOR {sensor_key.upper()} MISSING", (110, 220),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 230), 2, cv2.LINE_AA)
            cv2.putText(fr, "STATUS: NO SOURCE / DISABLED", (135, 260),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1, cv2.LINE_AA)

        quality = int(cfg.get("jpeg_quality", 85))
        ret, jpeg = cv2.imencode('.jpg', fr, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if ret:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
        
        time.sleep(0.033)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed/<sensor_key>')
def video_feed(sensor_key):
    return Response(generate_camera_stream(sensor_key),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# WebSocket Command Actions
@socketio.on('control_action')
def handle_control_action(data):
    action = data.get('action')
    if action == 'arm':
        APP_STATE["armed"] = not APP_STATE["armed"]
        log_message(f"[UI] System {'ARMED' if APP_STATE['armed'] else 'DISARMED'}")
    elif action == 'auto':
        APP_STATE["auto_shoot"] = not APP_STATE["auto_shoot"]
        log_message(f"[UI] Auto-Shoot: {'ON' if APP_STATE['auto_shoot'] else 'OFF'}")
    elif action == 'fire':
        APP_STATE["manual_fire_trigger"] = True
        log_message("[UI] Manual Fire Triggered")
    elif action == 'reload':
        APP_STATE["manual_reload_trigger"] = True
        log_message("[UI] Manual Reload Triggered")
    elif action == 'areload':
        APP_STATE["auto_reload"] = not APP_STATE["auto_reload"]
        log_message(f"[UI] Auto-Reload: {'ON' if APP_STATE['auto_reload'] else 'OFF'}")
    elif action == 'aim':
        APP_STATE["aim_sensor"] = "ir" if APP_STATE["aim_sensor"] == "rgb" else "rgb"
        log_message(f"[UI] Aim Sensor Switched to: {APP_STATE['aim_sensor'].upper()}")
    elif action == 'release':
        for tr in trackers.values():
            tr._release()
        log_message("[UI] Target locks released.")
    
    emit_telemetry()

@socketio.on('tap_target')
def handle_tap_target(data):
    sensor_key = data.get('sensor')
    x_norm = data.get('x')
    y_norm = data.get('y')
    if sensor_key in trackers:
        tr = trackers[sensor_key]
        target_det = None
        min_dist = float('inf')
        for d in tr.dets:
            x1, y1, x2, y2 = d["box"]
            px = x_norm * tr.w
            py = y_norm * tr.h
            if x1 <= px <= x2 and y1 <= py <= y2:
                target_det = d
                break
            dist = math.hypot(d["cx"] - px, d["cy"] - py)
            if dist < min_dist:
                min_dist = dist
                target_det = d
        
        if target_det:
            tr.manual_lock(target_det)
            APP_STATE["aim_sensor"] = sensor_key
            log_message(f"[UI] Tap Locked Target ID T{target_det['tid']} on {sensor_key.upper()}")
        else:
            log_message(f"[UI] Tap target missed on {sensor_key.upper()}")
    emit_telemetry()

def emit_telemetry():
    now = time.time()
    socketio.emit('telemetry_update', {
        'armed': APP_STATE["armed"],
        'auto_shoot': APP_STATE["auto_shoot"],
        'auto_reload': APP_STATE["auto_reload"],
        'aim_sensor': APP_STATE["aim_sensor"].upper(),
        'ammo': ammo_state(APP_STATE, now).upper(),
        'shots': APP_STATE["shots"]
    })

def tracking_loop():
    link = Esp32Link(cfg["esp32"])
    tag = " [sent to ESP32]" if link.ser else " [DRY RUN]"
    last_send = last_fire_time = last_reload_time = 0.0
    send_dt = 1.0 / cfg.get("send_hz", 30)

    for k in KEYS:
        if k in workers:
            workers[k].peers = workers

    while True:
        now = time.time()
        link.poll()
        fresh = link.status is not None and (now - link.status_t) <= cfg["esp32"].get("status_timeout_s", 1.0)
        APP_STATE["esp_status"] = link.status if fresh else None
        ammo_state(APP_STATE, now)

        got = False
        for key in KEYS:
            if key in workers:
                res = workers[key].take()
                if res:
                    trackers[key].step(trackers[key].filter_detections(res["boxes"]))
                    got = True

        if got or now - last_send >= send_dt:
            key = pick_source(APP_STATE, trackers, lambda k: k in workers)
            src = trackers[key] if key else None
            should_fire, last_fire_time = decide_fire(APP_STATE, src, now, last_fire_time, tag)
            do_reload, last_reload_time = decide_reload(APP_STATE, now, last_reload_time, tag)

            if src:
                box_w = src.last_wh[0] if src.last_wh else None
                pan, tilt = px_to_angles(src.center[0], src.center[1], key, box_w=box_w)
                link.send(pan, tilt, True, fire=should_fire, reload=do_reload)
            else:
                link.send(0.0, 0.0, False, fire=False, reload=do_reload)
            last_send = now

        emit_telemetry()
        time.sleep(0.01)

# Global Setup & Server Boot
models, sources, workers = {}, {}, {}

if __name__ == "__main__":
    log_message("Starting Valkyrie C-UAV Web Application Server...")
    for key in KEYS:
        if sensor_enabled(key):
            models[key] = load_model(key)
            sources[key] = make_source(key)
            workers[key] = SensorWorker(key, sources[key], models[key][0], models[key][1], APP_STATE, trackers)
            workers[key].start()

    # Background Tracking Loop
    t_track = threading.Thread(target=tracking_loop, daemon=True)
    t_track.start()

    # Launch Web Server on Port 5000
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)