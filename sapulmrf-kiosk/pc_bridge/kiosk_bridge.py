"""
SapulMRF Kiosk bridge  (runs on the PC with the USB camera)

Works like the SapulMRF stream: the KIOSK (ESP32-S3 7") is the server and this
script connects to it. The kiosk joins your Wi-Fi or a phone hotspot, or makes
its own hotspot "SapulMRF-Kiosk" (then it is always 192.168.4.1).
The PC just has to be on the same network as the kiosk.

  USB cam -> YOLO -> annotated JPEG frames + detection summary -> kiosk
  kiosk  -> intake records / PIN / recent list -> this script -> CSV (+ SapulMRF backend, optional)

TCP port 5000, packets both ways:  [1 byte type][4 byte little-endian length][payload]
  'J'  PC -> kiosk   JPEG frame (FRAME_W x FRAME_H), only while the kiosk is on the scan screen
  'A'  kiosk -> PC   ACK, frame shown: send the next one (same flow control as the stream)
  'M'  both ways     JSON:
        PC -> kiosk  {"type":"det","recyclable":2,"residual":1,"bio":0,"category":"Mixed household","stable":true,"fps":8}
                     {"type":"receipt","code":"D-2850"}  {"type":"auth","ok":true}
                     {"type":"recent","rows":[...]}      {"type":"stats","kg":128.4,"drops":17}
        kiosk -> PC  {"type":"mode","mode":"scan"|"idle"}  {"type":"intake",...}
                     {"type":"pin","pin":"1234"}  {"type":"recent"}  {"type":"stats"}  {"type":"lock"}
UDP port 5001: the kiosk broadcasts "SAPULMRF <ip> <port>" every 2 s so this script can find it.

Install:  pip install ultralytics opencv-python requests
Run:      python kiosk_bridge.py --model best.pt --cam 1             (finds the kiosk by itself)
          python kiosk_bridge.py --model best.pt --cam 1 192.168.1.77  (or give the IP on the kiosk screen)
"""

import argparse
import csv
import json
import os
import socket
import struct
import threading
import time
from collections import Counter, deque
from datetime import datetime

import cv2

try:
    import requests
except ImportError:
    requests = None

try:
    from ultralytics import YOLO
except ImportError:          # lets the network part be tested without YOLO installed
    YOLO = None

STREAM_PORT = 5000
BEACON_PORT = 5001
HOTSPOT_IP = "192.168.4.1"   # kiosk's own hotspot

# ---------------------------------------------------------------- settings
FRAME_W, FRAME_H = 480, 360      # matches the video box on the kiosk
JPEG_QUALITY = 55                # lower = faster on the ESP32
SEND_FPS = 10                    # cap for frames sent to the kiosk
CONF = 0.45
STABLE_FRAMES = 12               # same category this many frames in a row = "stable"
OPERATOR_PINS = {"1234": "Operator"}          # change these!
LOCAL_LOG = "kiosk_intake.csv"                 # always written, even with a backend

# Optional: your SapulMRF backend. Leave empty to run offline (CSV only).
# The bridge POSTs JSON to BACKEND_URL + "/api/intake" and expects {"code": "D-2850"} back.
BACKEND_URL = os.environ.get("SAPULMRF_API", "")
BACKEND_TOKEN = os.environ.get("SAPULMRF_TOKEN", "")

# Map YOUR model's class names to the three kiosk buckets (lower-case).
RECYCLABLE = {"bottle", "pet bottle", "plastic bottle", "can", "tin can", "aluminum can",
              "glass bottle", "carton", "cardboard", "paper"}
BIODEGRADABLE = {"food", "food waste", "leaves", "peel", "organic"}
# anything else = residual (sachets, wrappers, styro, etc.)

COLORS = {"recyclable": (122, 195, 76), "residual": (58, 163, 242), "bio": (46, 90, 122)}  # BGR


def bucket(name: str) -> str:
    n = name.lower()
    if n in RECYCLABLE:
        return "recyclable"
    if n in BIODEGRADABLE:
        return "bio"
    return "residual"


def category_from(counts: Counter) -> str:
    kinds = [k for k in ("recyclable", "residual", "bio") if counts[k] > 0]
    if not kinds:
        return "Nothing detected"
    if kinds == ["recyclable"]:
        return "Recyclable"
    if kinds == ["bio"]:
        return "Biodegradable"
    if kinds == ["residual"]:
        return "Residual"
    return "Mixed household"


# ---------------------------------------------------------------- state
class Bridge:
    def __init__(self, model_path, cam_index):
        self.model = YOLO(model_path) if YOLO and model_path else None
        backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY
        self.cap = cv2.VideoCapture(cam_index, backend) if self.model else None
        if self.cap is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.sock = None
        self.send_lock = threading.Lock()
        self.ack = threading.Event()
        self.alive = False
        self.mode = "idle"
        self.auth = False
        self.history = deque(maxlen=STABLE_FRAMES)
        self.last = {"recyclable": 0, "residual": 0, "bio": 0, "category": "Nothing detected"}
        self.fps = 0.0
        self.next_code = self._load_next_code()

    # ----- one camera frame -> YOLO -> annotated JPEG + summary
    def detect_once(self):
        ok, frame = self.cap.read()
        if not ok:
            return None, None
        res = self.model(frame, conf=CONF, verbose=False)[0]
        counts = Counter()
        for box in res.boxes:
            name = self.model.names[int(box.cls)]
            b = bucket(name)
            counts[b] += 1
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            color = COLORS[b]
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
            label = f"{name} {float(box.conf):.2f}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(frame, (x1, y1 - th - 8), (x1 + tw + 8, y1), color, -1)
            cv2.putText(frame, label, (x1 + 4, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (20, 20, 20), 2)
        cat = category_from(counts)
        self.history.append(cat)
        stable = len(self.history) == STABLE_FRAMES and len(set(self.history)) == 1 and cat != "Nothing detected"
        det = {"type": "det", "recyclable": counts["recyclable"], "residual": counts["residual"],
               "bio": counts["bio"], "category": cat, "stable": stable, "fps": round(self.fps)}
        self.last = det
        small = cv2.resize(frame, (FRAME_W, FRAME_H), interpolation=cv2.INTER_AREA)
        ok, jpg = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        return (jpg.tobytes() if ok else None), det

    # ----- intake records
    def _load_next_code(self):
        last = 2849
        if os.path.exists(LOCAL_LOG):
            with open(LOCAL_LOG, newline="") as f:
                for row in csv.DictReader(f):
                    try:
                        last = max(last, int(row["code"].split("-")[1]))
                    except (KeyError, ValueError, IndexError):
                        pass
        return last + 1

    def save_intake(self, msg):
        record = {
            "received": datetime.now().isoformat(timespec="seconds"),
            "source_area": f"Purok {int(msg.get('purok', 0))}",
            "category": msg.get("category", "Mixed household"),
            "weight_kg": round(float(msg.get("weight", 0)), 2),
            "manual": bool(msg.get("manual", False)),
            "recyclables_seen": int(msg.get("recyclables", 0)),
            "recorded_by": "Kiosk",
        }
        code = None
        if BACKEND_URL and requests:
            try:
                headers = {"Authorization": f"Bearer {BACKEND_TOKEN}"} if BACKEND_TOKEN else {}
                r = requests.post(BACKEND_URL.rstrip("/") + "/api/intake", json=record,
                                  headers=headers, timeout=4)
                r.raise_for_status()
                code = r.json().get("code")
            except Exception as e:  # keep the kiosk working even if the backend is down
                print("backend error, saving locally:", e)
        if not code:
            code = f"D-{self.next_code}"
        self.next_code = max(self.next_code, int(code.split("-")[1]) + 1)
        new = not os.path.exists(LOCAL_LOG)
        with open(LOCAL_LOG, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["code"] + list(record))
            if new:
                w.writeheader()
            w.writerow({"code": code, **record})
        return code

    def recent(self, n=6):
        if not os.path.exists(LOCAL_LOG):
            return []
        with open(LOCAL_LOG, newline="") as f:
            rows = list(csv.DictReader(f))[-n:]
        out = []
        for r in reversed(rows):
            t = datetime.fromisoformat(r["received"])
            out.append({"code": r["code"], "time": t.strftime("%I:%M %p").lstrip("0"),
                        "purok": r["source_area"], "cat": r["category"], "kg": f"{float(r['weight_kg']):.1f} kg"})
        return out

    def stats_today(self):
        kg, drops = 0.0, 0
        if os.path.exists(LOCAL_LOG):
            today = datetime.now().date()
            with open(LOCAL_LOG, newline="") as f:
                for r in csv.DictReader(f):
                    if datetime.fromisoformat(r["received"]).date() == today:
                        kg += float(r["weight_kg"])
                        drops += 1
        return {"type": "stats", "kg": round(kg, 1), "drops": drops}


    # ----- link to the kiosk
    def connect_and_serve(self, host, port):
        sock = socket.create_connection((host, port), timeout=5)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(None)
        self.sock = sock
        self.mode = "idle"
        self.auth = False
        self.alive = True
        self.ack.set()
        print(f"Connected to kiosk {host}:{port}")
        reader = threading.Thread(target=self.read_loop, daemon=True)
        reader.start()
        self.send_json(self.stats_today())
        try:
            self.stream_loop()
        finally:
            self.alive = False
            try:
                sock.close()
            except OSError:
                pass
            print("Kiosk disconnected")

    def send_packet(self, kind: bytes, payload: bytes):
        with self.send_lock:
            self.sock.sendall(kind + struct.pack("<I", len(payload)) + payload)

    def send_json(self, obj):
        self.send_packet(b"M", json.dumps(obj, separators=(",", ":")).encode())

    def recv_exact(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("kiosk closed the connection")
            buf += chunk
        return buf

    def read_loop(self):
        try:
            while self.alive:
                kind, length = struct.unpack("<cI", self.recv_exact(5))
                payload = self.recv_exact(length) if length else b""
                if kind == b"A":
                    self.ack.set()
                elif kind == b"M":
                    self.on_message(json.loads(payload))
        except (OSError, ConnectionError, ValueError) as e:
            if self.alive:
                print("link error:", e)
        self.alive = False
        self.ack.set()

    def on_message(self, msg):
        t = msg.get("type")
        if t == "mode":
            self.mode = "scan" if msg.get("mode") == "scan" else "idle"
            self.ack.set()
            print("kiosk mode:", self.mode)
        elif t == "intake":
            code = self.save_intake(msg)
            print("intake saved:", code, msg)
            self.send_json({"type": "receipt", "ok": True, "code": code})
            self.send_json(self.stats_today())
        elif t == "pin":
            name = OPERATOR_PINS.get(str(msg.get("pin", "")))
            self.auth = bool(name)
            self.send_json({"type": "auth", "ok": bool(name), "name": name or ""})
        elif t == "recent" and self.auth:
            self.send_json({"type": "recent", "rows": self.recent()})
        elif t == "stats":
            self.send_json(self.stats_today())
        elif t == "lock":
            self.auth = False

    def stream_loop(self):
        period = 1.0 / SEND_FPS
        while self.alive:
            if self.mode != "scan":
                self.history.clear()
                time.sleep(0.1)
                continue
            t0 = time.time()
            jpg, det = self.detect_once()
            if jpg is None:
                time.sleep(0.05)
                continue
            # wait for the kiosk to show the previous frame (newest-frame-only, no lag build-up)
            if not self.ack.wait(timeout=2.0):
                print("no ACK from kiosk in 2 s, sending anyway")
            if not self.alive or self.mode != "scan":
                continue
            self.ack.clear()
            self.send_packet(b"J", jpg)
            self.send_json(det)
            dt = time.time() - t0
            self.fps = 0.8 * self.fps + 0.2 * (1.0 / max(dt, 1e-3))
            time.sleep(max(0.0, period - dt))


def discover(timeout=6.0):
    """Listen for the kiosk's UDP beacon. Returns (ip, port) or None."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind(("", BEACON_PORT))
    except OSError as e:
        print("can't listen for the kiosk beacon:", e)
        return None
    s.settimeout(timeout)
    try:
        while True:
            data, addr = s.recvfrom(128)
            parts = data.decode(errors="ignore").split()
            if len(parts) == 3 and parts[0] == "SAPULMRF":
                return parts[1], int(parts[2])
    except socket.timeout:
        return None
    finally:
        s.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("host", nargs="?", help="kiosk IP shown on its welcome screen (optional)")
    ap.add_argument("--model", default="best.pt", help="your trained YOLO weights")
    ap.add_argument("--cam", type=int, default=0, help="USB camera index (external cam is usually 1)")
    ap.add_argument("--port", type=int, default=STREAM_PORT)
    args = ap.parse_args()

    if YOLO is None:
        raise SystemExit("Ultralytics isn't installed: pip install ultralytics")
    bridge = Bridge(args.model, args.cam)
    while True:
        host, port = args.host, args.port
        if not host:
            print("Looking for the kiosk on this network...")
            found = discover()
            if found:
                host, port = found
                print(f"Found kiosk at {host}:{port}")
            else:
                host = HOTSPOT_IP
                print(f"No beacon heard; trying the kiosk hotspot address {host}")
        try:
            bridge.connect_and_serve(host, port)
        except (OSError, ConnectionError) as e:
            print(f"Can't reach kiosk at {host}:{port} ({e}). Retrying in 3 s... (Ctrl+C to stop)")
        time.sleep(3)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped")
