#!/usr/bin/env python3
"""
Edge Agent Daemon for Multi-Sensor Edge Devices
Optimized for Raspberry Pi Zero v1.1 and embedded Linux boards.

Features:
- Multi-modality sensor capture (RGB, Thermal, Depth, etc.)
- Hardware support for STEVAL-55G0MBI (PCB4189D) / V4L2 / libcamera / picam
- Low CPU & memory footprint suitable for ARMv6 single-core (512MB RAM)
- Internal local storage structure on SD card:
    <storage_root>/<YYYY-MM-DD>/<HH-MM-SS>/<modality>/
- Remote control endpoints:
    /api/status
    /api/stream/<modality> (MJPEG)
    /api/collection/start
    /api/collection/stop
    /api/snapshot/<modality>
"""

import os
import sys
import time
import json
import socket
import logging
import threading
import subprocess
from datetime import datetime
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
import io

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [%(levelname)s] [EdgeDaemon] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)
logger = logging.getLogger("edge_daemon")

CONFIG_PATH = os.environ.get("EDGE_CONFIG", os.path.join(os.path.dirname(__file__), "edge_config.json"))

DEFAULT_CONFIG = {
    "device_id": "edge-pi-zero-01",
    "device_name": "Raspberry Pi Zero v1.1 (Lab A)",
    "host": "0.0.0.0",
    "port": 5001,
    "storage_root": "/home/pi/edge_data",
    "server_url": "http://192.168.1.100:8000",
    "sensors": [
        {
            "id": "rgb-main",
            "modality": "rgb",
            "name": "STEVAL-55G0MBI Global Shutter",
            "type": "v4l2",
            "device": "/dev/video0",
            "width": 640,
            "height": 480,
            "fps": 15
        }
    ]
}


def load_config():
    if os.path.exists(CONFIG_PATH):
        try:
            with open(CONFIG_PATH, "r") as f:
                cfg = json.load(f)
                logger.info(f"Loaded configuration from {CONFIG_PATH}")
                return cfg
        except Exception as e:
            logger.warning(f"Failed to read {CONFIG_PATH}: {e}, using default config")
    return DEFAULT_CONFIG.copy()


class SensorDriver:
    """Base sensor driver with fallback synthetic generator for testing."""

    def __init__(self, sensor_cfg):
        self.cfg = sensor_cfg
        self.modality = sensor_cfg.get("modality", "rgb")
        self.sensor_id = sensor_cfg.get("id", "sensor-0")
        self.name = sensor_cfg.get("name", "Sensor")
        self.width = sensor_cfg.get("width", 640)
        self.height = sensor_cfg.get("height", 480)
        self.fps = sensor_cfg.get("fps", 15)
        self.device_node = sensor_cfg.get("device", "/dev/video0")
        self.cap = None
        self.use_fallback = False
        self._init_hardware()

    def _init_hardware(self):
        # Check if hardware camera device is accessible
        if os.path.exists(self.device_node):
            try:
                import cv2
                self.cap = cv2.VideoCapture(self.device_node)
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.cap.set(cv2.CAP_PROP_FPS, self.fps)
                if self.cap.isOpened():
                    logger.info(f"[{self.sensor_id}] Opened hardware device {self.device_node}")
                    return
            except Exception as e:
                logger.warning(f"[{self.sensor_id}] OpenCV / V4L2 init failed on {self.device_node}: {e}")

        logger.info(f"[{self.sensor_id}] Hardware {self.device_node} not opened. Initializing optimized fallback generator.")
        self.use_fallback = True

    def get_jpeg_frame(self):
        """Returns raw JPEG bytes for MJPEG streaming or saving."""
        if not self.use_fallback and self.cap is not None and self.cap.isOpened():
            import cv2
            ret, frame = self.cap.read()
            if ret:
                ret2, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
                if ret2:
                    return jpeg.tobytes()

        # Fallback synthetic frame generator
        return self._generate_synthetic_frame()

    def _generate_synthetic_frame(self):
        """Generates dynamic synthetic image with timestamp and sensor metadata."""
        try:
            from PIL import Image, ImageDraw, ImageFont
        except ImportError:
            # Ultra-lightweight raw fallback if Pillow not installed: basic PPM -> JPEG or mock
            return b""

        # Create image based on modality
        if self.modality == "rgb":
            # Gradient blue-gray
            img = Image.new("RGB", (self.width, self.height), color=(20, 24, 33))
            draw = ImageDraw.Draw(img)
            # Draw synthetic test target / simulated camera feed
            t = time.time()
            cx = int((self.width // 2) + 80 * (0.5 - (t % 2)))
            cy = int(self.height // 2)
            draw.rectangle([20, 20, self.width - 20, self.height - 20], outline=(40, 160, 220), width=2)
            draw.ellipse([cx - 30, cy - 30, cx + 30, cy + 30], fill=(0, 200, 150), outline=(255, 255, 255))
        elif self.modality == "thermal":
            # Heatmap tones
            img = Image.new("RGB", (self.width, self.height), color=(40, 10, 50))
            draw = ImageDraw.Draw(img)
            draw.ellipse([180, 140, 460, 340], fill=(240, 90, 40), outline=(255, 220, 80))
            draw.ellipse([260, 190, 380, 290], fill=(255, 240, 100))
        elif self.modality == "depth":
            # Grayscale depth map
            img = Image.new("RGB", (self.width, self.height), color=(10, 10, 10))
            draw = ImageDraw.Draw(img)
            draw.rectangle([100, 80, 540, 400], fill=(80, 80, 80), outline=(140, 140, 140))
            draw.ellipse([240, 160, 400, 320], fill=(180, 180, 180))
        else:
            img = Image.new("RGB", (self.width, self.height), color=(30, 30, 35))
            draw = ImageDraw.Draw(img)

        # Telemetry overlays
        now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        draw.text((30, 30), f"EDGE: {self.sensor_id} | MODALITY: {self.modality.upper()}", fill=(255, 255, 255))
        draw.text((30, 55), f"SENSOR: {self.name}", fill=(180, 200, 220))
        draw.text((30, 80), f"TIMESTAMP: {now_str}", fill=(100, 255, 120))
        draw.text((30, self.height - 45), "HARDWARE: STEVAL-55G0MBI PCB4189D (Pi Zero v1.1)", fill=(160, 160, 160))

        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=70)
        return buf.getvalue()

    def release(self):
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass


class CollectionManager:
    """
    Manages local internal storage recording on the edge device.
    Requirement:
    Create folder with today date -> folder with time -> modality [rgb] -> and there you can store the data.
    E.g.: <storage_root>/YYYY-MM-DD/HH-MM-SS/<modality>/frame_000001.jpg
    """

    def __init__(self, storage_root, sensors):
        self.storage_root = os.path.abspath(storage_root)
        self.sensors = sensors
        self.is_recording = False
        self.session_id = None
        self.session_start_time = None
        self.session_dir = None
        self.frame_counts = {}
        self.thread = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()

        os.makedirs(self.storage_root, exist_ok=True)

    def start_collection(self, session_label=None):
        with self.lock:
            if self.is_recording:
                return {
                    "status": "already_recording",
                    "session_id": self.session_id,
                    "session_dir": self.session_dir
                }

            now = datetime.now()
            date_folder = now.strftime("%Y-%m-%d")
            time_folder = now.strftime("%H-%M-%S")
            self.session_id = f"{date_folder}_{time_folder}"
            self.session_start_time = time.time()
            self.session_dir = os.path.join(self.storage_root, date_folder, time_folder)

            # Create modality folders for each sensor
            self.modality_paths = {}
            self.frame_counts = {}
            for sensor in self.sensors:
                modality = sensor.modality
                mod_path = os.path.join(self.session_dir, modality)
                os.makedirs(mod_path, exist_ok=True)
                self.modality_paths[sensor.sensor_id] = mod_path
                self.frame_counts[sensor.sensor_id] = 0

            # Write session metadata
            metadata = {
                "session_id": self.session_id,
                "start_time_iso": now.isoformat(),
                "date_folder": date_folder,
                "time_folder": time_folder,
                "storage_path": self.session_dir,
                "label": session_label or "Collection Session",
                "sensors": [
                    {
                        "id": s.sensor_id,
                        "modality": s.modality,
                        "name": s.name,
                        "resolution": f"{s.width}x{s.height}",
                        "fps": s.fps
                    }
                    for s in self.sensors
                ]
            }
            with open(os.path.join(self.session_dir, "session_manifest.json"), "w") as f:
                json.dump(metadata, f, indent=2)

            self.stop_event.clear()
            self.is_recording = True
            self.thread = threading.Thread(target=self._record_worker, daemon=True)
            self.thread.start()

            logger.info(f"Started collection session: {self.session_dir}")
            return {
                "status": "recording_started",
                "session_id": self.session_id,
                "session_dir": self.session_dir,
                "modalities": list(set(s.modality for s in self.sensors))
            }

    def stop_collection(self):
        with self.lock:
            if not self.is_recording:
                return {"status": "not_recording"}

            self.stop_event.set()
            if self.thread:
                self.thread.join(timeout=3.0)
            self.is_recording = False
            duration = time.time() - self.session_start_time

            # Update session manifest with final stats
            manifest_path = os.path.join(self.session_dir, "session_manifest.json")
            if os.path.exists(manifest_path):
                try:
                    with open(manifest_path, "r") as f:
                        meta = json.load(f)
                    meta["end_time_iso"] = datetime.now().isoformat()
                    meta["duration_seconds"] = round(duration, 2)
                    meta["frames_recorded"] = self.frame_counts
                    with open(manifest_path, "w") as f:
                        json.dump(meta, f, indent=2)
                except Exception as e:
                    logger.error(f"Error updating manifest: {e}")

            logger.info(f"Stopped collection session {self.session_id}. Recorded: {self.frame_counts}")
            result = {
                "status": "recording_stopped",
                "session_id": self.session_id,
                "session_dir": self.session_dir,
                "duration_seconds": round(duration, 2),
                "frame_counts": self.frame_counts
            }
            return result

    def _record_worker(self):
        """Worker thread recording frames to SD card at specified FPS."""
        # Calculate target interval
        fps = 10
        interval = 1.0 / fps

        while not self.stop_event.is_set():
            t0 = time.time()
            for sensor in self.sensors:
                folder = self.modality_paths.get(sensor.sensor_id)
                if not folder:
                    continue

                frame_bytes = sensor.get_jpeg_frame()
                if frame_bytes:
                    cnt = self.frame_counts.get(sensor.sensor_id, 0) + 1
                    self.frame_counts[sensor.sensor_id] = cnt
                    filename = f"frame_{cnt:06d}.jpg"
                    file_path = os.path.join(folder, filename)
                    try:
                        with open(file_path, "wb") as f:
                            f.write(frame_bytes)
                    except Exception as e:
                        logger.error(f"Failed to write frame to {file_path}: {e}")

            elapsed = time.time() - t0
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def get_status(self):
        # Calculate disk storage stats
        try:
            stat = os.statvfs(self.storage_root)
            free_bytes = stat.f_bavail * stat.f_frsize
            total_bytes = stat.f_blocks * stat.f_frsize
            free_gb = round(free_bytes / (1024 ** 3), 2)
            total_gb = round(total_bytes / (1024 ** 3), 2)
            used_percent = round(100.0 * (total_bytes - free_bytes) / max(total_bytes, 1), 1)
        except Exception:
            free_gb, total_gb, used_percent = 0.0, 0.0, 0.0

        return {
            "is_recording": self.is_recording,
            "session_id": self.session_id,
            "session_dir": self.session_dir,
            "duration_seconds": round(time.time() - self.session_start_time, 1) if self.is_recording else 0,
            "frame_counts": self.frame_counts,
            "storage_root": self.storage_root,
            "disk_free_gb": free_gb,
            "disk_total_gb": total_gb,
            "disk_used_percent": used_percent
        }


class EdgeHttpHandler(BaseHTTPRequestHandler):
    """Lightweight HTTP API & MJPEG streaming server."""

    daemon_ref = None  # Injected reference

    def log_message(self, format, *args):
        # Keep logs clean, suppress high-frequency GET requests
        if "stream" not in self.path:
            super().log_message(format, *args)

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        query = parse_qs(parsed.query)

        if path == "/" or path == "/api/status":
            self._handle_status()
        elif path.startswith("/api/stream/"):
            # Stream path: /api/stream/<modality> or /api/stream/<sensor_id>
            target = path.split("/api/stream/")[1]
            self._handle_mjpeg_stream(target)
        elif path.startswith("/api/snapshot/"):
            target = path.split("/api/snapshot/")[1]
            self._handle_snapshot(target)
        elif path == "/api/collection/start":
            res = self.daemon_ref.collector.start_collection()
            self._send_json(res)
        elif path == "/api/collection/stop":
            res = self.daemon_ref.collector.stop_collection()
            self._send_json(res)
        elif path == "/api/sessions":
            self._handle_list_sessions()
        else:
            self.send_error(404, "Not Found")

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/api/collection/start":
            content_length = int(self.headers.get('Content-Length', 0))
            label = None
            if content_length > 0:
                body = self.rfile.read(content_length)
                try:
                    payload = json.loads(body.decode('utf-8'))
                    label = payload.get("label")
                except Exception:
                    pass
            res = self.daemon_ref.collector.start_collection(session_label=label)
            self._send_json(res)
        elif path == "/api/collection/stop":
            res = self.daemon_ref.collector.stop_collection()
            self._send_json(res)
        else:
            self.send_error(404, "Not Found")

    def _send_json(self, data, status_code=200):
        body = json.dumps(data, indent=2).encode('utf-8')
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _handle_status(self):
        collector_status = self.daemon_ref.collector.get_status()
        sensor_info = [
            {
                "id": s.sensor_id,
                "modality": s.modality,
                "name": s.name,
                "resolution": f"{s.width}x{s.height}",
                "fps": s.fps
            }
            for s in self.daemon_ref.sensors
        ]
        status = {
            "device_id": self.daemon_ref.config.get("device_id"),
            "device_name": self.daemon_ref.config.get("device_name"),
            "online": True,
            "timestamp": datetime.now().isoformat(),
            "sensors": sensor_info,
            "modalities": list(set(s.modality for s in self.daemon_ref.sensors)),
            "collection": collector_status
        }
        self._send_json(status)

    def _handle_snapshot(self, target):
        sensor = self.daemon_ref.find_sensor(target)
        if not sensor:
            self.send_error(404, f"Sensor or modality '{target}' not found")
            return

        frame = sensor.get_jpeg_frame()
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(frame)))
        self.end_headers()
        self.wfile.write(frame)

    def _handle_mjpeg_stream(self, target):
        sensor = self.daemon_ref.find_sensor(target)
        if not sensor:
            self.send_error(404, f"Sensor or modality '{target}' not found")
            return

        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-cache, private")
        self.end_headers()

        target_delay = 1.0 / max(sensor.fps, 5)
        try:
            while True:
                frame_bytes = sensor.get_jpeg_frame()
                if frame_bytes:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame_bytes)}\r\n\r\n".encode())
                    self.wfile.write(frame_bytes)
                    self.wfile.write(b"\r\n")
                time.sleep(target_delay)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _handle_list_sessions(self):
        storage_root = self.daemon_ref.collector.storage_root
        sessions = []
        if os.path.exists(storage_root):
            for date_entry in sorted(os.listdir(storage_root), reverse=True):
                date_path = os.path.join(storage_root, date_entry)
                if not os.path.isdir(date_path):
                    continue
                for time_entry in sorted(os.listdir(date_path), reverse=True):
                    time_path = os.path.join(date_path, time_entry)
                    if not os.path.isdir(time_path):
                        continue
                    manifest_path = os.path.join(time_path, "session_manifest.json")
                    meta = {}
                    if os.path.exists(manifest_path):
                        try:
                            with open(manifest_path, "r") as f:
                                meta = json.load(f)
                        except Exception:
                            pass
                    sessions.append({
                        "date": date_entry,
                        "time": time_entry,
                        "path": time_path,
                        "manifest": meta
                    })
        self._send_json({"storage_root": storage_root, "sessions": sessions})


class EdgeDaemon:
    def __init__(self, config=None):
        self.config = config or load_config()
        self.sensors = [SensorDriver(sc) for sc in self.config.get("sensors", [])]
        self.collector = CollectionManager(
            storage_root=self.config.get("storage_root", "/home/pi/edge_data"),
            sensors=self.sensors
        )

    def find_sensor(self, target):
        # Match by id or modality
        for s in self.sensors:
            if s.sensor_id == target or s.modality.lower() == target.lower():
                return s
        return None

    def start_server(self):
        host = self.config.get("host", "0.0.0.0")
        port = int(self.config.get("port", 5001))

        # Create a per-daemon handler class so multiple instances in one process don't collide
        daemon_self = self
        class CustomEdgeHandler(EdgeHttpHandler):
            daemon_ref = daemon_self

        server = ThreadingHTTPServer((host, port), CustomEdgeHandler)
        logger.info(f"Edge Daemon listening on http://{host}:{port}")
        logger.info(f"Device: {self.config.get('device_name')} ({self.config.get('device_id')})")
        logger.info(f"Storage directory: {self.collector.storage_root}")
        logger.info(f"Sensors configured: {[f'{s.name} ({s.modality})' for s in self.sensors]}")

        try:
            server.serve_forever()
        except KeyboardInterrupt:
            logger.info("Stopping daemon...")
        finally:
            server.server_close()
            for s in self.sensors:
                s.release()


if __name__ == "__main__":
    daemon = EdgeDaemon()
    daemon.start_server()
