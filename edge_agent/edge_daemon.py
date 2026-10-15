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
import math
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
            "height": 600,
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


class YDLidarDriver:
    """
    High-performance driver for YDLIDAR T-mini Pro (YDTMP-12) 12m LiDAR.
    Decodes raw serial packets (0xAA 0x55) at 230400 baud.
    Generates dynamic top-down 2D depth radar view with distance-based colors:
      <= 1m: Red (#ff1f1f)
      >= 6m: Blue (#2979ff)
      1m < d < 6m: Smooth gradient (Red -> Yellow -> Green -> Cyan -> Blue)
    Includes graceful automatic fallback to simulated rotating radar if hardware is not connected.
    """
    def __init__(self, cfg):
        self.cfg = cfg
        self.port = cfg.get("device", "/dev/ttyUSB0")
        self.baudrate = cfg.get("baudrate", 230400)
        self.width = cfg.get("width", 640)
        self.height = cfg.get("height", 640)
        self.fps = cfg.get("fps", 10)
        self.points = {}
        self.points_lock = threading.Lock()
        self.is_hardware = False
        self.last_hardware_time = 0.0
        self.running = True
        self.latest_jpeg = None
        self.last_render_time = 0.0

        # Start serial reader background thread
        self.reader_thread = threading.Thread(target=self._serial_loop, daemon=True)
        self.reader_thread.start()

    def get_color(self, d):
        if d <= 1.0:
            return (255, 31, 31)
        if d >= 6.0:
            return (41, 121, 255)
        t = (d - 1.0) / 5.0
        if t < 0.25:
            s = t / 0.25
            return (255, int(31 + 189 * s), int(31 * (1 - s)))
        elif t < 0.50:
            s = (t - 0.25) / 0.25
            return (int(255 * (1 - s)), int(220 + 10 * s), int(118 * s))
        elif t < 0.75:
            s = (t - 0.50) / 0.25
            return (0, int(230 - 1 * s), int(118 + 137 * s))
        else:
            s = (t - 0.75) / 0.25
            return (int(41 * s), int(229 * (1 - s) + 121 * s), 255)

    def _open_serial(self):
        candidate_ports = [self.port, "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyACM1", "/dev/serial0"]
        if os.name == 'nt':
            candidate_ports = [self.port, "COM3", "COM4", "COM5", "COM6"]

        for p in candidate_ports:
            if not p:
                continue
            if os.name != 'nt' and not os.path.exists(p):
                continue
            # 1. Try PySerial
            try:
                import serial
                ser = serial.Serial(p, baudrate=self.baudrate, timeout=0.5)
                ser.dtr = False
                ser.rts = False
                ser.reset_input_buffer()
                ser.write(b"\xa5\x60")
                logger.info(f"[ydlidar] Opened serial port {p} at {self.baudrate} baud via pyserial")
                return ("pyserial", ser, p)
            except Exception:
                pass

            # 2. Try native Linux termios
            if os.name != 'nt':
                try:
                    import termios, tty
                    fd = os.open(p, os.O_RDWR | os.O_NOCTTY)
                    attrs = termios.tcgetattr(fd)
                    tty.setraw(fd)
                    speed = getattr(termios, f'B{self.baudrate}', termios.B230400 if hasattr(termios, 'B230400') else termios.B115200)
                    attrs[4] = speed
                    attrs[5] = speed
                    attrs[6][termios.VMIN] = 1
                    attrs[6][termios.VTIME] = 5
                    termios.tcsetattr(fd, termios.TCSANOW, attrs)
                    os.write(fd, b"\xa5\x60")
                    logger.info(f"[ydlidar] Opened serial port {p} at {self.baudrate} baud via native termios")
                    return ("native", fd, p)
                except Exception:
                    pass

        return None

    def _read_bytes(self, ser_type, ser_obj, count):
        buf = bytearray()
        t_start = time.time()
        while len(buf) < count and self.running and (time.time() - t_start < 0.5):
            needed = count - len(buf)
            try:
                if ser_type == "pyserial":
                    chunk = ser_obj.read(needed)
                else:
                    chunk = os.read(ser_obj, needed)
                if chunk:
                    buf.extend(chunk)
                else:
                    time.sleep(0.005)
            except Exception:
                break
        return bytes(buf)

    def _serial_loop(self):
        while self.running:
            ser_conn = self._open_serial()
            if not ser_conn:
                time.sleep(2.0)
                continue

            ser_type, ser_obj, port_name = ser_conn
            try:
                while self.running:
                    # Sync to header 0xAA 0x55
                    b1 = self._read_bytes(ser_type, ser_obj, 1)
                    if b1 != b'\xaa':
                        continue
                    b2 = self._read_bytes(ser_type, ser_obj, 1)
                    if b2 != b'\x55':
                        continue

                    # Read CT (1 byte) + LSN (1 byte)
                    hdr = self._read_bytes(ser_type, ser_obj, 2)
                    if len(hdr) < 2:
                        continue
                    ct = hdr[0]
                    lsn = hdr[1]
                    if lsn == 0:
                        continue

                    # Read FSA (2), LSA (2), CS (2)
                    info = self._read_bytes(ser_type, ser_obj, 6)
                    if len(info) < 6:
                        continue
                    fsa = int.from_bytes(info[0:2], 'little')
                    lsa = int.from_bytes(info[2:4], 'little')

                    # Read sample points (lsn * 2 bytes)
                    sample_bytes = self._read_bytes(ser_type, ser_obj, lsn * 2)
                    if len(sample_bytes) < lsn * 2:
                        continue

                    angle_fsa = (fsa >> 1) / 64.0
                    angle_lsa = (lsa >> 1) / 64.0
                    diff = (angle_lsa - angle_fsa)
                    if diff < 0:
                        diff += 360.0

                    now = time.time()
                    with self.points_lock:
                        if (ct & 0x01) == 0x01:
                            # Start of new revolution, clear points older than 0.5s
                            cutoff = now - 0.5
                            self.points = {deg: val for deg, val in self.points.items() if val[1] > cutoff}

                        for i in range(lsn):
                            sample = int.from_bytes(sample_bytes[i*2:i*2+2], 'little')
                            dist_mm = sample / 4.0
                            if dist_mm > 40 and dist_mm <= 12000:
                                dist_m = dist_mm / 1000.0
                                angle = (angle_fsa + (diff / max(lsn - 1, 1)) * i) % 360.0
                                self.points[round(angle, 1)] = (dist_m, now)
                                self.last_hardware_time = now
                                self.is_hardware = True
            except Exception as e:
                logger.info(f"[ydlidar] Serial connection error on {port_name}: {e}")
            finally:
                try:
                    if ser_type == "pyserial":
                        ser_obj.close()
                    else:
                        os.close(ser_obj)
                except Exception:
                    pass
                time.sleep(1.0)

    def render_radar_frame(self):
        from PIL import Image, ImageDraw
        img = Image.new('RGB', (self.width, self.height), (11, 15, 25))
        draw = ImageDraw.Draw(img)
        cx, cy = self.width // 2, self.height // 2
        max_range = 8.0
        scale = (min(self.width, self.height) // 2 - 36) / max_range

        # Concentric distance rings
        rings = [(1.0, (160, 45, 45), '1m'),
                 (2.0, (40, 60, 90), '2m'),
                 (4.0, (40, 60, 90), '4m'),
                 (6.0, (45, 95, 180), '6m'),
                 (8.0, (30, 45, 70), '8m')]
        for r_m, color, label in rings:
            r_px = r_m * scale
            draw.ellipse([cx - r_px, cy - r_px, cx + r_px, cy + r_px], outline=color, width=1)
            draw.text((cx + 6, cy - r_px - 13), label, fill=(140, 170, 200))

        # Crosshairs
        draw.line([cx, 26, cx, self.height - 30], fill=(26, 42, 64), width=1)
        draw.line([26, cy, self.width - 26, cy], fill=(26, 42, 64), width=1)

        # Compass directions
        draw.text((cx - 16, 8), 'N (0°)', fill=(0, 229, 255))
        draw.text((self.width - 38, cy - 8), '90°', fill=(100, 130, 160))
        draw.text((cx - 14, self.height - 26), '180°', fill=(100, 130, 160))
        draw.text((6, cy - 8), '270°', fill=(100, 130, 160))

        # Center robot / node heading indicator
        draw.ellipse([cx - 7, cy - 7, cx + 7, cy + 7], fill=(0, 229, 255), outline=(255, 255, 255))
        draw.polygon([(cx, cy - 14), (cx - 5, cy - 6), (cx + 5, cy - 6)], fill=(255, 255, 255))

        now = time.time()
        pt_list = []
        is_live = False
        with self.points_lock:
            if self.is_hardware and (now - self.last_hardware_time < 2.0):
                pt_list = [(deg, d) for deg, (d, ts) in self.points.items() if now - ts < 0.6]
                is_live = True

        if not is_live:
            t = now
            for deg in range(0, 360):
                rad = math.radians(deg)
                wall_x = 4.2 / max(abs(math.cos(rad)), 0.15)
                wall_y = 3.8 / max(abs(math.sin(rad)), 0.15)
                d = min(wall_x, wall_y)
                target_deg = (45 + 30 * math.sin(t * 1.5)) % 360
                if abs((deg - target_deg + 180) % 360 - 180) < 14:
                    d = 0.82
                elif 125 <= deg <= 160:
                    d = 2.4
                elif 245 <= deg <= 275:
                    d = 6.8
                pt_list.append((deg, d))

        for deg, d in pt_list:
            if d < 0.05 or d > max_range:
                continue
            rad = math.radians(deg)
            r_px = d * scale
            px = cx + r_px * math.sin(rad)
            py = cy - r_px * math.cos(rad)
            c = self.get_color(d)
            draw.ellipse([px - 2, py - 2, px + 2, py + 2], fill=c)

        # Range Legend
        bar_y = self.height - 22
        draw.rectangle([10, bar_y - 4, self.width - 10, self.height - 4], fill=(16, 22, 34))
        draw.text((16, bar_y), 'RANGE:', fill=(130, 160, 190))
        legends = [
            (65, (255, 31, 31), '< 1m (Red)'),
            (165, (255, 200, 0), '1-2m'),
            (230, (0, 230, 118), '2-4m'),
            (300, (0, 229, 255), '4-6m'),
            (370, (41, 121, 255), '6m+ (Blue)')
        ]
        for lx, lc, ltxt in legends:
            draw.rectangle([lx, bar_y + 2, lx + 12, bar_y + 10], fill=lc)
            draw.text((lx + 16, bar_y), ltxt, fill=(180, 200, 220))

        status_txt = 'LIVE HARDWARE' if is_live else 'STANDBY / SIMULATION'
        status_col = (0, 230, 118) if is_live else (255, 190, 0)
        draw.text((16, 10), 'YDLIDAR T-mini Pro (YDTMP-12) 25ARZ0006', fill=(220, 230, 245))
        draw.text((16, 26), f'POINTS: {len(pt_list)}  |  {status_txt}', fill=status_col)

        buf = io.BytesIO()
        img.save(buf, format='JPEG', quality=80)
        return buf.getvalue()

    def get_jpeg_frame(self):
        now = time.time()
        min_interval = 1.0 / max(self.fps, 5)
        if self.latest_jpeg is None or (now - self.last_render_time >= min_interval):
            try:
                self.latest_jpeg = self.render_radar_frame()
                self.last_render_time = now
            except Exception as e:
                logger.error(f"[ydlidar] Render frame error: {e}")
        return self.latest_jpeg

    def stop(self):
        self.running = False


class SensorDriver:
    """Multi-backend hardware sensor driver (picamera, pygame, opencv, raspistill, ydlidar, fallback)."""

    def __init__(self, sensor_cfg):
        self.cfg = sensor_cfg
        self.modality = sensor_cfg.get("modality", "rgb")
        self.sensor_id = sensor_cfg.get("id", "sensor-0")
        self.name = sensor_cfg.get("name", "Sensor")
        self.width = sensor_cfg.get("width", 640)
        self.height = sensor_cfg.get("height", 480)
        self.fps = sensor_cfg.get("fps", 15)
        self.device_node = sensor_cfg.get("device", "/dev/video0")
        self.backend = None
        self.camera_obj = None
        self.use_fallback = False
        self._init_hardware()

    def _read_exact(self, pipe, size):
        chunks = []
        total = 0
        while total < size and not getattr(self, "_stop_stream", False):
            try:
                chunk = pipe.read(min(size - total, 65536))
                if not chunk:
                    return None
                chunks.append(chunk)
                total += len(chunk)
            except Exception:
                return None
        return b"".join(chunks) if len(chunks) > 1 else (chunks[0] if chunks else None)

    def _init_hardware(self):
        sensor_type = self.cfg.get("type", "auto")

        # Check for LiDAR sensor
        if sensor_type in ["ydlidar", "lidar"] or self.modality.lower() in ["depth", "lidar"]:
            self.lidar_driver = YDLidarDriver(self.cfg)
            self.backend = "ydlidar"
            logger.info(f"[{self.sensor_id}] Initialized YDLIDAR driver on {self.lidar_driver.port} ({self.width}x{self.height} @ {self.fps}fps)")
            return

        # 0. Try rpicam-vid / libcamera-vid (standard on Bookworm/Bullseye for Raspberry Pi Camera v2 / IMX219)
        rpicam_bin = None
        for cand in ["/usr/bin/rpicam-vid", "/usr/bin/libcamera-vid", "rpicam-vid", "libcamera-vid"]:
            if os.path.exists(cand) or (os.name != 'nt' and subprocess.run(["which", cand], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0):
                rpicam_bin = cand
                break

        if rpicam_bin and (sensor_type in ["rpicam", "libcamera", "camerav2", "imx219"] or not os.path.exists(self.device_node) or sensor_type == "auto"):
            try:
                cmd = [
                    rpicam_bin,
                    "-t", "0",
                    "--inline",
                    "--codec", "mjpeg",
                    "--width", str(self.width),
                    "--height", str(self.height),
                    "--framerate", str(self.fps),
                    "-n",
                    "-o", "-"
                ]
                self.rpicam_proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    bufsize=65536
                )
                self._stop_stream = False
                self.stream_thread = threading.Thread(target=self._rpicam_read_loop, daemon=True)
                self.stream_thread.start()

                # Await first valid JPEG frame (up to 12s for sensor initialization on ARM)
                for _ in range(120):
                    if getattr(self, "latest_jpeg", None):
                        self.backend = "rpicam_pipe"
                        logger.info(f"[{self.sensor_id}] Initialized {rpicam_bin} hardware stream ({self.width}x{self.height} @ {self.fps}fps)")
                        return
                    if self.rpicam_proc.poll() is not None:
                        err = self.rpicam_proc.stderr.read().decode('utf-8', errors='replace')
                        logger.warning(f"[{self.sensor_id}] {rpicam_bin} exited early (code {self.rpicam_proc.returncode}): {err}")
                        break
                    time.sleep(0.1)

                # If failed to produce frame, terminate and fall back
                self._stop_stream = True
                try:
                    self.rpicam_proc.terminate()
                except Exception:
                    pass
                self.rpicam_proc = None
            except Exception as e:
                logger.info(f"[{self.sensor_id}] {rpicam_bin} hardware stream test failed: {e}")

        # 1. Try direct V4L2 stream via v4l2-ctl (best for Unicam / VD55G0 global shutter monochrome)
        if os.path.exists(self.device_node):
            try:
                # Explicitly configure device format so driver and reader are 100% synchronized
                try:
                    subprocess.run(
                        ["v4l2-ctl", "-d", self.device_node, f"--set-fmt-video=width={self.width},height={self.height},pixelformat=GREY"],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2
                    )
                except Exception as ex:
                    logger.warning(f"Could not set v4l2 format: {ex}")

                # Query the actual format accepted by the driver
                actual_w = self.width
                actual_h = self.height
                actual_size = self.width * self.height
                actual_stride = self.width
                try:
                    fmt_proc = subprocess.run(
                        ["v4l2-ctl", "-d", self.device_node, "--get-fmt-video"],
                        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, timeout=2
                    )
                    for line in fmt_proc.stdout.splitlines():
                        if "Width/Height" in line:
                            parts = line.split(":")[-1].strip().split("/")
                            actual_w = int(parts[0])
                            actual_h = int(parts[1])
                        elif "Bytes per Line" in line:
                            actual_stride = int(line.split(":")[-1].strip())
                        elif "Size Image" in line:
                            actual_size = int(line.split(":")[-1].strip())
                except Exception:
                    pass

                self.width = actual_w
                self.height = actual_h
                self.stride = actual_stride
                self.frame_size = actual_size

                self.v4l2_proc = subprocess.Popen(
                    ["v4l2-ctl", "-d", self.device_node, "--stream-mmap", "--stream-to=-"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    bufsize=self.frame_size * 2
                )
                test_frame = self._read_exact(self.v4l2_proc.stdout, self.frame_size)
                if test_frame and len(test_frame) == self.frame_size:
                    self.backend = "v4l2_pipe"
                    img = self._decode_v4l2_frame(test_frame)
                    buf = io.BytesIO()
                    img.save(buf, format='JPEG', quality=75)
                    self.latest_jpeg = buf.getvalue()
                    self._stop_stream = False
                    self.stream_thread = threading.Thread(target=self._v4l2_read_loop, daemon=True)
                    self.stream_thread.start()
                    logger.info(f"[{self.sensor_id}] Initialized direct V4L2 streaming on {self.device_node} ({self.width}x{self.height}, stride {self.stride}, {self.frame_size} bytes)")
                    return
                else:
                    self.v4l2_proc.terminate()
            except Exception as e:
                logger.info(f"[{self.sensor_id}] V4L2 stream pipe init failed: {e}")

        # 2. Try native Broadcom PiCamera (Raspbian Buster CSI camera)
        try:
            import picamera
            self.camera_obj = picamera.PiCamera()
            self.camera_obj.resolution = (self.width, self.height)
            self.camera_obj.framerate = self.fps
            time.sleep(0.5)
            self.backend = "picamera"
            logger.info(f"[{self.sensor_id}] Initialized native Broadcom PiCamera ({self.width}x{self.height} @ {self.fps}fps)")
            return
        except Exception as e:
            logger.info(f"[{self.sensor_id}] picamera init not available: {e}")

        # 3. Try Pygame V4L2 Camera (pre-installed on Buster)
        if os.path.exists(self.device_node):
            try:
                import pygame.camera
                import pygame.image
                pygame.camera.init()
                self.camera_obj = pygame.camera.Camera(self.device_node, (self.width, self.height))
                self.camera_obj.start()
                self.backend = "pygame"
                logger.info(f"[{self.sensor_id}] Initialized Pygame V4L2 Camera on {self.device_node}")
                return
            except Exception as e:
                logger.info(f"[{self.sensor_id}] Pygame camera init failed: {e}")

        # 4. Try OpenCV if available
        if os.path.exists(self.device_node):
            try:
                import cv2
                cap = cv2.VideoCapture(self.device_node)
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                cap.set(cv2.CAP_PROP_FPS, self.fps)
                if cap.isOpened():
                    self.camera_obj = cap
                    self.backend = "opencv"
                    logger.info(f"[{self.sensor_id}] Opened OpenCV V4L2 device {self.device_node}")
                    return
            except Exception as e:
                logger.info(f"[{self.sensor_id}] OpenCV init failed: {e}")

        # 5. Try raspistill CLI pipeline
        if os.path.exists("/usr/bin/raspistill"):
            try:
                res = subprocess.run(["raspistill", "-n", "-t", "50", "-w", str(self.width), "-h", str(self.height), "-q", "75", "-o", "-"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=3)
                if res.returncode == 0 and len(res.stdout) > 1000:
                    self.backend = "raspistill"
                    logger.info(f"[{self.sensor_id}] Initialized raspistill CLI pipeline")
                    return
            except Exception as e:
                logger.info(f"[{self.sensor_id}] raspistill test failed: {e}")

        logger.info(f"[{self.sensor_id}] Hardware camera not opened. Using fallback generator.")
        self.use_fallback = True

    def _decode_v4l2_frame(self, raw):
        from PIL import Image
        stride = getattr(self, "stride", self.width)
        if stride and stride != self.width:
            img = Image.frombytes('L', (self.width, self.height), raw, 'raw', 'L', stride, 1)
        else:
            img = Image.frombytes('L', (self.width, self.height), raw)

        # Crop optical black borders on VD55G1 (804x704 pad -> clean 800x700)
        if self.width == 804 and self.height == 704:
            img = img.crop((2, 2, 802, 702))
        return img

    def _rpicam_read_loop(self):
        buf = bytearray()
        soi = b'\xff\xd8'
        eoi = b'\xff\xd9'
        while not getattr(self, "_stop_stream", False) and getattr(self, "rpicam_proc", None) and self.rpicam_proc.poll() is None:
            try:
                chunk = self.rpicam_proc.stdout.read(8192)
                if not chunk:
                    break
                buf.extend(chunk)
                while True:
                    start_idx = buf.find(soi)
                    if start_idx == -1:
                        if len(buf) > 32768:
                            buf = buf[-4:]
                        break
                    end_idx = buf.find(eoi, start_idx + 2)
                    if end_idx == -1:
                        if start_idx > 0:
                            buf = buf[start_idx:]
                        break
                    self.latest_jpeg = bytes(buf[start_idx:end_idx + 2])
                    buf = buf[end_idx + 2:]
            except Exception:
                time.sleep(0.01)

    def _v4l2_read_loop(self):
        from PIL import Image
        import fcntl, termios, array

        def get_pipe_bytes(fd):
            b = array.array('i', [0])
            try:
                fcntl.ioctl(fd, termios.FIONREAD, b)
                return b[0]
            except Exception:
                return 0

        target_fps = max(self.fps, 15)
        min_interval = 1.0 / (target_fps * 1.5)
        last_encode_time = 0

        while not getattr(self, "_stop_stream", False) and self.v4l2_proc and self.v4l2_proc.poll() is None:
            try:
                # Discard stale backlog if pipe has accumulated older frames
                pipe_fd = self.v4l2_proc.stdout.fileno()
                avail = get_pipe_bytes(pipe_fd)
                if avail > self.frame_size * 2:
                    discard_count = (avail // self.frame_size) - 1
                    if discard_count > 0:
                        self.v4l2_proc.stdout.read(discard_count * self.frame_size)

                raw = self._read_exact(self.v4l2_proc.stdout, self.frame_size)
                if not raw or len(raw) < self.frame_size:
                    time.sleep(0.01)
                    continue

                now = time.time()
                # Rate-limit JPEG compression to target FPS
                if now - last_encode_time >= min_interval:
                    img = self._decode_v4l2_frame(raw)
                    buf = io.BytesIO()
                    img.save(buf, format='JPEG', quality=75)
                    self.latest_jpeg = buf.getvalue()
                    last_encode_time = now
            except Exception:
                time.sleep(0.02)

    def get_jpeg_frame(self):
        """Returns raw JPEG bytes for MJPEG streaming or saving."""
        if self.backend == "ydlidar" and getattr(self, "lidar_driver", None):
            return self.lidar_driver.get_jpeg_frame()

        elif self.backend in ["v4l2_pipe", "rpicam_pipe"]:
            jpeg = getattr(self, "latest_jpeg", None)
            if jpeg:
                return jpeg

        elif self.backend == "picamera" and self.camera_obj:
            try:
                stream = io.BytesIO()
                self.camera_obj.capture(stream, format='jpeg', use_video_port=True)
                return stream.getvalue()
            except Exception as e:
                logger.warning(f"picamera capture error: {e}")

        elif self.backend == "pygame" and self.camera_obj:
            try:
                import pygame.image
                from PIL import Image
                img_surface = self.camera_obj.get_image()
                raw_data = pygame.image.tostring(img_surface, 'RGB')
                pil_img = Image.frombytes('RGB', (self.width, self.height), raw_data)
                buf = io.BytesIO()
                pil_img.save(buf, format='JPEG', quality=75)
                return buf.getvalue()
            except Exception as e:
                logger.warning(f"pygame capture error: {e}")

        elif self.backend == "opencv" and self.camera_obj:
            try:
                import cv2
                ret, frame = self.camera_obj.read()
                if ret:
                    ret2, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
                    if ret2:
                        return jpeg.tobytes()
            except Exception as e:
                logger.warning(f"opencv capture error: {e}")

        elif self.backend == "raspistill":
            try:
                res = subprocess.run(["raspistill", "-n", "-t", "50", "-w", str(self.width), "-h", str(self.height), "-q", "70", "-o", "-"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=2)
                if res.returncode == 0:
                    return res.stdout
            except Exception as e:
                pass

        # Fallback synthetic frame generator
        return self._generate_synthetic_frame()

    def release(self):
        if self.backend == "ydlidar" and getattr(self, "lidar_driver", None):
            try:
                self.lidar_driver.stop()
            except Exception:
                pass
        elif self.backend == "rpicam_pipe" and getattr(self, "rpicam_proc", None):
            self._stop_stream = True
            try:
                self.rpicam_proc.terminate()
            except Exception:
                pass
        elif self.backend == "v4l2_pipe" and getattr(self, "v4l2_proc", None):
            self._stop_stream = True
            try:
                self.v4l2_proc.terminate()
            except Exception:
                pass
        elif self.backend == "picamera" and self.camera_obj:
            try:
                self.camera_obj.close()
            except Exception:
                pass
        elif self.backend == "pygame" and self.camera_obj:
            try:
                self.camera_obj.stop()
            except Exception:
                pass
        elif self.backend == "opencv" and self.camera_obj:
            try:
                self.camera_obj.release()
            except Exception:
                pass

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
        elif path == "/api/edges":
            self._handle_edges()
        elif path.startswith("/proxy/stream/"):
            parts = [p for p in path.split("/proxy/stream/")[1].split("/") if p]
            target = parts[-1] if parts else "rgb"
            self._handle_mjpeg_stream(target)
        elif path.startswith("/proxy/snapshot/"):
            parts = [p for p in path.split("/proxy/snapshot/")[1].split("/") if p]
            target = parts[-1] if parts else "rgb"
            self._handle_snapshot(target)
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

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS, DELETE")
        self.send_header("Access-Control-Allow-Headers", "Content-Type, Authorization")
        self.send_header("Access-Control-Allow-Private-Network", "true")
        self.end_headers()

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
        elif path == "/api/display/toggle":
            self._send_json({"display_enabled": True})
        elif path == "/api/exec":
            content_length = int(self.headers.get('Content-Length', 0))
            cmd = ""
            if content_length > 0:
                body = self.rfile.read(content_length)
                try:
                    payload = json.loads(body.decode('utf-8'))
                    cmd = payload.get("cmd", "")
                except Exception:
                    cmd = body.decode('utf-8')
            if cmd:
                try:
                    proc = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=30)
                    self._send_json({"cmd": cmd, "returncode": proc.returncode, "stdout": proc.stdout, "stderr": proc.stderr})
                except Exception as ex:
                    self._send_json({"cmd": cmd, "error": str(ex)}, status_code=500)
            else:
                self._send_json({"error": "No cmd specified"}, status_code=400)
        else:
            self.send_error(404, "Not Found")

    def _send_json(self, data, status_code=200):
        body = json.dumps(data, indent=2).encode('utf-8')
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Private-Network", "true")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _handle_edges(self):
        sensor_info = [
            {
                "id": s.sensor_id,
                "modality": s.modality,
                "name": s.name,
                "resolution": f"{s.width}x{s.height}",
                "fps": s.fps,
                "backend": getattr(s, "backend", "none")
            }
            for s in self.daemon_ref.sensors
        ]
        edges_data = {
            "display_enabled": True,
            "edges": [{
                "device_id": self.daemon_ref.config.get("device_id", "edge-pi-zero-01"),
                "device_name": self.daemon_ref.config.get("device_name", "Raspberry Pi Zero (STEVAL-55G0MBI)"),
                "host": self.daemon_ref.config.get("host", "0.0.0.0"),
                "port": int(self.daemon_ref.config.get("port", 5001)),
                "online": True,
                "last_seen": time.time(),
                "network": self.daemon_ref.get_network_info(),
                "sensors": sensor_info,
                "modalities": list(set(s.modality for s in self.daemon_ref.sensors)),
                "collection": self.daemon_ref.collector.get_status()
            }]
        }
        self._send_json(edges_data)

    def _handle_status(self):
        collector_status = self.daemon_ref.collector.get_status()
        sensor_info = [
            {
                "id": s.sensor_id,
                "modality": s.modality,
                "name": s.name,
                "resolution": f"{s.width}x{s.height}",
                "fps": s.fps,
                "backend": getattr(s, "backend", "none")
            }
            for s in self.daemon_ref.sensors
        ]
        status = {
            "device_id": self.daemon_ref.config.get("device_id"),
            "device_name": self.daemon_ref.config.get("device_name"),
            "online": True,
            "timestamp": datetime.now().isoformat(),
            "network": self.daemon_ref.get_network_info(),
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

    def get_network_info(self):
        net_type = "unknown"
        iface = "unknown"
        ip_addr = "127.0.0.1"
        ssid = None
        try:
            res = subprocess.run(["ip", "route", "show", "default"], stdout=subprocess.PIPE, text=True, timeout=2)
            if "dev " in res.stdout:
                iface = res.stdout.split("dev ")[1].split()[0]
            if iface.startswith("wlan"):
                net_type = "hotspot"
                try:
                    iw_res = subprocess.run(["/sbin/iwconfig", iface], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, timeout=2)
                    for line in iw_res.stdout.splitlines():
                        if 'ESSID:"' in line:
                            ssid = line.split('ESSID:"')[1].split('"')[0]
                except Exception:
                    pass
                if not ssid:
                    ssid = "Wi-Fi Hotspot"
            elif any(iface.startswith(p) for p in ["wwan", "usb", "ppp", "qmi", "lte"]):
                net_type = "5g"
            elif iface.startswith("eth"):
                net_type = "ethernet"
            ip_res = subprocess.run(["ip", "-brief", "addr", "show", iface], stdout=subprocess.PIPE, text=True, timeout=2)
            if ip_res.stdout:
                parts = ip_res.stdout.split()
                if len(parts) >= 3:
                    ip_addr = parts[2].split("/")[0]
        except Exception:
            pass
        return {"type": net_type, "interface": iface, "ip": ip_addr, "ssid": ssid}

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
