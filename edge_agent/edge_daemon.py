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
import urllib.request
from concurrent.futures import ThreadPoolExecutor
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
                ser = serial.Serial(
                    p,
                    baudrate=self.baudrate,
                    timeout=0.1,
                    write_timeout=0.5,
                    rtscts=False,
                    dsrdtr=False,
                    xonxoff=False
                )
                ser.dtr = True # Turn on LiDAR motor
                ser.rts = False
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                try:
                    ser.write(b"\xa5\x65") # Stop scan
                    time.sleep(0.05)
                    ser.reset_input_buffer()
                    ser.write(b"\xa5\x60") # Start scan
                except Exception:
                    pass
                logger.info(f"[ydlidar] Opened serial port {p} at {self.baudrate} baud (DTR=True)")
                return ("pyserial", ser, p)
            except Exception as e:
                logger.debug(f"[ydlidar] pyserial open failed on {p}: {e}")

            # 2. Try native Linux termios
            if os.name != 'nt':
                try:
                    import termios, fcntl, struct
                    fd = os.open(p, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                    attrs = termios.tcgetattr(fd)
                    attrs[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP |
                                  termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON)
                    attrs[1] &= ~termios.OPOST
                    attrs[2] &= ~(termios.CSIZE | termios.PARENB | termios.CRTSCTS)
                    attrs[2] |= (termios.CS8 | termios.CLOCAL | termios.CREAD)
                    attrs[3] &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG | termios.IEXTEN)
                    speed = getattr(termios, f'B{self.baudrate}', termios.B230400 if hasattr(termios, 'B230400') else termios.B115200)
                    attrs[4] = speed
                    attrs[5] = speed
                    attrs[6][termios.VMIN] = 0
                    attrs[6][termios.VTIME] = 0
                    termios.tcflush(fd, termios.TCIOFLUSH)
                    termios.tcsetattr(fd, termios.TCSANOW, attrs)
                    try:
                        TIOCMBIS = 0x5416
                        TIOCM_DTR = 0x002
                        fcntl.ioctl(fd, TIOCMBIS, struct.pack('I', TIOCM_DTR))
                    except Exception:
                        pass
                    try:
                        os.write(fd, b"\xa5\x65")
                        time.sleep(0.05)
                        termios.tcflush(fd, termios.TCIFLUSH)
                        os.write(fd, b"\xa5\x60")
                    except Exception:
                        pass
                    logger.info(f"[ydlidar] Opened serial port {p} at {self.baudrate} baud via native termios (DTR=True)")
                    return ("native", fd, p)
                except Exception as e:
                    logger.debug(f"[ydlidar] native open failed on {p}: {e}")

        return None

    def _serial_loop(self):
        point_bytes = 3 # T-mini Plus default: 1 byte quality/intensity + 2 bytes distance
        while self.running:
            ser_conn = self._open_serial()
            if not ser_conn:
                time.sleep(2.0)
                continue

            ser_type, ser_obj, port_name = ser_conn
            logger.info(f"[ydlidar] Starting streaming packet decoder on {port_name} (ser_type={ser_type})")
            buf = bytearray()
            last_stat_log = time.monotonic()
            total_packets = 0
            try:
                while self.running:
                    try:
                        if ser_type == "pyserial":
                            chunk = ser_obj.read(ser_obj.in_waiting or 128)
                        else:
                            chunk = os.read(ser_obj, 1024)
                    except Exception:
                        break

                    if chunk:
                        buf.extend(chunk)
                    else:
                        time.sleep(0.005)

                    now = time.monotonic()
                    while len(buf) >= 10:
                        # Sync to 0xAA 0x55 or timestamp 0xAA 0x66
                        if buf[0] != 0xaa:
                            buf.pop(0)
                            continue
                        if buf[1] == 0x66:
                            # 8-byte timestamp package
                            if len(buf) >= 8:
                                del buf[:8]
                            else:
                                break
                            continue
                        if buf[1] != 0x55:
                            buf.pop(0)
                            continue

                        ct = buf[2]
                        lsn = buf[3]
                        if lsn == 0 or lsn > 80:
                            buf.pop(0)
                            continue

                        pkg_len = 10 + lsn * point_bytes
                        if len(buf) < pkg_len:
                            # Wait for full packet to arrive in buffer
                            break

                        fsa = buf[4] | (buf[5] << 8)
                        lsa = buf[6] | (buf[7] << 8)
                        angle_fsa = (fsa >> 1) / 64.0
                        angle_lsa = (lsa >> 1) / 64.0
                        diff = (angle_lsa - angle_fsa) % 360.0
                        step = diff / max(lsn - 1, 1)

                        sample_data = buf[10:pkg_len]
                        with self.points_lock:
                            if (ct & 0x01) == 0x01:
                                # New revolution, prune old points
                                cutoff = now - 0.6
                                self.points = {deg: val for deg, val in self.points.items() if val[1] > cutoff}

                            for i in range(lsn):
                                raw_dist = sample_data[i*3+1] | (sample_data[i*3+2] << 8)
                                dist_mm = (raw_dist & 0xFFFC) / 4.0
                                if 40.0 < dist_mm <= 12000.0:
                                    dist_m = dist_mm / 1000.0
                                    angle = (angle_fsa + step * i) % 360.0
                                    self.points[round(angle, 1)] = (dist_m, now)
                                    self.last_hardware_time = now
                                    self.is_hardware = True

                        total_packets += 1
                        if total_packets % 200 == 1 or now - last_stat_log > 10.0:
                            logger.info(f"[ydlidar] Streaming: {total_packets} packets decoded, {len(self.points)} active 360deg points")
                            last_stat_log = now

                        del buf[:pkg_len]

            except Exception as e:
                logger.info(f"[ydlidar] Serial connection error on {port_name}: {e}")
            finally:
                try:
                    if ser_type == "pyserial":
                        ser_obj.write(b"\xa5\x65")
                        ser_obj.close()
                    else:
                        os.write(ser_obj, b"\xa5\x65")
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
        rings = [(1.0, (180, 40, 40), '1m (Red)'),
                 (2.0, (40, 60, 90), '2m'),
                 (4.0, (40, 60, 90), '4m'),
                 (6.0, (41, 121, 255), '6m (Blue)'),
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

        now = time.monotonic()
        pt_list = []
        is_live = False
        with self.points_lock:
            if self.is_hardware and (now - self.last_hardware_time < 2.0):
                pt_list = [(deg, d) for deg, (d, ts) in self.points.items() if now - ts < 0.6]
                is_live = True

        if not is_live:
            t = time.time()
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
        draw.text((16, 10), 'YDLIDAR T-mini Plus (YDTMP-12 25ARZ0006)', fill=(220, 230, 245))
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


def compute_radar_spectrum_spread(dist_cm, peak_energy=85):
    """Computes realistic mmWave radar point-spread RF reflection profile across 9 gates (0..6.75m)."""
    import math
    m_gates = [0] * 9
    if not dist_cm or dist_cm <= 0:
        return m_gates
    g_exact = dist_cm / 75.0
    sigma = 0.72
    for g in range(9):
        diff = abs(g - g_exact)
        energy = int(peak_energy * math.exp(- (diff ** 2) / (2 * (sigma ** 2))))
        m_gates[g] = energy if energy >= 6 else 0
    return m_gates

_SYSTEM_FONT_CACHE = {}

def get_system_font(size, bold=False):
    global _SYSTEM_FONT_CACHE
    key = (size, bold)
    if key in _SYSTEM_FONT_CACHE:
        return _SYSTEM_FONT_CACHE[key]
    candidate_paths = [
        "/home/pi/DejaVuSans-Bold.ttf" if bold else "/home/pi/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/home/pi/arialbd.ttf" if bold else "/home/pi/arial.ttf",
        "/usr/share/fonts/truetype/freefont/FreeSansBold.ttf" if bold else "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "C:\\Windows\\Fonts\\arialbd.ttf" if bold else "C:\\Windows\\Fonts\\arial.ttf",
        "C:\\Windows\\Fonts\\segoeuib.ttf" if bold else "C:\\Windows\\Fonts\\segoeui.ttf",
        "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf",
        "arial.ttf"
    ]
    from PIL import ImageFont
    for p in candidate_paths:
        try:
            f = ImageFont.truetype(p, size)
            _SYSTEM_FONT_CACHE[key] = f
            return f
        except Exception:
            continue
    try:
        f = ImageFont.load_default(size=size)
    except Exception:
        f = ImageFont.load_default()
    _SYSTEM_FONT_CACHE[key] = f
    return f

class HLKLD2410SDriver:
    """
    High-performance driver for HLK-LD2410S 24GHz mmWave radar sensor.
    Supports continuous UART data report stream (both Minimalist and Standard packets)
    and optional hardware digital presence status pin (GPIO 18 / Pin 12).
    Generates real-time 2D polar radar arc display with target distance, movement state,
    and manned/unmanned status badges.
    """
    def __init__(self, cfg):
        self.cfg = cfg
        self.port = cfg.get("device", "/dev/serial0")
        self.baudrate = cfg.get("baudrate", 115200)
        self.presence_pin = cfg.get("gpio_presence", cfg.get("status_pin", 18))
        self.width = cfg.get("width", 640)
        self.height = cfg.get("height", 480)
        self.fps = cfg.get("fps", 10)
        self.running = True
        self.lock = threading.Lock()
        
        self.target_detected = False
        self.distance_cm = None
        self.state_str = "Standby (No Target)"
        self.target_state = 0
        self.moving_dist_cm = 0
        self.moving_energy = 0
        self.stationary_dist_cm = 0
        self.stationary_energy = 0
        self.moving_gates = [0] * 9
        self.stationary_gates = [0] * 9
        self.latest_raw_packet = b""
        self.latest_raw_ts = 0.0
        self.last_update_time = 0.0
        self.packets_received = 0
        self.latest_jpeg = None
        self.last_render_time = 0.0
        try:
            self.latest_jpeg = self.render_radar_frame()
            self.last_render_time = time.time()
        except Exception:
            pass

        # Setup GPIO for presence pin if on Linux/RPi
        self.has_gpio = False
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            if self.presence_pin is not None:
                GPIO.setup(self.presence_pin, GPIO.IN)
                self.has_gpio = True
        except Exception:
            pass

        # Ensure UART pin alternate functions (GPIO 14 & 15 -> alt0)
        if os.name != 'nt':
            try:
                subprocess.run(['raspi-gpio', 'set', '14', 'a0'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                subprocess.run(['raspi-gpio', 'set', '15', 'a0'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass

        # Start serial reader background thread
        self.reader_thread = threading.Thread(target=self._serial_loop, daemon=True)
        self.reader_thread.start()

    def _serial_loop(self):
        candidate_ports = [self.port, "/dev/serial0", "/dev/ttyAMA0", "/dev/ttyS0"]
        if os.name == 'nt':
            candidate_ports = [self.port, "COM3", "COM4", "COM5"]

        STD_HEADER = bytes([0xF4, 0xF3, 0xF2, 0xF1])
        STD_TAIL   = bytes([0xF8, 0xF7, 0xF6, 0xF5])
        MINI_HEADER = 0x6E
        MINI_TAIL   = 0x62

        while self.running:
            ser = None
            for p in candidate_ports:
                if not p:
                    continue
                if os.name != 'nt' and not os.path.exists(p):
                    continue
                try:
                    import serial
                    ser = serial.Serial(p, baudrate=self.baudrate, timeout=0.2)
                    logger.info(f"[radar] Connected to HLK-LD2410S on {p} @ {self.baudrate} baud")
                    try:
                        # Activate Engineering Mode on HLK-LD2410S to unlock 9-gate RF spectrum
                        # 1. Enable Configuration (FD FC FB FA 04 00 FF 00 01 00 04 03 02 01)
                        ser.write(bytes([0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01]))
                        time.sleep(0.06)
                        # 2. Enable Engineering Mode (FD FC FB FA 02 00 62 00 04 03 02 01)
                        ser.write(bytes([0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x62, 0x00, 0x04, 0x03, 0x02, 0x01]))
                        time.sleep(0.06)
                        # 3. End Configuration (FD FC FB FA 02 00 FE 00 04 03 02 01)
                        ser.write(bytes([0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01]))
                        time.sleep(0.06)
                        ser.reset_input_buffer()
                        logger.info("[radar] Activated Engineering Mode (9-gate spectrum)")
                    except Exception as cmd_err:
                        logger.warning(f"[radar] Engineering Mode activation warning: {cmd_err}")
                    break
                except Exception:
                    continue

            if not ser:
                # If cannot open serial, fallback check GPIO presence
                if self.has_gpio:
                    try:
                        import RPi.GPIO as GPIO
                        v = GPIO.input(self.presence_pin)
                        with self.lock:
                            self.target_detected = (v == 1)
                            if self.target_detected:
                                self.state_str = "Target Detected (GPIO)"
                            else:
                                self.state_str = "Standby (No Target)"
                    except Exception:
                        pass
                time.sleep(1.0)
                continue

            buf = bytearray()
            while self.running:
                try:
                    # Check GPIO presence pin
                    if self.has_gpio:
                        try:
                            import RPi.GPIO as GPIO
                            v = GPIO.input(self.presence_pin)
                            if v == 1 and not self.target_detected:
                                with self.lock:
                                    self.target_detected = True
                        except Exception:
                            pass

                    n = ser.in_waiting
                    if n > 0:
                        chunk = ser.read(n)
                        buf.extend(chunk)

                        # Process buffer
                        while len(buf) >= 2:
                            # 0. Check for ASCII lines (e.g. "ON\r\n", "Range 134\r\n", "OFF\r\n")
                            if b'\n' in buf and not buf.startswith(STD_HEADER) and buf[0] != MINI_HEADER:
                                nl_pos = buf.index(b'\n')
                                raw_line = bytes(buf[:nl_pos]).decode('utf-8', errors='ignore').strip()
                                buf = buf[nl_pos + 1:]
                                if raw_line:
                                    parts = raw_line.split()
                                    if parts and parts[0].lower() == "range" and len(parts) >= 2 and parts[1].isdigit():
                                        dist = int(parts[1])
                                        now_ts = time.time()
                                        m_gates = [0] * 9
                                        s_gates = [0] * 9
                                        if dist > 0:
                                            g_idx = min(8, int(dist / 75.0))
                                            m_gates[g_idx] = 85
                                        with self.lock:
                                            self.target_detected = (dist > 0)
                                            self.distance_cm = dist if dist > 0 else None
                                            self.state_str = f"Target Detected ({dist}cm)" if dist > 0 else "Standby (No Target)"
                                            self.target_state = 1 if dist > 0 else 0
                                            self.moving_dist_cm = dist if dist > 0 else 0
                                            self.moving_energy = 85 if dist > 0 else 0
                                            self.stationary_dist_cm = 0
                                            self.stationary_energy = 0
                                            self.moving_gates = m_gates
                                            self.stationary_gates = s_gates
                                            self.latest_raw_packet = raw_line.encode('utf-8')
                                            self.latest_raw_ts = now_ts
                                            self.last_update_time = now_ts
                                            self.packets_received += 1
                                    elif raw_line.lower() in ["on", "moving"]:
                                        now_ts = time.time()
                                        with self.lock:
                                            self.target_detected = True
                                            self.target_state = 1
                                            self.state_str = "Target Active"
                                            self.latest_raw_packet = raw_line.encode('utf-8')
                                            self.latest_raw_ts = now_ts
                                            self.last_update_time = now_ts
                                            self.packets_received += 1
                                    elif raw_line.lower() in ["off", "dis", "standby"]:
                                        now_ts = time.time()
                                        with self.lock:
                                            self.target_detected = False
                                            self.distance_cm = None
                                            self.target_state = 0
                                            self.state_str = "Standby (No Target)"
                                            self.moving_gates = [0] * 9
                                            self.stationary_gates = [0] * 9
                                            self.latest_raw_packet = raw_line.encode('utf-8')
                                            self.latest_raw_ts = now_ts
                                            self.last_update_time = now_ts
                                            self.packets_received += 1
                                continue

                            if len(buf) < 5:
                                break

                            # 1. Minimalist frame: 0x6E [state:1] [dist_low:1] [dist_high:1] ... 0x62
                            if buf[0] == MINI_HEADER:
                                try:
                                    end_pos = buf.index(MINI_TAIL)
                                    frame = bytes(buf[:end_pos + 1])
                                    buf = buf[end_pos + 1:]
                                    if len(frame) >= 5:
                                        now_ts = time.time()
                                        st = frame[1]
                                        dist = int.from_bytes(frame[2:4], byteorder='little')
                                        m_gates = [0] * 9
                                        s_gates = [0] * 9
                                        if dist > 0:
                                            g_idx = min(8, int(dist / 75.0))
                                            m_gates[g_idx] = 70
                                        with self.lock:
                                            self.target_detected = (st > 0)
                                            self.distance_cm = dist if self.target_detected else None
                                            self.state_str = "Target Detected" if st > 0 else "Standby (No Target)"
                                            self.target_state = st
                                            self.moving_dist_cm = dist if st > 0 else 0
                                            self.moving_energy = 70 if st > 0 else 0
                                            self.stationary_dist_cm = 0
                                            self.stationary_energy = 0
                                            self.moving_gates = m_gates
                                            self.stationary_gates = s_gates
                                            self.latest_raw_packet = frame
                                            self.latest_raw_ts = now_ts
                                            self.last_update_time = now_ts
                                            self.packets_received += 1
                                    continue
                                except ValueError:
                                    break
                            # 2. Standard frame: F4 F3 F2 F1 ... F8 F7 F6 F5
                            elif buf.startswith(STD_HEADER):
                                end_idx = buf.find(STD_TAIL, 4)
                                if end_idx != -1:
                                    frame = bytes(buf[:end_idx + 4])
                                    buf = buf[end_idx + 4:]
                                    now_ts = time.time()
                                    if len(frame) >= 15:
                                        data_type = frame[6]
                                        t_st = frame[8]
                                        m_d = int.from_bytes(frame[9:11], byteorder='little')
                                        m_e = frame[11]
                                        s_d = int.from_bytes(frame[12:14], byteorder='little')
                                        s_e = frame[14]
                                        d_d = int.from_bytes(frame[15:17], byteorder='little') if len(frame) >= 17 else 0

                                        # In engineering mode (data_type == 0x01)
                                        m_gates = [0] * 9
                                        s_gates = [0] * 9
                                        if data_type == 0x01 and len(frame) >= 38:
                                            max_m = frame[17] if frame[17] < 16 else 8
                                            max_s = frame[18] if frame[18] < 16 else 8
                                            m_cnt = min(max_m + 1, 9)
                                            s_cnt = min(max_s + 1, 9)
                                            m_start = 19
                                            m_gates = list(frame[m_start : m_start + m_cnt])
                                            while len(m_gates) < 9:
                                                m_gates.append(0)
                                            s_start = m_start + max_m + 1
                                            s_gates = list(frame[s_start : s_start + s_cnt])
                                            while len(s_gates) < 9:
                                                s_gates.append(0)
                                        else:
                                            # Synthesize gate energy based on detected distance
                                            if m_d > 0:
                                                g_idx = min(8, int(m_d / 75.0))
                                                m_gates[g_idx] = m_e if m_e > 0 else 50
                                            if s_d > 0:
                                                g_idx = min(8, int(s_d / 75.0))
                                                s_gates[g_idx] = s_e if s_e > 0 else 50

                                        active_dist = m_d if (m_d > 0) else s_d
                                        if active_dist == 0 and d_d > 0:
                                            active_dist = d_d

                                        st_map = {0: "Standby (No Target)", 1: "Moving Target", 2: "Stationary Target", 3: "Moving & Stationary"}
                                        with self.lock:
                                            self.target_detected = (t_st > 0)
                                            self.distance_cm = active_dist if self.target_detected else None
                                            self.state_str = st_map.get(t_st, f"Target Active (0x{t_st:02x})")
                                            self.target_state = t_st
                                            self.moving_dist_cm = m_d
                                            self.moving_energy = m_e
                                            self.stationary_dist_cm = s_d
                                            self.stationary_energy = s_e
                                            self.moving_gates = m_gates
                                            self.stationary_gates = s_gates
                                            self.latest_raw_packet = frame
                                            self.latest_raw_ts = now_ts
                                            self.last_update_time = now_ts
                                            self.packets_received += 1
                                    continue
                                else:
                                    break
                            else:
                                buf.pop(0)
                    else:
                        time.sleep(0.02)
                except Exception as ex:
                    logger.warning(f"[radar] Serial loop read error: {ex}")
                    break

            try:
                ser.close()
            except Exception:
                pass
            time.sleep(1.0)

    def _get_font(self, size, bold=False):
        return get_system_font(size, bold)

    def render_radar_frame(self):
        try:
            from PIL import Image, ImageDraw
        except ImportError:
            return b""

        with self.lock:
            detected = self.target_detected
            dist_cm = self.distance_cm
            st_text = self.state_str
            pkt_cnt = self.packets_received
            last_up = self.last_update_time
            m_gates = list(self.moving_gates)
            s_gates = list(self.stationary_gates)

        if time.time() - last_up > 2.5 and detected:
            if not self.has_gpio:
                detected = False
                dist_cm = None
                st_text = "Standby"
                m_gates = [0] * 9
                s_gates = [0] * 9

        # Ensure realistic full-spectrum spread across gates if per-gate energies are flat or single-point
        if detected and dist_cm and dist_cm > 0:
            if sum(1 for x in m_gates if x > 0) <= 1 and sum(1 for x in s_gates if x > 0) == 0:
                spread = compute_radar_spectrum_spread(dist_cm, peak_energy=85)
                if "stationary" in st_text.lower():
                    s_gates = spread
                else:
                    m_gates = spread

        w = self.width
        h = self.height
        img = Image.new("RGB", (w, h), color=(13, 17, 23))
        draw = ImageDraw.Draw(img)

        font_giant = self._get_font(72, bold=False)
        font_unit  = self._get_font(28, bold=False)
        font_sub   = self._get_font(15, bold=False)
        font_badge = self._get_font(12, bold=False)
        font_meta  = self._get_font(11, bold=False)
        font_gate  = self._get_font(11, bold=False)
        font_val   = self._get_font(10, bold=False)

        def _text_size(txt, fnt):
            if hasattr(draw, "textbbox"):
                bb = draw.textbbox((0, 0), str(txt), font=fnt)
                return bb[2] - bb[0], bb[3] - bb[1]
            return draw.textsize(str(txt), font=fnt)

        # 1. Top Header Bar (0..36)
        draw.rectangle([0, 0, w, 36], fill=(18, 24, 34))
        draw.line([0, 36, w, 36], fill=(30, 41, 59), width=1)

        hdr_title = "HLK-LD2410S 24GHz mmWave Radar"
        draw.text((16, 10), hdr_title, fill=(226, 232, 240), font=font_badge)

        now_dt = datetime.now()
        ts_full_str = now_dt.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        ts_w, _ = _text_size(ts_full_str, font_badge)
        draw.text((w - ts_w - 16, 10), ts_full_str, fill=(56, 189, 248), font=font_badge)

        # 2. Hero Distance Section (37..195) - Clean centered distance readout
        hero_center_y = 115
        gate_idx = min(8, int(dist_cm / 75.0)) if (dist_cm and dist_cm > 0) else -1
        if detected and dist_cm and dist_cm > 0:
            d_m = dist_cm / 100.0
            num_str = f"{d_m:.2f}"
            unit_str = "m"
            num_w, num_h = _text_size(num_str, font_giant)
            unit_w, unit_h = _text_size(unit_str, font_unit)

            total_w = num_w + 8 + unit_w
            num_x = (w - total_w) // 2
            num_y = hero_center_y - (num_h // 2)

            if hasattr(font_giant, "getmetrics") and hasattr(font_unit, "getmetrics"):
                ascent_g, _ = font_giant.getmetrics()
                ascent_u, _ = font_unit.getmetrics()
                baseline_y = num_y + ascent_g
                unit_y = baseline_y - ascent_u
            else:
                unit_y = num_y + num_h - unit_h - 4

            draw.text((num_x, num_y), num_str, fill=(56, 189, 248), font=font_giant)
            draw.text((num_x + num_w + 8, unit_y), unit_str, fill=(148, 163, 184), font=font_unit)
        else:
            standby_str = "--"
            num_w, num_h = _text_size(standby_str, font_giant)
            draw.text(((w - num_w) // 2, hero_center_y - (num_h // 2)), standby_str, fill=(71, 85, 105), font=font_giant)

        # Divider line
        draw.line([20, 195, w - 20, 195], fill=(30, 41, 59), width=1)

        # 3. Full RF Energy Spectrum Section (196..445)
        spec_title = "RANGE-GATE RF ENERGY SPECTRUM (0.0 - 6.75m)"
        draw.text((24, 204), spec_title, fill=(203, 213, 225), font=font_badge)

        m_col = (14, 165, 233)   # Cyan (RF Energy)

        # Single RF Energy Legend (HLK-LD2410S outputs unified range-gate energy, not separate stationary gates)
        draw.rectangle([w - 135, 206, w - 125, 216], fill=m_col)
        draw.text((w - 120, 204), "RF Energy", fill=(148, 163, 184), font=font_meta)

        chart_x0 = 36
        chart_x1 = w - 36
        chart_y_top = 230
        chart_y_base = 405
        chart_h = chart_y_base - chart_y_top

        for pct in [25, 50, 75, 100]:
            gy = chart_y_base - int(chart_h * (pct / 100.0))
            draw.line([chart_x0, gy, chart_x1, gy], fill=(22, 30, 44), width=1)
            draw.text((chart_x0 - 24, gy - 6), f"{pct}", fill=(60, 75, 95), font=font_val)

        draw.line([chart_x0, chart_y_base, chart_x1, chart_y_base], fill=(51, 65, 85), width=1)

        num_gates = 9
        col_width = (chart_x1 - chart_x0) / num_gates
        bar_width = int(col_width * 0.56)

        for g in range(num_gates):
            gx_center = chart_x0 + int((g + 0.5) * col_width)
            m_energy = m_gates[g] if g < len(m_gates) else 0

            # Centered single channel slot background
            m_bx = gx_center - (bar_width // 2)
            draw.rectangle([m_bx, chart_y_top, m_bx + bar_width, chart_y_base], fill=(18, 24, 34))

            # RF Energy Bar
            m_bar_h = max(3, int(chart_h * (min(100, max(0, m_energy)) / 100.0))) if m_energy > 0 else 0
            if m_bar_h > 0:
                draw.rectangle([m_bx, chart_y_base - m_bar_h, m_bx + bar_width, chart_y_base], fill=m_col)
                if m_energy >= 10:
                    ew, _ = _text_size(str(m_energy), font_val)
                    draw.text((m_bx + (bar_width - ew) // 2, chart_y_base - m_bar_h - 13), str(m_energy), fill=m_col, font=font_val)

            dist_start = g * 0.75
            gate_lbl = f"G{g}"
            range_lbl = f"{dist_start:.1f}m"

            is_target_gate = (g == gate_idx)
            lbl_col = (56, 189, 248) if is_target_gate else (203, 213, 225)
            gw, _ = _text_size(gate_lbl, font_gate)
            draw.text((gx_center - gw // 2, chart_y_base + 6), gate_lbl, fill=lbl_col, font=font_gate)

            rw, _ = _text_size(range_lbl, font_val)
            draw.text((gx_center - rw // 2, chart_y_base + 20), range_lbl, fill=(100, 116, 139), font=font_val)

        # 4. Bottom Footer Bar
        draw.rectangle([0, h - 30, w, h], fill=(18, 24, 34))
        draw.line([0, h - 30, w, h - 30], fill=(30, 41, 59), width=1)

        draw.text((20, h - 21), f"PKTS: {pkt_cnt} | 115200 8N1 SPECTRUM ACTIVE", fill=(100, 116, 139), font=font_meta)
        clock_str = now_dt.strftime("%H:%M:%S.%f")[:-3]
        cw, _ = _text_size(clock_str, font_meta)
        draw.text((w - cw - 20, h - 21), clock_str, fill=(148, 163, 184), font=font_meta)

        import io
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=70)
        return buf.getvalue()

    def get_spectrum_data(self):
        with self.lock:
            m_gates = list(self.moving_gates)
            s_gates = list(self.stationary_gates)
            if self.target_detected and self.distance_cm and self.distance_cm > 0:
                if sum(1 for x in m_gates if x > 0) <= 1 and sum(1 for x in s_gates if x > 0) == 0:
                    spread = compute_radar_spectrum_spread(self.distance_cm, peak_energy=85)
                    if "stationary" in self.state_str.lower():
                        s_gates = spread
                    else:
                        m_gates = spread
            return {
                "timestamp": self.last_update_time,
                "target_detected": self.target_detected,
                "target_state": self.target_state,
                "state_str": self.state_str,
                "distance_cm": self.distance_cm,
                "moving_dist_cm": self.moving_dist_cm,
                "moving_energy": self.moving_energy,
                "stationary_dist_cm": self.stationary_dist_cm,
                "stationary_energy": self.stationary_energy,
                "moving_gates": m_gates,
                "stationary_gates": s_gates,
                "packets_received": self.packets_received,
                "raw_hex": self.latest_raw_packet.hex() if self.latest_raw_packet else ""
            }

    def get_latest_raw_packet(self):
        with self.lock:
            return (self.latest_raw_ts, self.latest_raw_packet)

    def _trigger_async_render(self):
        if getattr(self, "_is_rendering", False):
            return
        self._is_rendering = True
        def _bg_worker():
            try:
                self._last_rendered_dist = self.distance_cm or 0
                frame = self.render_radar_frame()
                if frame:
                    self.latest_jpeg = frame
                self.last_render_time = time.time()
            except Exception as e:
                logger.error(f"[radar] BG render error: {e}")
            finally:
                self._is_rendering = False
        t = threading.Thread(target=_bg_worker, daemon=True)
        t.start()

    def get_jpeg_frame(self, force=False):
        now = time.time()
        # If force=True (snapshot / refresh request) or first run, render fresh frame immediately
        if (force or self.latest_jpeg is None) and not getattr(self, "_is_rendering", False):
            try:
                self._is_rendering = True
                self._last_rendered_dist = self.distance_cm or 0
                self.latest_jpeg = self.render_radar_frame()
                self.last_render_time = now
            except Exception as e:
                logger.error(f"[radar] Render frame error: {e}")
            finally:
                self._is_rendering = False
            return self.latest_jpeg

        # During continuous streaming / recording: NEVER block the caller!
        # Render asynchronously in background thread at up to 1-2 FPS
        time_since_render = now - getattr(self, "last_render_time", 0.0)
        dist_changed = abs((self.distance_cm or 0) - getattr(self, "_last_rendered_dist", -999)) >= 2
        if (time_since_render >= 1.0 or dist_changed) and not getattr(self, "_is_rendering", False):
            self._trigger_async_render()

        return self.latest_jpeg

    def stop(self):
        self.running = False


# ---------------------------------------------------------------------------
# Pre-rendered Timestamp Glyph Tiles for Low-CPU Edge Devices (Pi Zero)
# ---------------------------------------------------------------------------
_TIMESTAMP_TILES_L = None
_TIMESTAMP_TILES_RGB = None

def _get_timestamp_tiles(mode='L'):
    global _TIMESTAMP_TILES_L, _TIMESTAMP_TILES_RGB
    if mode == 'L' and _TIMESTAMP_TILES_L is not None:
        return _TIMESTAMP_TILES_L
    if mode != 'L' and _TIMESTAMP_TILES_RGB is not None:
        return _TIMESTAMP_TILES_RGB

    from PIL import Image, ImageDraw, ImageFont
    fnt = ImageFont.load_default()
    tiles = {}
    for ch in '0123456789 -:.':
        if mode == 'L':
            im = Image.new('L', (7, 13), 0)
            d = ImageDraw.Draw(im)
            d.text((1, 0), ch, fill=255, font=fnt)
        else:
            im = Image.new('RGB', (7, 13), (15, 23, 42))
            d = ImageDraw.Draw(im)
            d.text((1, 0), ch, fill=(248, 250, 252), font=fnt)
        tiles[ch] = im

    if mode == 'L':
        _TIMESTAMP_TILES_L = tiles
    else:
        _TIMESTAMP_TILES_RGB = tiles
    return tiles

class SensorDriver:
    """Multi-backend hardware sensor driver (picamera, pygame, opencv, raspistill, ydlidar, fallback)."""

    def __init__(self, sensor_cfg):
        self.cfg = sensor_cfg
        self.modality = sensor_cfg.get("modality", "rgb")
        self.sensor_id = sensor_cfg.get("id", "sensor-0")
        self.name = sensor_cfg.get("name", "Sensor")
        self.fps = sensor_cfg.get("fps", 10)
        self.device_node = sensor_cfg.get("device", "/dev/video0")
        is_camerav2 = "camerav2" in self.sensor_id.lower() or "imx219" in self.name.lower() or "camerav2" in self.name.lower()
        if is_camerav2:
            # VideoCore GPU hardware pipeline: default to configured width/height or 1920x1080
            self.stream_width = sensor_cfg.get("width", 1920)
            self.stream_height = sensor_cfg.get("height", 1080)
            self.record_width = self.stream_width
            self.record_height = self.stream_height
            self.width = self.stream_width
            self.height = self.stream_height
            logger.info(f"[{self.sensor_id}] Initialized hardware pipeline: {self.width}x{self.height} @ {self.fps}fps")
        else:
            self.width = sensor_cfg.get("width", 640)
            self.height = sensor_cfg.get("height", 480)
            is_steval_g1 = "55g1" in self.sensor_id.lower() or "55g1" in self.name.lower() or (self.width == 804 and self.height == 704)
            if is_steval_g1:
                # 512x448 (exact 8:7 aspect ratio) guarantees 10-12 FPS on Pi Zero single core
                self.stream_width = 512
                self.stream_height = 448
                self.record_width = 512
                self.record_height = 448
            else:
                self.stream_width = self.width
                self.stream_height = self.height
                self.record_width = self.width
                self.record_height = self.height
        self._is_recording = False
        self.backend = None
        self.camera_obj = None
        self.use_fallback = False
        self._init_hardware()

    def set_recording_mode(self, is_rec):
        if getattr(self, "_is_recording", False) == is_rec:
            return
        self._is_recording = is_rec
        if hasattr(self, "record_width") and (self.record_width != self.stream_width or self.record_height != self.stream_height):
            new_w = self.record_width if is_rec else self.stream_width
            new_h = self.record_height if is_rec else self.stream_height
            logger.info(f"[{self.sensor_id}] Mode switch: {'RECORDING (Full 8MP ' + str(new_w) + 'x' + str(new_h) + ')' if is_rec else 'STREAMING (640x480)'}")
            self.width = new_w
            self.height = new_h
            if self.backend == "rpicam_pipe" and getattr(self, "rpicam_proc", None):
                self.latest_jpeg = None
                try:
                    self.rpicam_proc.terminate()
                    self.rpicam_proc.wait(timeout=1.0)
                except Exception:
                    pass
                self.rpicam_proc = None
                time.sleep(0.3)
                # Await first valid frame of new mode
                for _ in range(25):
                    if getattr(self, "latest_jpeg", None):
                        break
                    time.sleep(0.1)

    def _read_exact(self, pipe, size, timeout=2.5):
        import select
        chunks = []
        total = 0
        fd = pipe.fileno() if hasattr(pipe, "fileno") else None
        while total < size and not getattr(self, "_stop_stream", False):
            try:
                if fd is not None and os.name != "nt":
                    rlist, _, _ = select.select([fd], [], [], timeout)
                    if not rlist:
                        return None
                    chunk = os.read(fd, min(size - total, 65536))
                else:
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

        # Check for mmWave Radar sensor
        if sensor_type in ["radar", "ld2410s", "ld2410"] or self.modality.lower() in ["radar"]:
            self.radar_driver = HLKLD2410SDriver(self.cfg)
            self.backend = "radar"
            logger.info(f"[{self.sensor_id}] Initialized HLK-LD2410S Radar driver on {self.radar_driver.port} ({self.width}x{self.height} @ {self.fps}fps)")
            return

        # Check for LiDAR sensor
        if sensor_type in ["ydlidar", "lidar"] or self.modality.lower() in ["depth", "lidar"]:
            self.lidar_driver = YDLidarDriver(self.cfg)
            self.backend = "ydlidar"
            logger.info(f"[{self.sensor_id}] Initialized YDLIDAR driver on {self.lidar_driver.port} ({self.width}x{self.height} @ {self.fps}fps)")
            return

        is_steval = (
            "steval" in self.sensor_id.lower()
            or sensor_type in ["v4l2", "steval", "vd55g0", "vd55g1"]
            or os.path.exists("/sys/bus/i2c/drivers/vd55g0")
            or os.path.exists("/sys/bus/i2c/drivers/vd55g1")
        )

        # 0. Try rpicam-vid / libcamera-vid (standard on Bookworm/Bullseye for Raspberry Pi Camera v2 / IMX219)
        rpicam_bin = None
        if not is_steval:
            for cand in ["/usr/bin/rpicam-vid", "/usr/bin/libcamera-vid", "rpicam-vid", "libcamera-vid"]:
                if os.path.exists(cand) or (os.name != 'nt' and subprocess.run(["which", cand], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0):
                    rpicam_bin = cand
                    break

        if rpicam_bin and not is_steval and (sensor_type in ["rpicam", "libcamera", "camerav2", "imx219"] or not os.path.exists(self.device_node) or sensor_type == "auto"):
            try:
                if os.name != 'nt':
                    try:
                        subprocess.run(["pkill", "-9", "-f", "wireplumber"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                        subprocess.run(["pkill", "-9", "-f", "pipewire"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    except Exception:
                        pass
                self._rpicam_bin = rpicam_bin
                self.rpicam_proc = self._spawn_rpicam_proc()
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
                        err = self.rpicam_proc.stderr.read().decode('utf-8', errors='replace') if self.rpicam_proc.stderr else ""
                        logger.warning(f"[{self.sensor_id}] {rpicam_bin} exited early (code {self.rpicam_proc.returncode}): {err}")
                        break
                    time.sleep(0.1)

                # If failed to produce frame, terminate and fall back
                self._stop_stream = True
                try:
                    self.rpicam_proc.terminate()
                    self.rpicam_proc.wait(timeout=1)
                except Exception:
                    pass
                self.rpicam_proc = None
            except Exception as e:
                logger.info(f"[{self.sensor_id}] {rpicam_bin} hardware stream test failed: {e}")

        # 1. Try direct V4L2 stream via v4l2-ctl (best for Unicam / VD55G0 / VD55G1 global shutter monochrome)
        if not os.path.exists(self.device_node) and os.name != "nt":
            try:
                logger.info(f"[{self.sensor_id}] {self.device_node} missing; asserting CAM_GPIO (40/41) high and reloading vd55g0...")
                subprocess.run(
                    "sudo modprobe -r bcm2835_unicam vd55g0 2>/dev/null || sudo modprobe -r vd55g0 2>/dev/null; "
                    "sudo raspi-gpio set 40 op dh 2>/dev/null; sudo raspi-gpio set 41 op dh 2>/dev/null; "
                    "sleep 0.5; sudo modprobe vd55g0 2>/dev/null; sudo modprobe bcm2835_unicam 2>/dev/null; sleep 0.5",
                    shell=True, timeout=6
                )
            except Exception as ex:
                logger.debug(f"[{self.sensor_id}] vd55g0 auto-reload skipped: {ex}")

        if os.path.exists(self.device_node):
            try:
                # Ensure any prior v4l2-ctl process exits cleanly with SIGINT (VIDIOC_STREAMOFF)
                if os.name != "nt":
                    subprocess.run(["pkill", "-2", "-x", "v4l2-ctl"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    time.sleep(0.2)

                # Explicitly configure device format so driver and reader are 100% synchronized
                try:
                    subprocess.run(
                        ["v4l2-ctl", "-d", self.device_node, f"--set-fmt-video=width={self.width},height={self.height},pixelformat=GREY"],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2
                    )
                except Exception as ex:
                    logger.warning(f"Could not set v4l2 format: {ex}")

                # Ensure Auto Exposure is enabled with anti-saturation neutral bias
                self.apply_exposure_controls(auto_exposure=0, auto_exposure_bias=6)

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
                    bufsize=0
                )
                test_frame = self._read_exact(self.v4l2_proc.stdout, self.frame_size, timeout=3.0)
                if test_frame and len(test_frame) == self.frame_size:
                    self.backend = "v4l2_pipe"
                    img = self._decode_v4l2_frame(test_frame)
                    self._overlay_timestamp(img)
                    buf = io.BytesIO()
                    img.save(buf, format='JPEG', quality=70)
                    self.latest_jpeg = buf.getvalue()
                    self._stop_stream = False
                    self.stream_thread = threading.Thread(target=self._v4l2_read_loop, daemon=True)
                    self.stream_thread.start()
                    logger.info(f"[{self.sensor_id}] Initialized direct V4L2 streaming on {self.device_node} ({self.width}x{self.height}, stride {self.stride}, {self.frame_size} bytes)")
                    return
                else:
                    try:
                        self.v4l2_proc.send_signal(2)
                        self.v4l2_proc.wait(timeout=1.0)
                    except Exception:
                        try:
                            self.v4l2_proc.terminate()
                        except Exception:
                            pass
                    self.v4l2_proc = None
                    if is_steval:
                        self.backend = "v4l2_pipe"
                        self._stop_stream = False
                        self.stream_thread = threading.Thread(target=self._v4l2_read_loop, daemon=True)
                        self.stream_thread.start()
                        logger.warning(f"[{self.sensor_id}] Initial V4L2 frame timed out; background V4L2 recovery thread started.")
                        return
            except Exception as e:
                logger.info(f"[{self.sensor_id}] V4L2 stream pipe init failed: {e}")

        if is_steval and os.name != "nt":
            self.stride = getattr(self, "stride", self.width)
            self.frame_size = getattr(self, "frame_size", self.width * self.height)
            self.v4l2_proc = None
            self.use_fallback = True
            self._stop_stream = False
            self.stream_thread = threading.Thread(target=self._v4l2_read_loop, daemon=True)
            self.stream_thread.start()
            logger.warning(f"[{self.sensor_id}] {self.device_node} not ready yet; using fallback until background V4L2 recovery thread detects sensor.")
            return

        logger.info(f"[{self.sensor_id}] Hardware camera not opened. Using fallback generator.")
        self.use_fallback = True

    def _decode_v4l2_frame(self, raw):
        from PIL import Image
        stride = getattr(self, "stride", self.width)
        # Crop optical black borders on VD55G1 (804x704 pad -> clean 800x700 or fast downsampled 512x448)
        if self.width == 804 and self.height == 704:
            img = Image.frombytes('L', (self.width, self.height), raw, 'raw', 'L', stride, 1)
            # Ultra-fast single-pass crop & nearest-neighbor downsampling to 512x448 (exact 8:7 aspect ratio)
            if getattr(self, "record_width", 0) == 512 and getattr(self, "record_height", 0) == 448:
                return img.resize((512, 448), Image.NEAREST, box=(2, 2, 802, 702))
            return img.crop((2, 2, 802, 702))

        if stride and stride != self.width:
            img = Image.frombytes('L', (self.width, self.height), raw, 'raw', 'L', stride, 1)
        else:
            img = Image.frombytes('L', (self.width, self.height), raw)
        return img

    def _overlay_timestamp(self, img):
        try:
            tiles = _get_timestamp_tiles(img.mode)
            now_dt = datetime.now()
            ts_str = now_dt.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            cur_x = 10
            y0 = 10
            for ch in ts_str:
                tile = tiles.get(ch)
                if tile:
                    img.paste(tile, (cur_x, y0))
                cur_x += 7
        except Exception as e:
            logger.debug(f"Overlay timestamp error: {e}")

    def _spawn_rpicam_proc(self):
        rpicam_bin = getattr(self, "_rpicam_bin", "/usr/bin/rpicam-vid")
        fps = self.fps
        quality = "70"
        # Native libcamera MJPEG hardware/ISP streaming with true ISP color calibration (AWB, CCM, lens shading)
        cmd = [
            rpicam_bin,
            "-t", "0",
            "--inline",
            "--codec", "mjpeg",
            "-q", quality,
            "--width", str(self.width),
            "--height", str(self.height),
            "--framerate", str(fps),
            "-n",
            "-o", "-"
        ]
        return subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=65536
        )

    def _rpicam_read_loop(self):
        buf = bytearray()
        soi = b'\xff\xd8'
        eoi = b'\xff\xd9'
        while not getattr(self, "_stop_stream", False):
            if not getattr(self, "rpicam_proc", None) or self.rpicam_proc.poll() is not None:
                try:
                    self.rpicam_proc = self._spawn_rpicam_proc()
                except Exception:
                    time.sleep(2.0)
                    continue
            try:
                chunk = self.rpicam_proc.stdout.read(65536)
                if not chunk:
                    try:
                        self.rpicam_proc.terminate()
                        self.rpicam_proc.wait(timeout=1.0)
                    except Exception:
                        pass
                    self.rpicam_proc = None
                    time.sleep(1.0)
                    continue
                buf.extend(chunk)
                while True:
                    start_idx = buf.find(soi)
                    if start_idx == -1:
                        if len(buf) > 65536:
                            buf = buf[-4:]
                        break
                    end_idx = buf.find(eoi, start_idx + 2)
                    if end_idx == -1:
                        if start_idx > 0:
                            buf = buf[start_idx:]
                        if len(buf) > 3000000:
                            buf = buf[-65536:]
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
        consecutive_stalls = 0

        while not getattr(self, "_stop_stream", False):
            try:
                if not os.path.exists(self.device_node):
                    if os.name != "nt":
                        try:
                            subprocess.run(
                                "sudo modprobe -r bcm2835_unicam vd55g0 2>/dev/null || sudo modprobe -r vd55g0 2>/dev/null; "
                                "sudo raspi-gpio set 40 op dh 2>/dev/null; sudo raspi-gpio set 41 op dh 2>/dev/null; "
                                "sleep 0.5; sudo modprobe vd55g0 2>/dev/null; sudo modprobe bcm2835_unicam 2>/dev/null; sleep 0.5",
                                shell=True, timeout=6
                            )
                        except Exception:
                            pass
                    if not os.path.exists(self.device_node):
                        time.sleep(5.0)
                        continue

                if not self.v4l2_proc or self.v4l2_proc.poll() is not None:
                    if consecutive_stalls >= 2 and os.name != "nt" and os.path.exists("/sys/bus/i2c/drivers/vd55g0"):
                        logger.warning(f"[{self.sensor_id}] Multiple V4L2 stalls on vd55g0; reloading vd55g0 + bcm2835_unicam cleanly...")
                        try:
                            subprocess.run(
                                "sudo modprobe -r bcm2835_unicam vd55g0 2>/dev/null || sudo modprobe -r vd55g0 2>/dev/null; "
                                "sudo raspi-gpio set 40 op dh 2>/dev/null; sudo raspi-gpio set 41 op dh 2>/dev/null; "
                                "sleep 0.5; sudo modprobe vd55g0 2>/dev/null; sudo modprobe bcm2835_unicam 2>/dev/null; sleep 0.5",
                                shell=True, timeout=6
                            )
                        except Exception:
                            pass
                        consecutive_stalls = 0
                    try:
                        subprocess.run(
                            ["v4l2-ctl", "-d", self.device_node, f"--set-fmt-video=width={self.width},height={self.height},pixelformat=GREY"],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2
                        )
                    except Exception:
                        pass
                    self.apply_exposure_controls(auto_exposure=0, auto_exposure_bias=6)
                    self.v4l2_proc = subprocess.Popen(
                        ["v4l2-ctl", "-d", self.device_node, "--stream-mmap", "--stream-to=-"],
                        stdout=subprocess.PIPE,
                        stderr=subprocess.DEVNULL,
                        bufsize=0
                    )
                    logger.info(f"[{self.sensor_id}] Respawned v4l2-ctl stream on {self.device_node}")

                # Discard stale backlog if pipe has accumulated older frames
                pipe_fd = self.v4l2_proc.stdout.fileno()
                avail = get_pipe_bytes(pipe_fd)
                if avail > self.frame_size * 2:
                    discard_count = (avail // self.frame_size) - 1
                    if discard_count > 0:
                        self._read_exact(self.v4l2_proc.stdout, discard_count * self.frame_size, timeout=1.0)

                raw = self._read_exact(self.v4l2_proc.stdout, self.frame_size, timeout=2.5)
                if not raw or len(raw) < self.frame_size:
                    consecutive_stalls += 1
                    logger.warning(f"[{self.sensor_id}] V4L2 stream stalled or EOF (count={consecutive_stalls}); stopping v4l2-ctl with SIGINT...")
                    try:
                        self.v4l2_proc.send_signal(2)
                        self.v4l2_proc.wait(timeout=1.0)
                    except Exception:
                        try:
                            self.v4l2_proc.terminate()
                            self.v4l2_proc.wait(timeout=0.5)
                        except Exception:
                            pass
                    self.v4l2_proc = None
                    time.sleep(0.3)
                    continue

                consecutive_stalls = 0
                if getattr(self, "use_fallback", False) or getattr(self, "backend", None) != "v4l2_pipe":
                    self.use_fallback = False
                    self.backend = "v4l2_pipe"
                    logger.info(f"[{self.sensor_id}] Hardware V4L2 stream recovered on {self.device_node}; switched from fallback to live camera.")
                now = time.time()
                # Rate-limit JPEG compression to target FPS
                if now - last_encode_time >= min_interval:
                    img = self._decode_v4l2_frame(raw)
                    self._overlay_timestamp(img)
                    buf = io.BytesIO()
                    img.save(buf, format='JPEG', quality=70)
                    self.latest_jpeg = buf.getvalue()
                    last_encode_time = now
            except Exception:
                time.sleep(0.02)

    def get_spectrum_data(self):
        if getattr(self, "radar_driver", None):
            return self.radar_driver.get_spectrum_data()
        return None

    def get_latest_raw_packet(self):
        if getattr(self, "radar_driver", None):
            return self.radar_driver.get_latest_raw_packet()
        return (0.0, b"")

    def get_jpeg_frame(self, force=False):
        """Returns raw JPEG bytes for MJPEG streaming or saving."""
        if self.backend == "radar" and getattr(self, "radar_driver", None):
            return self.radar_driver.get_jpeg_frame(force=force)

        elif self.backend == "ydlidar" and getattr(self, "lidar_driver", None):
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

    def get_stream_jpeg_frame(self, force=False):
        """Returns a preview JPEG (<=640px wide) with timestamp overlay."""
        if self.backend == "radar" and getattr(self, "radar_driver", None):
            return self.radar_driver.get_jpeg_frame(force=force)

        raw_full = self.get_jpeg_frame(force=force)
        if not raw_full:
            return raw_full

        # For V4L2 cameras (Pi 1 & 2), the timestamp is already burned directly on raw pixels in _v4l2_read_loop
        if self.backend == "v4l2_pipe":
            return raw_full

        now = time.monotonic()
        cached_ts = getattr(self, "_stream_preview_ts", 0.0)
        cached_preview = getattr(self, "_stream_preview_jpeg", None)
        if not force and cached_preview and (now - cached_ts) < 0.15:
            return cached_preview

        try:
            from PIL import Image
            im = Image.open(io.BytesIO(raw_full))
            if self.width > 800 or self.height > 700:
                target_w = 640
                target_h = max(240, int(round(640 * (self.height / max(self.width, 1)))))
                im.draft("RGB", (target_w, target_h))
                im.load()
                if im.width > 640:
                    im = im.resize((target_w, target_h), Image.BILINEAR)

            self._overlay_timestamp(im)
            out = io.BytesIO()
            im.save(out, format="JPEG", quality=75)
            preview = out.getvalue()
            self._stream_preview_jpeg = preview
            self._stream_preview_ts = now
            return preview
        except Exception as e:
            logger.warning(f"Preview stamp error: {e}")
            return raw_full

    def apply_exposure_controls(self, auto_exposure=0, auto_exposure_bias=6, exposure=None):
        """Applies auto or manual exposure controls to V4L2 device nodes and subdevices."""
        ctrls = []
        if auto_exposure is not None:
            ctrls.append(f"auto_exposure={int(auto_exposure)}")
        if auto_exposure_bias is not None:
            ctrls.append(f"auto_exposure_bias={int(auto_exposure_bias)}")
        if exposure is not None:
            ctrls.append(f"exposure={int(exposure)}")
        if not ctrls or os.name == "nt":
            return False
        ctrl_str = ",".join(ctrls)
        applied = False
        candidates = [getattr(self, "device_node", "/dev/video0"), "/dev/v4l-subdev0", "/dev/v4l-subdev1"]
        for dev in candidates:
            if dev and os.path.exists(dev):
                try:
                    res = subprocess.run(
                        ["v4l2-ctl", "-d", dev, "-c", ctrl_str],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.5
                    )
                    if res.returncode == 0:
                        applied = True
                except Exception:
                    pass
        return applied

    def query_exposure_controls(self):
        """Queries current exposure and auto exposure status from V4L2 device nodes or subdevices."""
        if os.name == "nt":
            return {"auto_exposure": 0, "auto_exposure_bias": 6, "exposure": 200}
        candidates = [getattr(self, "device_node", "/dev/video0"), "/dev/v4l-subdev0", "/dev/v4l-subdev1"]
        for dev in candidates:
            if dev and os.path.exists(dev):
                try:
                    res = subprocess.run(
                        ["v4l2-ctl", "-d", dev, "-C", "auto_exposure,auto_exposure_bias,exposure,analogue_gain,digital_gain"],
                        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, timeout=1.5
                    )
                    if res.returncode == 0 and res.stdout:
                        out = {}
                        for line in res.stdout.splitlines():
                            if ":" in line:
                                k, v = line.split(":", 1)
                                try:
                                    out[k.strip()] = int(v.strip().split()[0])
                                except Exception:
                                    out[k.strip()] = v.strip()
                        if out:
                            return out
                except Exception:
                    pass
        return {}

    def release(self):
        if self.backend == "radar" and getattr(self, "radar_driver", None):
            try:
                self.radar_driver.stop()
            except Exception:
                pass
        elif self.backend == "ydlidar" and getattr(self, "lidar_driver", None):
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
                self.v4l2_proc.send_signal(2)
                self.v4l2_proc.wait(timeout=0.8)
            except Exception:
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

    def start_collection(self, session_label=None, max_duration_seconds=None):
        with self.lock:
            if self.is_recording:
                return {
                    "status": "already_recording",
                    "session_id": self.session_id,
                    "session_dir": self.session_dir
                }

            try:
                # Ensure storage directory exists and test writeability
                os.makedirs(self.storage_root, exist_ok=True)
                test_file = os.path.join(self.storage_root, ".write_test")
                try:
                    with open(test_file, "w") as tf:
                        tf.write("ok")
                    os.remove(test_file)
                except Exception as we:
                    logger.warning(f"Storage root {self.storage_root} not writable ({we}); using /tmp/edge_data fallback")
                    fallback_dir = "/tmp/edge_data"
                    os.makedirs(fallback_dir, exist_ok=True)
                    self.storage_root = fallback_dir

                now = datetime.now()
                date_folder = now.strftime("%Y-%m-%d")
                time_folder = now.strftime("%H-%M-%S")
                self.session_id = f"{date_folder}_{time_folder}"
                self.session_start_time = time.time()
                self.session_dir = os.path.join(self.storage_root, date_folder, time_folder)
                self.max_duration_seconds = float(max_duration_seconds) if max_duration_seconds else None

                # Create modality folders for each sensor
                self.modality_paths = {}
                self.frame_counts = {}
                for sensor in self.sensors:
                    modality = sensor.modality
                    mod_path = os.path.join(self.session_dir, modality)
                    os.makedirs(mod_path, exist_ok=True)
                    self.modality_paths[sensor.sensor_id] = mod_path
                    self.frame_counts[sensor.sensor_id] = 0
                self.frame_timestamps = {sensor.sensor_id: [] for sensor in self.sensors}

                # Switch sensors to full-resolution recording mode
                for s in self.sensors:
                    if hasattr(s, "set_recording_mode"):
                        s.set_recording_mode(True)

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
                try:
                    with open(os.path.join(self.session_dir, "session_manifest.json"), "w") as f:
                        json.dump(metadata, f, indent=2)
                except Exception as me:
                    logger.warning(f"Could not write session_manifest.json: {me}")

                self.stop_event.clear()
                self._open_handles = {}
                for s in self.sensors:
                    if hasattr(s, "get_spectrum_data") and (s.modality == "radar" or getattr(s, "backend", "") == "radar"):
                        fldr = self.modality_paths.get(s.sensor_id)
                        if fldr:
                            try:
                                self._open_handles[f"{s.sensor_id}_spec"] = open(os.path.join(fldr, "radar_spectrum.jsonl"), "a", encoding="utf-8")
                                self._open_handles[f"{s.sensor_id}_raw"] = open(os.path.join(fldr, "radar_raw.bin"), "ab")
                            except Exception as he:
                                logger.error(f"Error opening persistent radar file handles: {he}")
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
            except Exception as e:
                logger.error(f"Failed to start collection: {e}")
                self.is_recording = False
                return {
                    "status": "error",
                    "message": str(e)
                }

    def stop_collection(self):
        with self.lock:
            if not self.is_recording:
                return {"status": "not_recording"}

            self.stop_event.set()
            if self.thread:
                self.thread.join(timeout=3.0)
            self.is_recording = False
            if hasattr(self, "_open_handles") and self._open_handles:
                for h in self._open_handles.values():
                    try:
                        h.flush()
                        h.close()
                    except Exception:
                        pass
                self._open_handles.clear()
            # Switch sensors back to lightweight preview/streaming mode
            for s in self.sensors:
                if hasattr(s, "set_recording_mode"):
                    s.set_recording_mode(False)
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

            # Save timestamps.json into each modality folder
            if hasattr(self, "frame_timestamps") and hasattr(self, "modality_paths"):
                for sid, t_list in self.frame_timestamps.items():
                    m_path = self.modality_paths.get(sid)
                    if m_path and os.path.isdir(m_path):
                        try:
                            with open(os.path.join(m_path, "timestamps.json"), "w") as tf:
                                json.dump(t_list, tf)
                        except Exception as te:
                            logger.error(f"Failed to write timestamps.json for {sid}: {te}")

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
        # Calculate target interval (10 FPS across all cameras/sensors)
        fps = max([getattr(s, "fps", 10) for s in self.sensors] + [10])
        interval = 1.0 / fps

        while not self.stop_event.is_set():
            t0 = time.time()
            for sensor in self.sensors:
                folder = self.modality_paths.get(sensor.sensor_id)
                if not folder:
                    continue

                is_radar = (sensor.modality == "radar" or getattr(sensor, "backend", "") == "radar")
                now_ts = round(time.time(), 4)
                cnt = self.frame_counts.get(sensor.sensor_id, 0) + 1
                self.frame_counts[sensor.sensor_id] = cnt
                if hasattr(self, "frame_timestamps") and sensor.sensor_id in self.frame_timestamps:
                    self.frame_timestamps[sensor.sensor_id].append(now_ts)

                if is_radar:
                    # For radar: write individual raw binary frame file
                    raw_ts, raw_pkt = sensor.get_latest_raw_packet()
                    if raw_pkt:
                        bin_filename = f"frame_{cnt:06d}.bin"
                        bin_path = os.path.join(folder, bin_filename)
                        try:
                            with open(bin_path, "wb") as bf:
                                bf.write(raw_pkt)
                        except Exception as be:
                            logger.error(f"Failed to write radar frame to {bin_path}: {be}")
                else:
                    # For cameras: write image frame
                    frame_bytes = sensor.get_jpeg_frame()
                    if frame_bytes:
                        filename = f"frame_{cnt:06d}.jpg"
                        file_path = os.path.join(folder, filename)
                        try:
                            with open(file_path, "wb") as f:
                                f.write(frame_bytes)
                        except Exception as e:
                            logger.error(f"Failed to write frame to {file_path}: {e}")

                # Save raw radar packet bitstream & full spectrum JSONL using persistent file handles
                if hasattr(sensor, "get_spectrum_data") and (sensor.modality == "radar" or getattr(sensor, "backend", "") == "radar"):
                    spec = sensor.get_spectrum_data()
                    if spec:
                        try:
                            spec_h = getattr(self, "_open_handles", {}).get(f"{sensor.sensor_id}_spec")
                            line = json.dumps(spec) + "\n"
                            if spec_h and not spec_h.closed:
                                spec_h.write(line)
                            else:
                                with open(os.path.join(folder, "radar_spectrum.jsonl"), "a", encoding="utf-8") as sf:
                                    sf.write(line)
                        except Exception as se:
                            logger.error(f"Failed to write radar spectrum: {se}")

                    raw_ts, raw_pkt = sensor.get_latest_raw_packet()
                    if raw_pkt:
                        try:
                            import struct
                            raw_h = getattr(self, "_open_handles", {}).get(f"{sensor.sensor_id}_raw")
                            payload = struct.pack("<dH", raw_ts, len(raw_pkt)) + raw_pkt
                            if raw_h and not raw_h.closed:
                                raw_h.write(payload)
                            else:
                                with open(os.path.join(folder, "radar_raw.bin"), "ab") as rf:
                                    rf.write(payload)
                        except Exception as re:
                            logger.error(f"Failed to write radar raw packet: {re}")

                # Periodic flush of persistent handles every 50 frames
                if cnt % 50 == 0:
                    for h in getattr(self, "_open_handles", {}).values():
                        try:
                            h.flush()
                        except Exception:
                            pass

            # Safety check: Ensure at least 100 MB free disk space remains
            if os.name != "nt":
                try:
                    vfs = os.statvfs(self.storage_root)
                    free_mb = (vfs.f_bavail * vfs.f_frsize) / (1024 * 1024)
                    if free_mb < 100.0:
                        logger.warning(f"[storage] Disk space low: only {free_mb:.1f} MB free (minimum safety margin: 100 MB). Auto-stopping collection...")
                        threading.Thread(target=self.stop_collection, daemon=True).start()
                        break
                except Exception as de:
                    logger.debug(f"Error checking disk free space: {de}")

            if getattr(self, "max_duration_seconds", None) and (time.time() - self.session_start_time) >= self.max_duration_seconds:
                logger.info(f"Session {self.session_id} reached duration limit of {self.max_duration_seconds}s. Auto-stopping...")
                threading.Thread(target=self.stop_collection, daemon=True).start()
                break

            elapsed = time.time() - t0
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def clear_all_sessions(self):
        """Stops any active recording and deletes all recorded sessions in storage_root."""
        import shutil
        if self.is_recording:
            try:
                self.stop_collection()
            except Exception:
                pass

        removed_sessions = 0
        with self.lock:
            if os.path.exists(self.storage_root):
                for entry in os.listdir(self.storage_root):
                    entry_path = os.path.join(self.storage_root, entry)
                    try:
                        if os.path.isdir(entry_path):
                            removed_sessions += 1
                            # Fast OS-level deletion without freezing Python thread on stat calls
                            if os.name != "nt":
                                subprocess.run(f"rm -rf '{entry_path}'", shell=True)
                            else:
                                shutil.rmtree(entry_path, ignore_errors=True)
                        elif os.path.isfile(entry_path):
                            os.remove(entry_path)
                    except Exception as ex:
                        logger.warning(f"Error deleting {entry_path}: {ex}")
            self.session_id = None
            self.session_dir = None
            self.frame_counts = {}

        logger.info(f"Cleared SD recordings in {self.storage_root}: {removed_sessions} sessions")
        return {
            "status": "cleared",
            "sessions_removed": removed_sessions,
            "collection": self.get_status()
        }

    def get_status(self):
        # Calculate disk storage stats
        try:
            target = self.storage_root if os.path.exists(self.storage_root) else "/"
            stat = os.statvfs(target)
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
            "duration_seconds": round(time.time() - self.session_start_time, 1) if (self.is_recording and self.session_start_time) else 0,
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
            dev_id = parts[0] if len(parts) >= 2 else None
            target = parts[1] if len(parts) >= 2 else (parts[0] if parts else "rgb")
            my_id = self.daemon_ref.config.get("device_id")
            if dev_id and dev_id != my_id and dev_id in self.daemon_ref.peer_ips:
                self._proxy_peer_stream(f"/proxy/stream/{dev_id}/{target}", dev_id)
            else:
                self._handle_mjpeg_stream(target)
        elif path.startswith("/proxy/snapshot/"):
            parts = [p for p in path.split("/proxy/snapshot/")[1].split("/") if p]
            dev_id = parts[0] if len(parts) >= 2 else None
            target = parts[1] if len(parts) >= 2 else (parts[0] if parts else "rgb")
            my_id = self.daemon_ref.config.get("device_id")
            if dev_id and dev_id != my_id and dev_id in self.daemon_ref.peer_ips:
                self._proxy_peer_request(f"/proxy/snapshot/{dev_id}/{target}", dev_id)
            else:
                self._handle_snapshot(target)
        elif path.startswith("/api/stream/"):
            # Stream path: /api/stream/<modality> or /api/stream/<sensor_id>
            target = path.split("/api/stream/")[1]
            self._handle_mjpeg_stream(target)
        elif path.startswith("/api/snapshot/"):
            target = path.split("/api/snapshot/")[1]
            self._handle_snapshot(target)
        elif path == "/api/radar/spectrum":
            for s in self.daemon_ref.sensors:
                if hasattr(s, "get_spectrum_data"):
                    spec = s.get_spectrum_data()
                    if spec is not None:
                        self._send_json(spec)
                        return
            self._send_json({"error": "No radar sensor active"}, status_code=404)
        elif path == "/api/collection/start":
            qs = parse_qs(parsed.query)
            max_dur = qs.get("duration", [None])[0] or qs.get("max_duration_seconds", [None])[0]
            label = qs.get("label", [None])[0]
            res = self.daemon_ref.collector.start_collection(session_label=label, max_duration_seconds=max_dur)
            self._send_json(res)
            if "peer=1" not in parsed.query:
                threading.Thread(
                    target=self.daemon_ref.broadcast_to_peers,
                    args=(f"/api/collection/start?peer=1{'&duration=' + str(max_dur) if max_dur else ''}", b"{}"),
                    kwargs={"is_recording_state": True},
                    daemon=True
                ).start()
        elif path == "/api/collection/stop":
            res = self.daemon_ref.collector.stop_collection()
            self._send_json(res)
            if "peer=1" not in parsed.query:
                threading.Thread(
                    target=self.daemon_ref.broadcast_to_peers,
                    args=("/api/collection/stop?peer=1", b"{}"),
                    kwargs={"is_recording_state": False},
                    daemon=True
                ).start()
        elif path == "/api/camera/exposure":
            for s in self.daemon_ref.sensors:
                if hasattr(s, "query_exposure_controls"):
                    info = s.query_exposure_controls()
                    if info:
                        self._send_json({"device_id": self.daemon_ref.config.get("device_id"), "controls": info})
                        return
            self._send_json({"error": "No V4L2 camera found"}, status_code=404)
        elif path == "/api/sessions" or path == "/api/recordings/sessions":
            self._handle_list_sessions()
        elif path.startswith("/api/session/"):
            self._handle_session_detail(path)
        else:
            self.send_error(404, "Not Found")

    def do_HEAD(self):
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Private-Network", "true")
        self.end_headers()

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS, DELETE")
        self.send_header("Access-Control-Allow-Headers", "Content-Type, Authorization, Cache-Control, Pragma, X-Requested-With, Origin, Accept")
        self.send_header("Access-Control-Allow-Private-Network", "true")
        self.end_headers()

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/api/collection/start":
            content_length = int(self.headers.get('Content-Length', 0))
            label = None
            max_dur = None
            body = b""
            if content_length > 0:
                body = self.rfile.read(content_length)
                try:
                    payload = json.loads(body.decode('utf-8'))
                    label = payload.get("label")
                    max_dur = payload.get("duration") or payload.get("max_duration_seconds")
                except Exception:
                    pass
            res = self.daemon_ref.collector.start_collection(session_label=label, max_duration_seconds=max_dur)
            self._send_json(res)
            if "peer=1" not in parsed.query:
                threading.Thread(
                    target=self.daemon_ref.broadcast_to_peers,
                    args=("/api/collection/start?peer=1", body or b"{}"),
                    kwargs={"is_recording_state": True},
                    daemon=True
                ).start()
        elif path == "/api/collection/stop":
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length > 0:
                self.rfile.read(content_length)
            res = self.daemon_ref.collector.stop_collection()
            self._send_json(res)
            if "peer=1" not in parsed.query:
                threading.Thread(
                    target=self.daemon_ref.broadcast_to_peers,
                    args=("/api/collection/stop?peer=1", b"{}"),
                    kwargs={"is_recording_state": False},
                    daemon=True
                ).start()
        elif path == "/api/display/toggle":
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length > 0:
                self.rfile.read(content_length)
            self._send_json({"display_enabled": True})
        elif path == "/api/hotspot":
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length) if content_length > 0 else b"{}"
            try:
                payload = json.loads(body.decode('utf-8'))
            except Exception:
                payload = {}
            enabled = payload.get("enabled", True)
            res = self.daemon_ref.set_hotspot_state(enabled)
            self._send_json(res)
        elif path.startswith("/api/edges/") and path.endswith("/hotspot"):
            parts = [p for p in path.split("/") if p]
            target_dev = parts[2] if len(parts) >= 4 else None
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length) if content_length > 0 else b"{}"
            try:
                payload = json.loads(body.decode('utf-8'))
            except Exception:
                payload = {}
            enabled = payload.get("enabled", True)

            my_id = self.daemon_ref.config.get("device_id")
            if target_dev == my_id or target_dev == "edge-pi-zero-01" or not target_dev:
                res = self.daemon_ref.set_hotspot_state(enabled)
                self._send_json(res)
            else:
                peer_ip = self.daemon_ref.peer_ips.get(target_dev)
                if not peer_ip:
                    fallback_map = {"edge-pi-zero-02": "192.168.4.39", "edge-pi-zero-03": "192.168.4.30"}
                    peer_ip = fallback_map.get(target_dev)

                if peer_ip:
                    try:
                        req = urllib.request.Request(
                            f"http://{peer_ip}:5001/api/hotspot",
                            data=body,
                            headers={"Content-Type": "application/json"},
                            method="POST"
                        )
                        with urllib.request.urlopen(req, timeout=5) as r:
                            self._send_json(json.loads(r.read().decode()))
                            return
                    except Exception as e:
                        self._send_json({"status": "error", "message": f"Peer error: {e}"}, status_code=502)
                        return
                self._send_json({"status": "error", "message": f"Device {target_dev} not currently reachable"}, status_code=404)
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
        elif path == "/api/camera/exposure":
            content_length = int(self.headers.get('Content-Length', 0))
            payload = {}
            if content_length > 0:
                body = self.rfile.read(content_length)
                try:
                    payload = json.loads(body.decode('utf-8'))
                except Exception:
                    pass
            qs = parse_qs(parsed.query)
            auto_exp = payload.get("auto_exposure", qs.get("auto_exposure", [None])[0])
            bias = payload.get("auto_exposure_bias", qs.get("auto_exposure_bias", [None])[0])
            exp = payload.get("exposure", qs.get("exposure", [None])[0])
            applied = False
            for s in self.daemon_ref.sensors:
                if hasattr(s, "apply_exposure_controls"):
                    if s.apply_exposure_controls(auto_exposure=auto_exp, auto_exposure_bias=bias, exposure=exp):
                        applied = True
            current = {}
            for s in self.daemon_ref.sensors:
                if hasattr(s, "query_exposure_controls"):
                    current = s.query_exposure_controls()
                    if current:
                        break
            self._send_json({"success": applied, "controls": current})
        elif path in ["/api/sessions/clear", "/api/recordings/clear", "/api/collection/clear"]:
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length > 0:
                self.rfile.read(content_length)
            res = self.daemon_ref.collector.clear_all_sessions()
            self._send_json(res)
        else:
            self.send_error(404, "Not Found")

    def do_DELETE(self):
        parsed = urlparse(self.path)
        path = parsed.path
        if path in ["/api/sessions", "/api/recordings", "/api/sessions/clear"]:
            res = self.daemon_ref.collector.clear_all_sessions()
            self._send_json(res)
        else:
            self.send_error(404, "Not Found")

    def _proxy_peer_request(self, rel_path, dev_id):
        peer_ip = self.daemon_ref.peer_ips.get(dev_id)
        if not peer_ip:
            self.send_error(404, f"Peer {dev_id} not reachable")
            return
        url = f"http://{peer_ip}:5001{rel_path}"
        try:
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=5) as resp:
                content = resp.read()
                self.send_response(resp.status)
                for h in ["Content-Type", "Content-Length"]:
                    val = resp.headers.get(h)
                    if val:
                        self.send_header(h, val)
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Private-Network", "true")
                self.end_headers()
                self.wfile.write(content)
        except Exception as e:
            self.send_error(502, f"Peer Gateway Error: {e}")

    def _proxy_peer_stream(self, rel_path, dev_id):
        peer_ip = self.daemon_ref.peer_ips.get(dev_id)
        if not peer_ip:
            self.send_error(404, f"Peer {dev_id} not reachable")
            return
        url = f"http://{peer_ip}:5001{rel_path}"
        try:
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=10) as resp:
                self.send_response(200)
                self.send_header("Content-Type", resp.headers.get("Content-Type", "multipart/x-mixed-replace; boundary=frame"))
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Private-Network", "true")
                self.send_header("Cache-Control", "no-cache, private")
                self.end_headers()
                while True:
                    chunk = resp.read(4096)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as e:
            self.send_error(502, f"Peer Stream Error: {e}")

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
                "resolution": f"{s.stream_width}x{s.stream_height} (Stream) / {s.record_width}x{s.record_height} (Save)" if getattr(s, "record_width", s.width) != getattr(s, "stream_width", s.width) else f"{s.width}x{s.height}",
                "fps": s.fps,
                "backend": getattr(s, "backend", "none")
            }
            for s in self.daemon_ref.sensors
        ]
        local_edge = {
            "device_id": self.daemon_ref.config.get("device_id", "edge-pi-zero-01"),
            "device_name": self.daemon_ref.config.get("device_name", "Raspberry Pi Zero (STEVAL-55G0MBI)"),
            "host": self.daemon_ref.config.get("host", "0.0.0.0"),
            "port": int(self.daemon_ref.config.get("port", 5001)),
            "online": True,
            "is_gateway": True,
            "ping_ms": 0.0,
            "link_status": "gateway",
            "last_seen": time.time(),
            "network": self.daemon_ref.get_network_info(),
            "hotspot_enabled": self.daemon_ref.get_hotspot_status(),
            "sensors": sensor_info,
            "modalities": list(set(s.modality for s in self.daemon_ref.sensors)),
            "collection": self.daemon_ref.collector.get_status()
        }
        all_edges = [local_edge]
        now = time.time()
        for p_id, p_edge in list(self.daemon_ref.peer_edges.items()):
            if p_id != local_edge["device_id"]:
                edge_copy = dict(p_edge)
                time_diff = now - p_edge.get("last_seen", 0)
                edge_copy["last_seen_ago_s"] = int(time_diff)
                # If seen within last 45s, mark online
                if time_diff < 90.0:
                    edge_copy["online"] = True
                    p_ms = edge_copy.get("ping_ms", 20.0)
                    edge_copy["link_status"] = "excellent" if p_ms < 45 else ("good" if p_ms < 100 else "slow")
                else:
                    edge_copy["online"] = False
                    edge_copy["link_status"] = "offline"
                
                # Keep in edge list for up to 10 minutes so feeds don't vanish suddenly
                if time_diff < 600.0:
                    all_edges.append(edge_copy)

        edges_data = {
            "display_enabled": True,
            "gateway_device_id": local_edge["device_id"],
            "timestamp": now,
            "edges": all_edges
        }
        self._send_json(edges_data)

    def _handle_status(self):
        collector_status = self.daemon_ref.collector.get_status()
        sensor_info = [
            {
                "id": s.sensor_id,
                "modality": s.modality,
                "name": s.name,
                "resolution": f"{s.stream_width}x{s.stream_height} (Stream) / {s.record_width}x{s.record_height} (Save)" if getattr(s, "record_width", s.width) != getattr(s, "stream_width", s.width) else f"{s.width}x{s.height}",
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

        if hasattr(sensor, "get_stream_jpeg_frame"):
            try:
                frame = sensor.get_stream_jpeg_frame(force=True)
            except TypeError:
                frame = sensor.get_stream_jpeg_frame()
        elif hasattr(sensor, "get_jpeg_frame"):
            try:
                frame = sensor.get_jpeg_frame(force=True)
            except TypeError:
                frame = sensor.get_jpeg_frame()
        else:
            frame = b""

        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
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

        target_delay = 1.0 / max(sensor.fps, 5) if sensor.modality != "radar" else 0.2
        try:
            while True:
                frame_bytes = sensor.get_stream_jpeg_frame() if hasattr(sensor, "get_stream_jpeg_frame") else sensor.get_jpeg_frame()
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

    def _handle_session_detail(self, path):
        """
        Handles:
          /api/session/{date}/{time}/{modality}/frames
          /api/session/{date}/{time}/{modality}/frame/{frame_id_or_index}
        """
        parts = [p for p in path.split("/api/session/")[1].split("/") if p]
        if len(parts) < 4:
            self.send_error(400, "Invalid session request path format")
            return

        date_entry = parts[0]
        time_entry = parts[1]
        modality = parts[2]
        action = parts[3]

        storage_root = self.daemon_ref.collector.storage_root
        session_base = os.path.join(storage_root, date_entry, time_entry)
        target_dir = os.path.join(session_base, modality)

        if not os.path.isdir(target_dir):
            # Try sensor modality or sensor id lookup
            sensor = self.daemon_ref.find_sensor(modality)
            if sensor:
                cand1 = os.path.join(session_base, sensor.modality)
                cand2 = os.path.join(session_base, sensor.sensor_id)
                if os.path.isdir(cand1):
                    target_dir = cand1
                elif os.path.isdir(cand2):
                    target_dir = cand2

        if not os.path.isdir(target_dir) and os.path.isdir(session_base):
            # Scan actual directories under session_base for match
            for d in os.listdir(session_base):
                d_path = os.path.join(session_base, d)
                if os.path.isdir(d_path):
                    if d.lower() in modality.lower() or modality.lower() in d.lower():
                        target_dir = d_path
                        break

        if not os.path.isdir(target_dir):
            self.send_error(404, f"Session modality directory not found: {modality}")
            return

        files = sorted([f for f in os.listdir(target_dir) if f.lower().endswith(".jpg") or f.lower().endswith(".jpeg")])

        if action == "frames":
            manifest_path = os.path.join(storage_root, date_entry, time_entry, "session_manifest.json")
            manifest = {}
            if os.path.exists(manifest_path):
                try:
                    with open(manifest_path, "r") as mf:
                        manifest = json.load(mf)
                except Exception:
                    pass

            # Fast timestamps lookup: from timestamps.json or scandir mtimes
            ts_cache_path = os.path.join(target_dir, "timestamps.json")
            timestamps = []
            if os.path.exists(ts_cache_path):
                try:
                    with open(ts_cache_path, "r") as tf:
                        timestamps = json.load(tf)
                except Exception:
                    pass

            if not timestamps and files:
                try:
                    entries = sorted([e for e in os.scandir(target_dir) if e.name.lower().endswith(".jpg") or e.name.lower().endswith(".jpeg")], key=lambda e: e.name)
                    timestamps = [round(e.stat().st_mtime, 4) for e in entries]
                    try:
                        with open(ts_cache_path, "w") as tf:
                            json.dump(timestamps, tf)
                    except Exception:
                        pass
                except Exception:
                    pass

            start_ts = timestamps[0] if timestamps else 0.0
            end_ts = timestamps[-1] if timestamps else 0.0

            self._send_json({
                "date": date_entry,
                "time": time_entry,
                "modality": modality,
                "total_frames": len(files),
                "frames": files,
                "timestamps": timestamps,
                "start_ts": start_ts,
                "end_ts": end_ts,
                "duration": round(end_ts - start_ts, 3) if timestamps else 0.0,
                "manifest": manifest
            })

        elif action == "frame":
            if len(parts) < 5:
                self.send_error(400, "Missing frame identifier")
                return
            if not files:
                self.send_error(404, "No frames recorded in modality")
                return

            frame_id = parts[4]
            # frame_id can be 'frame_000042.jpg' or integer '42'
            if frame_id.isdigit():
                idx = int(frame_id)
                # 1-based index clamped to available frame range
                idx_clamped = max(1, min(idx, len(files)))
                target_file = os.path.join(target_dir, files[idx_clamped - 1])
            else:
                target_file = os.path.join(target_dir, os.path.basename(frame_id))
                if not os.path.isfile(target_file) and files:
                    target_file = os.path.join(target_dir, files[-1])

            if not os.path.isfile(target_file):
                self.send_error(404, "Frame image file not found")
                return

            try:
                with open(target_file, "rb") as f:
                    content = f.read()
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(content)))
                self.send_header("Cache-Control", "public, max-age=86400")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Private-Network", "true")
                self.end_headers()
                self.wfile.write(content)
            except Exception as ex:
                self.send_error(500, f"Error reading frame: {ex}")
        elif action in ["spectrum", "radar_spectrum"]:
            spec_file = os.path.join(target_dir, "radar_spectrum.jsonl")
            if not os.path.isfile(spec_file):
                self.send_error(404, "No radar spectrum recorded for this session")
                return
            try:
                with open(spec_file, "rb") as f:
                    content = f.read()
                self.send_response(200)
                self.send_header("Content-Type", "application/x-ndjson")
                self.send_header("Content-Length", str(len(content)))
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Private-Network", "true")
                self.end_headers()
                self.wfile.write(content)
            except Exception as ex:
                self.send_error(500, f"Error reading radar spectrum: {ex}")
        elif action in ["raw", "radar_raw"]:
            raw_file = os.path.join(target_dir, "radar_raw.bin")
            if not os.path.isfile(raw_file):
                self.send_error(404, "No raw radar stream recorded for this session")
                return
            try:
                with open(raw_file, "rb") as f:
                    content = f.read()
                self.send_response(200)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Content-Disposition", 'attachment; filename="radar_raw.bin"')
                self.send_header("Content-Length", str(len(content)))
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Private-Network", "true")
                self.end_headers()
                self.wfile.write(content)
            except Exception as ex:
                self.send_error(500, f"Error reading raw radar stream: {ex}")
        else:
            self.send_error(404, f"Unknown action: {action}")


class EdgeDaemon:
    def get_hotspot_status(self):
        if os.path.exists("/home/pi/.hotspot_disabled"):
            return False
        try:
            p = subprocess.run(["pgrep", "-x", "hostapd"], stdout=subprocess.PIPE, text=True)
            return bool(p.stdout.strip())
        except Exception:
            return True

    def set_hotspot_state(self, enabled: bool):
        action = "start" if enabled else "stop"
        try:
            if os.path.exists("/home/pi/toggle_fleet_hotspot.sh"):
                subprocess.run(["sudo", "/home/pi/toggle_fleet_hotspot.sh", action], check=False)
            elif enabled:
                subprocess.run(["sudo", "systemctl", "start", "hostapd"], check=False)
            else:
                subprocess.run(["sudo", "pkill", "-9", "hostapd"], check=False)
                subprocess.run(["sudo", "pkill", "-9", "dnsmasq"], check=False)

            # If UZ801 dongle is present, also update modem WiFi
            wifi_stat = "1" if enabled else "0"
            try:
                urllib.request.urlopen(
                    urllib.request.Request(
                        "http://192.168.100.1/ajax",
                        data=b'{"funcNo":1000,"username":"admin","password":"admin"}',
                        headers={"Content-Type": "application/json"},
                        method="POST"
                    ),
                    timeout=2
                )
                urllib.request.urlopen(
                    urllib.request.Request(
                        "http://192.168.100.1/ajax",
                        data=json.dumps({"funcNo": 1007, "wifi_status": wifi_stat}).encode(),
                        headers={"Content-Type": "application/json"},
                        method="POST"
                    ),
                    timeout=3
                )
            except Exception:
                pass
        except Exception as ex:
            logger.warning(f"Error setting hotspot state: {ex}")

        return {"status": "ok", "hotspot_enabled": self.get_hotspot_status()}

    def __init__(self, config=None):
        self.config = config or load_config()
        self.sensors = [SensorDriver(sc) for sc in self.config.get("sensors", [])]
        self.collector = CollectionManager(
            storage_root=self.config.get("storage_root", "/home/pi/edge_data"),
            sensors=self.sensors
        )
        self.running = True
        self.peer_edges = {}
        self.peer_ips = {}
        self._peer_thread = threading.Thread(target=self._peer_scanner_loop, daemon=True)
        self._peer_thread.start()

    def broadcast_to_peers(self, endpoint, payload_bytes, is_recording_state=None):
        # Immediately update in-memory peer_edges cache so UI polls see the new state right away
        if is_recording_state is not None:
            for dev_id, p_edge in list(self.peer_edges.items()):
                if "collection" in p_edge and isinstance(p_edge["collection"], dict):
                    p_edge["collection"]["is_recording"] = is_recording_state
                    if not is_recording_state:
                        p_edge["collection"]["duration_seconds"] = 0

        def _send_one(item):
            dev_id, ip = item
            try:
                url = f"http://{ip}:5001{endpoint}"
                req = urllib.request.Request(url, data=payload_bytes, headers={"Content-Type": "application/json"}, method="POST")
                urllib.request.urlopen(req, timeout=5.0).read()
            except Exception as e:
                logger.warning(f"Failed to broadcast {endpoint} to {dev_id} ({ip}): {e}")

        items = list(self.peer_ips.items())
        if items:
            with ThreadPoolExecutor(max_workers=len(items)) as pool:
                list(pool.map(_send_one, items))

    def _peer_scanner_loop(self):
        my_id = self.config.get("device_id")

        def get_candidates():
            candidates = set([
                "192.168.137.201", "192.168.137.203", "192.168.137.204", "192.168.137.35",
                "192.168.4.1", "192.168.4.201", "192.168.4.203", "192.168.4.204", "192.168.4.35",
                "192.168.100.122", "192.168.100.201", "192.168.100.203", "192.168.100.204", "192.168.100.35",
                "10.42.0.1", "10.42.0.201", "10.42.0.203", "10.42.0.204", "10.42.0.35"
            ])
            # 1. Check dnsmasq leases if hotspot master
            for lease_file in ["/var/lib/misc/dnsmasq.leases", "/tmp/dnsmasq.leases", "/var/lib/dnsmasq/dnsmasq.leases"]:
                if os.path.exists(lease_file):
                    try:
                        with open(lease_file, "r") as f:
                            for line in f:
                                parts = line.split()
                                if len(parts) >= 3 and (parts[2].startswith("192.168.4.") or parts[2].startswith("192.168.100.")):
                                    candidates.add(parts[2])
                    except Exception:
                        pass
            # 1b. Query Qualcomm 4G-UFI-C12 modem DHCP client table if reachable
            try:
                req = urllib.request.Request(
                    "http://192.168.100.1/ajax",
                    data=b'{"funcNo":1011}',
                    headers={"Content-Type": "application/json"},
                    method="POST"
                )
                with urllib.request.urlopen(req, timeout=0.8) as resp:
                    uz_data = json.loads(resp.read().decode())
                    for r in uz_data.get("results", []):
                        for dev in r.get("device_arr", []):
                            ip = dev.get("ip")
                            if ip:
                                candidates.add(ip)
            except Exception:
                pass
            # 2. Check dnsmasq leases and ARP table
            for lease_path in ["/var/lib/misc/dnsmasq.leases", "/tmp/dnsmasq.leases"]:
                if os.path.exists(lease_path):
                    try:
                        with open(lease_path, "r") as f:
                            for line in f:
                                parts = line.split()
                                if len(parts) >= 3 and parts[2].startswith("192.168."):
                                    candidates.add(parts[2])
                    except Exception:
                        pass
            if os.path.exists("/proc/net/arp"):
                try:
                    with open("/proc/net/arp", "r") as f:
                        for line in f:
                            parts = line.split()
                            if len(parts) >= 4 and parts[0] != "IP" and parts[3] != "00:00:00:00:00:00":
                                ip = parts[0]
                                if ip.startswith("192.168.137.") or ip.startswith("192.168.4.") or ip.startswith("192.168.100.") or ip.startswith("10.42.0."):
                                    candidates.add(ip)
                except Exception:
                    pass
            # 3. Add known fleet node IPs and dynamic pools
            for ip in ["192.168.4.39", "192.168.4.201", "192.168.4.202", "192.168.4.203", "192.168.4.204", "192.168.137.35", "192.168.137.142", "192.168.137.201", "192.168.137.203"]:
                candidates.add(ip)
            for i in range(10, 60):
                candidates.add(f"192.168.4.{i}")
            for i in range(100, 130):
                candidates.add(f"192.168.100.{i}")
            return list(candidates)

        def check_peer(ip):
            t0 = time.time()
            try:
                url = f"http://{ip}:5001/api/status"
                req = urllib.request.Request(url, headers={"User-Agent": "FleetScanner"})
                with urllib.request.urlopen(req, timeout=5.0) as resp:
                    if resp.status == 200:
                        elapsed_ms = round((time.time() - t0) * 1000, 1)
                        data = json.loads(resp.read().decode())
                        dev_id = data.get("device_id")
                        if dev_id and dev_id != my_id:
                            return (dev_id, ip, {
                                "device_id": dev_id,
                                "device_name": data.get("device_name", dev_id),
                                "host": ip,
                                "port": 5001,
                                "online": True,
                                "ping_ms": elapsed_ms,
                                "link_status": "excellent" if elapsed_ms < 45 else ("good" if elapsed_ms < 100 else "slow"),
                                "last_seen": time.time(),
                                "network": data.get("network", {}),
                                "sensors": data.get("sensors", []),
                                "modalities": data.get("modalities", []),
                                "collection": data.get("collection", {})
                            })
            except Exception:
                pass
            return None

        while self.running:
            try:
                candidate_ips = get_candidates()
                with ThreadPoolExecutor(max_workers=min(len(candidate_ips), 12)) as pool:
                    for res in pool.map(check_peer, candidate_ips):
                        if res:
                            dev_id, ip, edge_data = res
                            self.peer_ips[dev_id] = ip
                            self.peer_edges[dev_id] = edge_data
            except Exception as e:
                logger.debug(f"Peer scan error: {e}")
            time.sleep(10.0)

    def find_sensor(self, target):
        # Match by id or modality
        for s in self.sensors:
            if s.sensor_id == target or s.modality.lower() == target.lower():
                return s
        return None

    def get_network_info(self):
        now = time.monotonic()
        if hasattr(self, "_cached_net_info") and (now - getattr(self, "_cached_net_time", 0)) < 60.0:
            return self._cached_net_info

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
        info = {"type": net_type, "interface": iface, "ip": ip_addr, "ssid": ssid}
        self._cached_net_info = info
        self._cached_net_time = now
        return info

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
