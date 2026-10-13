#!/usr/bin/env python3
"""
Multi-Edge Device Simulator
Launches multiple simulated edge daemons on different ports to test:
1. Edge 1 (Pi Zero v1.1): STEVAL-55G0MBI (RGB)
2. Edge 2 (Multi-Modal Node): RGB Camera + Thermal Sensor + Depth Sensor
3. Edge 3 (Stereo Rig): Left RGB + Right RGB

All storing frames in the exact requested directory hierarchy:
<storage_root>/<YYYY-MM-DD>/<HH-MM-SS>/<modality>/
"""

import os
import sys
import time
import copy
import threading

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from edge_agent.edge_daemon import EdgeDaemon

BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

EDGE_CONFIGS = [
    {
        "device_id": "edge-pi-zero-01",
        "device_name": "Raspberry Pi Zero v1.1 (STEVAL-55G0MBI)",
        "host": "127.0.0.1",
        "port": 5001,
        "storage_root": os.path.join(BASE_DIR, "data_storage", "edge-pi-zero-01"),
        "sensors": [
            {
                "id": "rgb-steval",
                "modality": "rgb",
                "name": "STEVAL-55G0MBI (VD55G0 Global Shutter)",
                "type": "v4l2",
                "device": "/dev/video0",
                "width": 640,
                "height": 480,
                "fps": 15
            }
        ]
    },
    {
        "device_id": "edge-multimodal-02",
        "device_name": "Edge Rig B (RGB + Thermal + Depth)",
        "host": "127.0.0.1",
        "port": 5002,
        "storage_root": os.path.join(BASE_DIR, "data_storage", "edge-multimodal-02"),
        "sensors": [
            {
                "id": "rgb-main",
                "modality": "rgb",
                "name": "Sony IMX477 RGB",
                "type": "v4l2",
                "device": "/dev/video1",
                "width": 640,
                "height": 480,
                "fps": 15
            },
            {
                "id": "thermal-lwir",
                "modality": "thermal",
                "name": "FLIR Lepton 3.5 LWIR",
                "type": "spi",
                "device": "/dev/spidev0.0",
                "width": 640,
                "height": 480,
                "fps": 10
            },
            {
                "id": "depth-tof",
                "modality": "depth",
                "name": "RealSense / ToF Depth",
                "type": "depth",
                "device": "/dev/video2",
                "width": 640,
                "height": 480,
                "fps": 15
            }
        ]
    },
    {
        "device_id": "edge-stereo-03",
        "device_name": "Edge Rig C (Stereo RGB)",
        "host": "127.0.0.1",
        "port": 5003,
        "storage_root": os.path.join(BASE_DIR, "data_storage", "edge-stereo-03"),
        "sensors": [
            {
                "id": "rgb-stereo-left",
                "modality": "rgb",
                "name": "Stereo Left RGB",
                "type": "v4l2",
                "device": "/dev/video3",
                "width": 640,
                "height": 480,
                "fps": 15
            },
            {
                "id": "rgb-stereo-right",
                "modality": "rgb",
                "name": "Stereo Right RGB",
                "type": "v4l2",
                "device": "/dev/video4",
                "width": 640,
                "height": 480,
                "fps": 15
            }
        ]
    }
]


def run_daemon(cfg):
    daemon = EdgeDaemon(config=cfg)
    daemon.start_server()


def main():
    threads = []
    print("=" * 60)
    print("Launching Multi-Edge Device Simulator...")
    print("=" * 60)
    for cfg in EDGE_CONFIGS:
        t = threading.Thread(target=run_daemon, args=(cfg,), daemon=True)
        t.start()
        threads.append(t)
        print(f" [+] Launched {cfg['device_name']} on port {cfg['port']}")

    print("\nAll simulated edge devices running! Press Ctrl+C to exit.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping simulated edges...")


if __name__ == "__main__":
    main()
