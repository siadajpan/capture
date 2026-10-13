# Deployment & Operations Guide &bull; capture.me.uk

Complete guide for building, uploading, and running the Multi-Sensor Edge Capture Platform across **Cloudflare**, the **Local Hub Server**, and **Edge Devices (Raspberry Pi Zero v1.1 + STEVAL-55G0MBI)**.

---

## 📑 Table of Contents

1. [Architecture Overview](#1-architecture-overview)
2. [Astro Web App: Build & Upload to Cloudflare](#2-astro-web-app-build--upload-to-cloudflare)
   - [Method A: Deploy with Wrangler CLI (Fastest)](#method-a-deploy-with-wrangler-cli-fastest)
   - [Method B: Deploy via GitHub (Continuous Deployment)](#method-b-deploy-via-github-continuous-deployment)
3. [Local Hub Server: Setup & Execution](#3-local-hub-server-setup--execution)
4. [Raspberry Pi Zero v1.1: Hardware & Daemon Setup](#4-raspberry-pi-zero-v11-hardware--daemon-setup)
5. [Connecting the Cloudflare Dashboard to Edge Devices](#5-connecting-the-cloudflare-dashboard-to-edge-devices)
6. [Data Storage & Directory Structure](#6-data-storage--directory-structure)

---

## 1. Architecture Overview

```
                      ┌──────────────────────────────────────────┐
                      │          Cloudflare Edge (Global)         │
                      │           https://capture.me.uk          │
                      │        (Astro Static Assets Worker)      │
                      └─────────────────────┬────────────────────┘
                                            │
                                  Browser on Phone/PC
                                            │
                     ┌──────────────────────┴──────────────────────┐
                     │                                             │
            (Local Wi-Fi / Hotspot)                       (Cloudflare Tunnel /
                     │                                         Tailscale)
                     ▼                                             ▼
        ┌─────────────────────────┐                   ┌─────────────────────────┐
        │  Local Hub Server       │                   │  Direct Edge Connection │
        │  (FastAPI on Port 8000) │                   │  (Port 5001 per device) │
        └────────────┬────────────┘                   └─────────────────────────┘
                     │
          ┌──────────┴──────────────────────────┐
          │                                     │
          ▼                                     ▼
┌───────────────────────────┐         ┌───────────────────────────┐
│ Edge 1: Pi Zero v1.1      │         │ Edge 2: Multi-Sensor Rig  │
│ - STEVAL-55G0MBI (RGB)    │         │ - RGB Camera              │
│ - Port 5001 (daemon)      │         │ - Thermal (FLIR Lepton)   │
│ - Internal SD: YYYY-MM-DD │         │ - Depth (RealSense / ToF) │
└───────────────────────────┘         └───────────────────────────┘
```

---

## 2. Astro Web App: Build & Upload to Cloudflare

The web application is located in the `capture/` folder.

### Prerequisites:
- Node.js `v20+` or `v24+`
- npm `v10+`

### Build the Application:
```bash
cd /home/kmisiarz/research/edge_collection/capture
npm run build
```
This builds the optimized production static site into `./dist` in under a second.

### Method A: Deploy with Wrangler CLI (Fastest)

To push updates directly to your live Cloudflare Worker:
```bash
cd /home/kmisiarz/research/edge_collection/capture
npm run deploy
```
*(This automatically runs `astro build` and then `npx wrangler deploy`).*

The `wrangler.jsonc` file controls this deployment:
```jsonc
{
  "name": "capture",
  "compatibility_date": "2026-10-13",
  "assets": {
    "directory": "./dist"
  }
}
```

### Method B: Deploy via GitHub (Continuous Deployment)

1. **Commit and push to GitHub**:
   ```bash
   cd /home/kmisiarz/research/edge_collection/capture
   git add .
   git commit -m "feat: update dashboard"
   git push origin main
   ```
2. **If linked to Cloudflare Pages**:
   Cloudflare automatically detects commits on `main`, runs `npm run build`, and publishes changes to `capture.me.uk` automatically.

---

## 3. Local Hub Server: Setup & Execution

The Hub Server aggregates video feeds from edge devices, proxies them to prevent CORS/firewall issues, and broadcasts synchronized collection commands.

### Location: `capture/server/`

### 1. Install Dependencies
```bash
cd /home/kmisiarz/research/edge_collection/capture
python3 -m venv venv
./venv/bin/pip install fastapi uvicorn requests pillow aiohttp
```

### 2. Start the Server
```bash
./venv/bin/python3 server/app.py
```
- **Local Address**: `http://localhost:8000`
- **LAN Address**: `http://<YOUR_LOCAL_IP>:8000` (e.g. `http://192.168.41.78:8000`)

### 3. Edge Registry (`server/edges_registry.json`)
The Hub automatically maintains a list of known edge devices:
```json
{
  "edges": {
    "edge-pi-zero-01": {
      "device_id": "edge-pi-zero-01",
      "device_name": "Raspberry Pi Zero v1.1 (STEVAL-55G0MBI)",
      "host": "192.168.1.150",
      "port": 5001
    }
  }
}
```
You can also add new edge devices dynamically from the web dashboard UI without touching config files.

---

## 4. Raspberry Pi Zero v1.1: Hardware & Daemon Setup

### Hardware Checklist:
- **Board**: Raspberry Pi Zero v1.1 (ARMv6, single-core 1.0 GHz, 512 MB RAM).
- **Camera**: STMicroelectronics **STEVAL-55G0MBI (PCB4189D)** with VD55G0 global shutter image sensor.
- **Cable**: 15-pin 0.5 mm pitch FFC ribbon cable.

### Step 1: Connect Camera
1. Pull up the plastic retaining clip on the Pi Zero CSI camera connector.
2. Insert the ribbon cable with the **silver contact pins facing the HDMI/USB ports** (and the blue stiffener facing the back).
3. Connect the opposite end to the STEVAL-55G0MBI module.
4. Press the retaining clip down firmly.

### Step 2: Enable Camera in Raspberry Pi OS
Open `/boot/firmware/config.txt` (or `/boot/config.txt`):
```ini
camera_auto_detect=1
dtoverlay=vd55g0
```
Reboot:
```bash
sudo reboot
```

Verify camera is recognized:
```bash
v4l2-ctl --list-devices
# Expected: /dev/video0
```

### Step 3: Install & Start Daemon on Pi Zero
Copy `edge_agent/` to the Pi Zero (e.g., `/home/pi/edge_agent/`):
```bash
cd /home/pi/edge_agent
sudo bash setup_pi_zero.sh
```

This automated script:
1. Installs minimal packages (`python3-pil`, `v4l-utils`).
2. Creates `/home/pi/edge_data` storage folder on the SD card.
3. Installs and starts the systemd service: `edge-agent.service`.

### Useful Systemd Commands on the Pi:
```bash
# Check status
sudo systemctl status edge-agent.service

# View live logs
journalctl -u edge-agent.service -f

# Restart daemon
sudo systemctl restart edge-agent.service
```

---

## 5. Connecting the Cloudflare Dashboard to Edge Devices

When you visit **`https://capture.me.uk`** on your phone or computer:

1. Click the **Network Badge** in the top navigation bar (labeled **"Local Hub"**).
2. Enter your Hub or edge address:
   - **On the same Wi-Fi**: enter `http://<YOUR_COMPUTER_IP>:8000` (e.g. `http://192.168.41.78:8000`).
   - **Across different networks**: enter your **Cloudflare Tunnel** URL or **Tailscale** IP.
3. Click **Save**.
4. The web dashboard will save this address in your browser's `localStorage` and automatically poll and stream feeds from that point forward!

---

## 6. Data Storage & Directory Structure

When you click **COLLECT FEEDS** on the dashboard, the Hub sends a synchronized trigger to each edge node.

Each edge device records frames locally on its internal SD card under:
```text
/home/pi/edge_data/
└── <YYYY-MM-DD>/               # Today's date (e.g. 2026-10-13)
    └── <HH-MM-SS>/             # Recording start time (e.g. 11-15-30)
        ├── session_manifest.json
        └── <modality>/         # Modality subfolder
            ├── frame_000001.jpg
            ├── frame_000002.jpg
            └── ...
```

### Manifest File (`session_manifest.json`):
```json
{
  "session_id": "2026-10-13_11-15-30",
  "start_time_iso": "2026-10-13T11:15:30.123456",
  "end_time_iso": "2026-10-13T11:17:30.654321",
  "duration_seconds": 120.5,
  "label": "capture.me.uk Session",
  "sensors": [
    {
      "id": "rgb-steval",
      "modality": "rgb",
      "name": "STEVAL-55G0MBI (VD55G0 Global Shutter)",
      "resolution": "640x480",
      "fps": 15
    }
  ],
  "frames_recorded": {
    "rgb-steval": 1807
  }
}
```
