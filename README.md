# capture.me.uk &bull; Multi-Sensor Edge Collection Platform

Cloudflare-ready **Astro** application and Edge Capture system designed for mobile monitoring, synchronized recording, and embedded edge devices (starting with Raspberry Pi Zero v1.1 + STEVAL-55G0MBI camera).

Domain: [https://capture.me.uk](https://capture.me.uk)

---

## 🌟 Architecture Overview

```text
capture/
├── src/                        # Astro Web Application (Frontend)
│   ├── pages/index.astro       # Mobile-first Glassmorphic Dashboard
│   └── styles/global.css       # Design system & responsive layout
├── public/                     # Static assets (PWA manifest, favicon)
├── astro.config.mjs            # Astro configuration for capture.me.uk
├── package.json                # Dependencies & scripts
│
├── edge_agent/                 # Lightweight daemon for Raspberry Pi Zero
│   ├── edge_daemon.py          # Multimodal capture & local storage daemon
│   ├── edge_config.json        # Sensor config (STEVAL-55G0MBI)
│   └── setup_pi_zero.sh        # Hardware setup & systemd installer
│
├── server/                     # Local Hub & Stream Proxy Gateway
│   ├── app.py                  # FastAPI hub (proxies streams, broadcasts record)
│   └── edges_registry.json     # Dynamic registry for connected edge nodes
│
└── simulator/                  # Multi-node testing simulator
    └── simulate_edges.py       # Simulates Pi Zero + multimodal rigs
```

---

## ⚡ Cloudflare Pages Deployment

This repository is pre-configured for **Cloudflare Pages** connected to your domain `capture.me.uk`.

### Step 1: Push to GitHub
```bash
git add .
git commit -m "feat: migrate edge capture platform to Astro for capture.me.uk"
git push origin main
```

### Step 2: Configure in Cloudflare Dashboard
1. Go to [Cloudflare Dashboard](https://dash.cloudflare.com/) &rarr; **Compute (Workers & Pages)** &rarr; **Create** &rarr; **Pages** &rarr; **Connect to Git**.
2. Select your repository: `siadajpan/capture`.
3. Set the build settings:
   - **Framework preset**: `Astro`
   - **Build command**: `npm run build`
   - **Build output directory**: `dist`
4. Click **Save and Deploy**.

### Step 3: Connect Your Domain (`capture.me.uk`)
1. In your Cloudflare Pages project, click **Custom domains**.
2. Enter `capture.me.uk` (and `www.capture.me.uk` if desired).
3. Cloudflare will automatically configure the DNS records and SSL certificate.

---

## 📱 Mobile App Usage (capture.me.uk)

1. Open `https://capture.me.uk` on your mobile phone browser (Safari or Chrome).
2. Tap **Add to Home Screen** to install as a fullscreen native app.
3. In the top bar, tap **Local Hub** / network badge to set your Hub / Edge address (local IP, Tailscale IP, or Cloudflare Tunnel).
4. Tap **DISPLAY FEEDS** to toggle video streaming on/off.
5. Tap **COLLECT FEEDS** to trigger synchronized recording across all edge devices.

---

## 💾 Edge Storage Structure

Data is saved **internally on each edge device's SD card**:
```text
<storage_root>/
└── <YYYY-MM-DD>/           # Today's date (e.g. 2026-10-13)
    └── <HH-MM-SS>/         # Recording time (e.g. 10-19-14)
        ├── session_manifest.json
        └── <modality>/     # Modality folder (e.g. rgb, thermal, depth)
            ├── frame_000001.jpg
            └── ...
```

---

## 🍓 Raspberry Pi Zero v1.1 Setup (STEVAL-55G0MBI)

1. Connect the STEVAL-55G0MBI camera via 15-pin FFC cable to the Pi Zero camera port.
2. In `/boot/firmware/config.txt` (or `/boot/config.txt`), ensure camera is enabled:
   ```ini
   camera_auto_detect=1
   dtoverlay=vd55g0
   ```
3. Copy `edge_agent/` to the Pi and run:
   ```bash
   cd edge_agent
   sudo bash setup_pi_zero.sh
   ```
