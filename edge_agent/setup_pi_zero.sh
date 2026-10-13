#!/bin/bash
# ==============================================================================
# Setup Script for Raspberry Pi Zero v1.1 + STEVAL-55G0MBI (PCB4189D)
# ==============================================================================

set -e

echo "=========================================================="
echo " Setting up Edge Agent on Raspberry Pi Zero v1.1"
echo " Camera: STMicroelectronics STEVAL-55G0MBI (PCB4189D)"
echo "=========================================================="

# 1. Check Root
if [ "$EUID" -ne 0 ]; then
  echo "[!] Please run as root (sudo bash setup_pi_zero.sh)"
  exit 1
fi

TARGET_USER=${SUDO_USER:-pi}
TARGET_HOME=$(getent passwd "$TARGET_USER" | cut -d: -f6)
INSTALL_DIR="$TARGET_HOME/edge_collection"
STORAGE_DIR="$TARGET_HOME/edge_data"

echo "[*] User: $TARGET_USER"
echo "[*] Storage Root: $STORAGE_DIR"

# 2. Install lightweight dependencies for ARMv6 (avoid heavy builds)
echo "[*] Updating apt package cache and installing minimal requirements..."
apt-get update -y
apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-pil \
    v4l-utils \
    libcamera-tools \
    curl

# 3. Create storage directories on SD card
echo "[*] Preparing internal storage directories..."
mkdir -p "$STORAGE_DIR"
chown -R "$TARGET_USER:$TARGET_USER" "$STORAGE_DIR"

# 4. Check Camera hardware & V4L2 configuration
echo "[*] Checking Camera hardware interface..."
BOOT_CONFIG="/boot/firmware/config.txt"
if [ ! -f "$BOOT_CONFIG" ]; then
    BOOT_CONFIG="/boot/config.txt"
fi

if [ -f "$BOOT_CONFIG" ]; then
    echo "[*] Ensuring camera is enabled in $BOOT_CONFIG..."
    # Make sure camera_auto_detect is 1 or start_x=1
    if ! grep -q "^camera_auto_detect=1" "$BOOT_CONFIG" && ! grep -q "^start_x=1" "$BOOT_CONFIG"; then
        echo "camera_auto_detect=1" >> "$BOOT_CONFIG"
    fi
    # ST VD55G0 / STEVAL-55G0MBI overlay check
    if ! grep -q "dtoverlay=vd55g0" "$BOOT_CONFIG" && ! grep -q "dtoverlay=ov5647" "$BOOT_CONFIG"; then
        echo "# Custom sensor overlay can be specified here:" >> "$BOOT_CONFIG"
        echo "# dtoverlay=vd55g0" >> "$BOOT_CONFIG"
    fi
fi

# 5. Create Systemd Service for auto-start
SERVICE_PATH="/etc/systemd/system/edge-agent.service"
echo "[*] Creating systemd service at $SERVICE_PATH..."

cat <<EOF > "$SERVICE_PATH"
[Unit]
Description=Edge Camera Collection Daemon (Pi Zero)
After=network.target

[Service]
Type=simple
User=$TARGET_USER
WorkingDirectory=$INSTALL_DIR/edge_agent
ExecStart=/usr/bin/python3 $INSTALL_DIR/edge_agent/edge_daemon.py
Restart=always
RestartSec=3
Environment=PYTHONUNBUFFERED=1
Environment=EDGE_CONFIG=$INSTALL_DIR/edge_agent/edge_config.json

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
echo "[+] Setup completed!"
echo ""
echo "To start the edge agent immediately:"
echo "   sudo systemctl enable --now edge-agent.service"
echo ""
echo "To verify status:"
echo "   sudo systemctl status edge-agent.service"
echo "   v4l2-ctl --list-devices"
echo "=========================================================="
