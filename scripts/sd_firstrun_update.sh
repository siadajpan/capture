#!/bin/bash
BOOT="/boot/firmware"
[ ! -d "$BOOT" ] && BOOT="/boot"
LOG="$BOOT/firstrun.log"

echo "=== FIRSTRUN START $(date) ===" >> "$LOG"

# 1. Update hostname to pi-zero-03
echo "pi-zero-03" > /etc/hostname
hostname pi-zero-03 2>/dev/null || true
hostnamectl set-hostname pi-zero-03 2>/dev/null || true
if grep -q "127.0.1.1" /etc/hosts; then
    sed -i 's/127.0.1.1.*/127.0.1.1\tpi-zero-03/' /etc/hosts
else
    echo -e "127.0.1.1\tpi-zero-03" >> /etc/hosts
fi
echo "Hostname set to $(cat /etc/hostname)" >> "$LOG"

# 2. Fix and unblock SSH login completely
rm -f /etc/ssh/sshd_config.d/rename_user.conf
id -u pi >> "$LOG" 2>&1 || useradd -m -s /bin/bash pi >> "$LOG" 2>&1 || true
echo "pi:raspberry" | chpasswd >> "$LOG" 2>&1 || true
echo "root:raspberry" | chpasswd >> "$LOG" 2>&1 || true
usermod -aG sudo,video,input pi 2>/dev/null || true
echo "pi ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/010_pi-nopasswd
chmod 440 /etc/sudoers.d/010_pi-nopasswd

# Install Windows SSH public key
mkdir -p /home/pi/.ssh
echo "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAILZJwXVyeiYX8ks3PEsnmbxjWfG/jWAiaZpZdx0zGkR0 ugent\kmisiarz@UG-55LQ954" > /home/pi/.ssh/authorized_keys
chown -R 1000:1000 /home/pi/.ssh
chmod 700 /home/pi/.ssh
chmod 600 /home/pi/.ssh/authorized_keys
systemctl restart ssh >> "$LOG" 2>&1 || true
echo "SSH configured" >> "$LOG"

# 3. Bring up Wi-Fi and static IP 192.168.137.203 immediately
rfkill unblock all || true
ip link set wlan0 up || true
iw dev wlan0 set power_save off || true
killall wpa_supplicant 2>/dev/null || true
sleep 1
if [ -f "$BOOT/wpa_supplicant.conf" ]; then
    cp "$BOOT/wpa_supplicant.conf" /etc/wpa_supplicant/wpa_supplicant.conf
fi
wpa_supplicant -B -i wlan0 -c /etc/wpa_supplicant/wpa_supplicant.conf >> "$LOG" 2>&1 || true
sleep 3
ip addr add 192.168.137.203/24 dev wlan0 2>/dev/null || true
ip link set wlan0 up
ip route add default via 192.168.137.1 dev wlan0 2>/dev/null || true
echo "nameserver 192.168.137.1" > /etc/resolv.conf
echo "Wi-Fi started, IP assigned:" >> "$LOG"
ip addr show wlan0 >> "$LOG" 2>&1

# Install wifi_keepalive background watchdog
if [ -f "$BOOT/wifi_keepalive.sh" ]; then
    cp "$BOOT/wifi_keepalive.sh" /usr/local/bin/wifi_keepalive.sh
    chmod +x /usr/local/bin/wifi_keepalive.sh
    systemctl restart wifi-keepalive.service 2>/dev/null || true
fi

# 4. Copy edge_config.json and edge_daemon.py
mkdir -p /home/pi/edge_data
cp "$BOOT/edge_config.json" /home/pi/edge_config.json
cp "$BOOT/edge_daemon.py" /home/pi/edge_daemon.py
chmod +x /home/pi/edge_daemon.py
chmod 644 /home/pi/edge_config.json
chown -R 1000:1000 /home/pi

# 5. Setup and restart edge-daemon service
cat << 'EOF' > /etc/systemd/system/edge-daemon.service
[Unit]
Description=Multi-Sensor Edge Collector Daemon
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi
Environment=PYTHONUNBUFFERED=1
Environment=EDGE_CONFIG=/home/pi/edge_config.json
ExecStart=/usr/bin/python3 /home/pi/edge_daemon.py
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable --now edge-daemon.service >> "$LOG" 2>&1 || true
systemctl restart edge-daemon.service >> "$LOG" 2>&1 || true

echo "=== FIRSTRUN CONFIG COMPLETE $(date) ===" >> "$LOG"
