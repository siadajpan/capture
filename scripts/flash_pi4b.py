import os
import shutil
import time

def deploy_to_pi4b_sd(drive="E:"):
    print(f"Checking drive {drive}\\...")
    if not os.path.exists(f"{drive}\\"):
        print(f"Drive {drive}\\ not found!")
        return False
    print(f"Drive {drive}\\ detected.")

    # 1. edge_daemon.py
    daemon_src = os.path.join("edge_agent", "edge_daemon.py")
    daemon_dst = f"{drive}\\edge_daemon.py"
    data = open(daemon_src, "rb").read().replace(b"\r\n", b"\n")
    open(daemon_dst, "wb").write(data)
    print(f"Copied {daemon_dst} ({len(data)} bytes, LF endings)")

    # 2. edge_config.json
    config_dst = f"{drive}\\edge_config.json"
    config_content = """{
  "device_id": "edge-pi-4b",
  "device_name": "Raspberry Pi 4B (Camera v2 + LiDAR)",
  "host": "0.0.0.0",
  "port": 5001,
  "storage_root": "/home/pi/edge_data",
  "server_url": "http://192.168.137.1:8000",
  "sensors": [
    {
      "id": "rgb-camerav2",
      "modality": "rgb",
      "name": "Raspberry Pi Camera Module v2 (IMX219)",
      "type": "rpicam",
      "device": "/dev/video0",
      "width": 640,
      "height": 480,
      "fps": 15
    },
    {
      "id": "lidar-topdown",
      "modality": "depth",
      "name": "LiDAR YDTMP-12 25ARZ0006 (Top-Down Radar)",
      "type": "ydlidar",
      "device": "/dev/ttyUSB0",
      "baudrate": 230400,
      "width": 640,
      "height": 640,
      "fps": 10
    }
  ]
}
"""
    open(config_dst, "wb").write(config_content.replace("\r\n", "\n").encode("utf-8"))
    print(f"Copied {config_dst}")

    # 3. wpa_supplicant.conf
    wpa_dst = f"{drive}\\wpa_supplicant.conf"
    wpa_content = """country=BE
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

network={
    ssid="karol"
    psk="klapeczki"
    key_mgmt=WPA-PSK
}
"""
    open(wpa_dst, "wb").write(wpa_content.replace("\r\n", "\n").encode("utf-8"))
    print(f"Copied {wpa_dst}")

    # 4. wifi_keepalive.sh
    keepalive_dst = f"{drive}\\wifi_keepalive.sh"
    keepalive_content = """#!/bin/bash
BOOT="/boot/firmware"
[ ! -d "$BOOT" ] && BOOT="/boot"
LOG="$BOOT/firstrun.log"
echo "=== WIFI KEEPALIVE START $(date) ===" >> "$LOG"

rfkill unblock all || true
ip link set wlan0 up || true
iw dev wlan0 set power_save off || true
iwconfig wlan0 power off 2>/dev/null || true

# Start wpa_supplicant if not running
if ! pgrep -x "wpa_supplicant" > /dev/null; then
    killall wpa_supplicant 2>/dev/null || true
    sleep 1
    wpa_supplicant -B -i wlan0 -c /etc/wpa_supplicant/wpa_supplicant.conf >> "$LOG" 2>&1
    sleep 4
fi

# Assign static IP on hotspot
ip addr add 192.168.137.204/24 dev wlan0 2>/dev/null || true
ip link set wlan0 up
ip route add default via 192.168.137.1 dev wlan0 2>/dev/null || true
echo "nameserver 192.168.137.1" > /etc/resolv.conf

echo "=== IP ASSIGNED ===" >> "$LOG"
ip addr show wlan0 >> "$LOG" 2>&1

# Background watchdog loop to keep link up and power-save off
while true; do
    iw dev wlan0 set power_save off 2>/dev/null || true
    if ! ip addr show wlan0 | grep -q "192.168.137.204"; then
        ip addr add 192.168.137.204/24 dev wlan0 2>/dev/null || true
        ip route add default via 192.168.137.1 dev wlan0 2>/dev/null || true
    fi
    sleep 5
done
"""
    open(keepalive_dst, "wb").write(keepalive_content.replace("\r\n", "\n").encode("utf-8"))
    print(f"Copied {keepalive_dst}")

    # 5. firstrun.sh
    firstrun_dst = f"{drive}\\firstrun.sh"
    firstrun_content = """#!/bin/bash
BOOT="/boot/firmware"
[ ! -d "$BOOT" ] && BOOT="/boot"
LOG="$BOOT/firstrun.log"

echo "=== FIRSTRUN START $(date) ===" > "$LOG"

# 1. Update hostname to pi-4b-01
echo "pi-4b-01" > /etc/hostname
hostname pi-4b-01 2>/dev/null || true
hostnamectl set-hostname pi-4b-01 2>/dev/null || true
if grep -q "127.0.1.1" /etc/hosts; then
    sed -i 's/127.0.1.1.*/127.0.1.1\\tpi-4b-01/' /etc/hosts
else
    echo -e "127.0.1.1\\tpi-4b-01" >> /etc/hosts
fi
echo "Hostname set to $(cat /etc/hostname)" >> "$LOG"

# 2. Fix and unblock SSH login completely
rm -f /etc/ssh/sshd_config.d/rename_user.conf /run/nologin /etc/nologin
sed -i 's/^account.*pam_nologin.so/# &/' /etc/pam.d/sshd 2>/dev/null || true

id -u pi >> "$LOG" 2>&1 || useradd -m -s /bin/bash pi >> "$LOG" 2>&1 || true
chsh -s /bin/bash pi 2>/dev/null || true
echo "pi:raspberry" | chpasswd >> "$LOG" 2>&1 || true
echo "root:raspberry" | chpasswd >> "$LOG" 2>&1 || true
usermod -aG sudo,video,input,dialout,tty pi 2>/dev/null || true
echo "pi ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/010_pi-nopasswd
chmod 440 /etc/sudoers.d/010_pi-nopasswd

# Install Windows SSH public keys
mkdir -p /home/pi/.ssh
cat << 'EOF' > /home/pi/.ssh/authorized_keys
ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAACAQDaJQ6LpNKgrP2vCka3dOYquvPNRkIhk6EWbleagHlmVdDCLlLB9C4Qobmr1TVLa9ZZb6z19ov8cvWeyyqO+FYRaT4RhDRfaAOzx1eG8adCc2swS56pQR+fQcaLvBgk/twwfKZDrbfOdIUEK6h9CXd5PLH9dNQWEHdPJM7ikZq7TDDtp/k2im9717RrMMe10K8A/RxiAVoAJlmO20OAY5T2b5n1yryctq72kkBjpZMuJUgWIlMyYTHbPo9p3+IyWF2cKdbzHoYPhZZl+UHCv6or+u3PslhaDJZOTxRP8Ek/2mCIINxzJ1exMb6HM/hYGYTdFDJirAILvkpfIjzdj+Jb+lBfr9uAJjKB8LwTRc4srwI+Oi/RfS0KaJQebXe5kYVjvTm6Znk//XVVCWLKNzTV+j8ZMkXUhP9ZfXXToQNl8VGrQPcxFxZbID7Ixm7rapqMTDU5m+iFhFUBPvfaWYfcb8dpXVPP5CfwYU6IwGT2AieOpRdzVVksB2kvcQSOx+Rc61WbI6Ldg/km8CF1VuyQRoMYfpT1bXxJtWlRT1O7jFFHKDbbkXm33tLznK0SkqDfv4oa91yPou7wci1kUWFcxs3gzLWbx+fApGA+lREoIXaW9uls+3aarrAIk1GpOg4oBIyJ9yy7K1iegTkXto8JZCrX6pfPqhlcVsjudiCG2w== ugent\\kmisiarz@UG-55LQ954
ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAILZJwXVyeiYX8ks3PEsnmbxjWfG/jWAiaZpZdx0zGkR0 ugent\\kmisiarz@UG-55LQ954
EOF
chown -R 1000:1000 /home/pi/.ssh
chmod 700 /home/pi/.ssh
chmod 600 /home/pi/.ssh/authorized_keys
systemctl restart ssh >> "$LOG" 2>&1 || true
echo "SSH configured" >> "$LOG"

# 3. Disable NetworkManager so it doesn't conflict
systemctl stop NetworkManager 2>/dev/null || true
systemctl disable NetworkManager 2>/dev/null || true
systemctl mask NetworkManager 2>/dev/null || true

# 4. Bring up Wi-Fi and static IP 192.168.137.204 immediately
mkdir -p /etc/wpa_supplicant
if [ -f "$BOOT/wpa_supplicant.conf" ]; then
    cp "$BOOT/wpa_supplicant.conf" /etc/wpa_supplicant/wpa_supplicant.conf
fi
chmod 600 /etc/wpa_supplicant/wpa_supplicant.conf

if [ -f "$BOOT/wifi_keepalive.sh" ]; then
    cp "$BOOT/wifi_keepalive.sh" /usr/local/bin/wifi_keepalive.sh
    chmod +x /usr/local/bin/wifi_keepalive.sh
    cat << 'EOF' > /etc/systemd/system/wifi-keepalive.service
[Unit]
Description=Wi-Fi Keepalive to Karol Hotspot
After=network.target

[Service]
Type=simple
ExecStart=/bin/bash /usr/local/bin/wifi_keepalive.sh
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF
    systemctl daemon-reload
    systemctl enable --now wifi-keepalive.service >> "$LOG" 2>&1 || true
fi

# 5. Copy edge_config.json and edge_daemon.py
mkdir -p /home/pi/edge_data
cp "$BOOT/edge_config.json" /home/pi/edge_config.json
cp "$BOOT/edge_daemon.py" /home/pi/edge_daemon.py
chmod +x /home/pi/edge_daemon.py
chmod 644 /home/pi/edge_config.json
chown -R 1000:1000 /home/pi

# 6. Setup and start edge-daemon service
cat << 'EOF' > /etc/systemd/system/edge-daemon.service
[Unit]
Description=Multi-Sensor Edge Collector Daemon (Camera v2 + LiDAR)
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

# 7. Background package update & dependencies installation
(
    sleep 5
    apt-get update -y
    apt-get install -y python3-pil python3-serial rpicam-apps-lite v4l-utils
    systemctl restart edge-daemon.service
) >> "$BOOT/apt_install.log" 2>&1 &

echo "=== FIRSTRUN CONFIG COMPLETE $(date) ===" >> "$LOG"
"""
    open(firstrun_dst, "wb").write(firstrun_content.replace("\r\n", "\n").encode("utf-8"))
    print(f"Copied {firstrun_dst}")

    # 6. ssh empty file
    ssh_dst = f"{drive}\\ssh"
    open(ssh_dst, "wb").write(b"")
    print(f"Created {ssh_dst}")

    # 7. userconf.txt
    userconf_dst = f"{drive}\\userconf.txt"
    userconf_content = "pi:$y$jB5$qIHEXtmMMFrErM2SkNpQs.$Xqwdv5Ki/DqG6nNpnqL3jKVTbVhZup1/iC.buKyq2NB\n"
    open(userconf_dst, "wb").write(userconf_content.encode("utf-8"))
    print(f"Created {userconf_dst}")

    # 8. config.txt updates
    config_txt_dst = f"{drive}\\config.txt"
    if os.path.exists(config_txt_dst):
        c_lines = open(config_txt_dst, "r").read().splitlines()
        new_lines = []
        for line in c_lines:
            if line.startswith("camera_auto_detect="):
                new_lines.append("camera_auto_detect=0")
            else:
                new_lines.append(line)
        
        # Ensure dtoverlay=imx219 and enable_uart=1 are present
        content = "\n".join(new_lines)
        if "dtoverlay=imx219" not in content:
            content += "\n\n# Raspberry Pi Camera Module v2\ndtoverlay=imx219\n"
        if "enable_uart=1" not in content:
            content += "enable_uart=1\n"
        open(config_txt_dst, "wb").write((content + "\n").encode("utf-8"))
        print(f"Updated {config_txt_dst}")

    # 9. cmdline.txt hook
    cmdline_dst = f"{drive}\\cmdline.txt"
    if os.path.exists(cmdline_dst):
        cmdline = open(cmdline_dst, "r").read().strip()
        hook = 'systemd.run="sh /boot/firmware/firstrun.sh" systemd.run_success_action=none systemd.run_failure_action=none'
        if hook not in cmdline:
            cmdline += f" {hook}\n"
            open(cmdline_dst, "wb").write(cmdline.encode("utf-8"))
            print(f"Updated {cmdline_dst}")
        else:
            print(f"{cmdline_dst} already has firstrun hook.")

    print("\n>>> ALL PI 4B SD CARD BOOT FILES FLASHED SUCCESSFULLY! <<<")
    return True

if __name__ == "__main__":
    deploy_to_pi4b_sd()
