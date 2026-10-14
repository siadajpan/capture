import os
import shutil
import time

def deploy_to_sd(drive="E:"):
    print(f"Waiting for drive {drive}\\...")
    while not os.path.exists(f"{drive}\\"):
        time.sleep(1)
        print(".", end="", flush=True)
    print(f"\nDrive {drive}\\ detected!")

    # 1. Copy firstrun.sh
    firstrun_src = os.path.join("scripts", "sd_firstrun_update.sh")
    firstrun_dst = f"{drive}\\firstrun.sh"
    data = open(firstrun_src, "rb").read().replace(b"\r\n", b"\n")
    open(firstrun_dst, "wb").write(data)
    print(f"Copied {firstrun_dst} ({len(data)} bytes, LF endings)")

    # 2. Copy edge_daemon.py
    daemon_src = os.path.join("edge_agent", "edge_daemon.py")
    daemon_dst = f"{drive}\\edge_daemon.py"
    data = open(daemon_src, "rb").read().replace(b"\r\n", b"\n")
    open(daemon_dst, "wb").write(data)
    print(f"Copied {daemon_dst} ({len(data)} bytes, LF endings)")

    # 3. Copy edge_config.json
    config_dst = f"{drive}\\edge_config.json"
    config_content = """{
  "device_id": "edge-pi-zero-03",
  "device_name": "Raspberry Pi Zero (Camera v2)",
  "host": "0.0.0.0",
  "port": 5001,
  "storage_root": "/home/pi/edge_data",
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
    }
  ]
}
"""
    open(config_dst, "wb").write(config_content.replace("\r\n", "\n").encode("utf-8"))
    print(f"Copied {config_dst}")

    # 4. Touch ssh
    ssh_dst = f"{drive}\\ssh"
    open(ssh_dst, "wb").write(b"")
    print(f"Created {ssh_dst}")

    # 5. Check cmdline.txt
    cmdline_dst = f"{drive}\\cmdline.txt"
    if os.path.exists(cmdline_dst):
        cmdline = open(cmdline_dst, "r").read()
        if 'systemd.run="sh /boot/firmware/firstrun.sh"' not in cmdline:
            cmdline = cmdline.strip() + ' systemd.run="sh /boot/firmware/firstrun.sh" systemd.run_success_action=none systemd.run_failure_action=none\n'
            open(cmdline_dst, "wb").write(cmdline.encode("utf-8"))
            print(f"Updated {cmdline_dst}")
        else:
            print(f"{cmdline_dst} already has systemd.run hook.")

    print("\nALL FILES DEPLOYED TO SD CARD SUCCESSFULLY!")

if __name__ == "__main__":
    deploy_to_sd()
