import os
import urllib.request
import paramiko

debs = [
    "http://raspbian.raspberrypi.com/raspbian/pool/main/p/pyserial/python3-serial_3.5-2_all.deb",
    "http://raspbian.raspberrypi.com/raspbian/pool/main/p/pillow/python3-pil_12.3.0-1_armhf.deb",
    "http://raspbian.raspberrypi.com/raspbian/pool/main/p/pillow/python3-pil_11.1.0-5+deb13u4_armhf.deb"
]

os.makedirs("scratch/debs", exist_ok=True)
downloaded = []

for url in debs:
    fname = os.path.basename(url)
    local_path = os.path.join("scratch", "debs", fname)
    if not os.path.exists(local_path):
        print(f"Downloading {fname}...")
        req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
        with urllib.request.urlopen(req, timeout=30) as resp, open(local_path, "wb") as f:
            f.write(resp.read())
        print(f"Downloaded {local_path} ({os.path.getsize(local_path)} bytes)")
    downloaded.append((fname, local_path))

# Connect to Pi 4B and copy debs
ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
print("Connecting to 192.168.137.204...")
ssh.connect("192.168.137.204", username="pi", password="raspberry", timeout=10)

sftp = ssh.open_sftp()
for fname, local_path in downloaded:
    remote_path = f"/home/pi/{fname}"
    print(f"Uploading {fname} to {remote_path}...")
    sftp.put(local_path, remote_path)
sftp.close()

# Install python3-serial first
print("Installing python3-serial...")
stdin, stdout, stderr = ssh.exec_command("sudo dpkg -i /home/pi/python3-serial_3.5-2_all.deb")
print(stdout.read().decode())
print(stderr.read().decode())

# Try installing python3-pil 12.3.0 or 11.1.0
print("Installing python3-pil...")
stdin, stdout, stderr = ssh.exec_command("sudo dpkg -i /home/pi/python3-pil_12.3.0-1_armhf.deb || sudo dpkg -i /home/pi/python3-pil_11.1.0-5+deb13u4_armhf.deb")
print(stdout.read().decode())
print(stderr.read().decode())

# Test import PIL and serial
print("Testing imports...")
stdin, stdout, stderr = ssh.exec_command("python3 -c 'import PIL; print(\"PIL OK!\", PIL.__file__)' 2>&1; python3 -c 'import serial; print(\"serial OK!\", serial.__file__)' 2>&1")
print(stdout.read().decode())
print(stderr.read().decode())

ssh.close()
