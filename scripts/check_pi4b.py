import sys
import paramiko

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8')

def test_hardware():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("192.168.137.204", username="pi", password="raspberry", timeout=10)

    commands = [
        "which ffmpeg convert cjpeg ppmtojpeg",
        "python3 -c 'import pygame; print(\"pygame ok\")' 2>&1",
        "python3 -c 'import cv2; print(\"cv2 ok\")' 2>&1",
        "python3 -c 'import serial; print(\"pyserial ok\")' 2>&1",
        "sudo stty -F /dev/ttyUSB0 230400 raw -echo && sudo timeout 2 xxd -p -l 64 /dev/ttyUSB0 || true",
        "sudo stty -F /dev/ttyUSB0 115200 raw -echo && sudo timeout 2 xxd -p -l 64 /dev/ttyUSB0 || true"
    ]

    for cmd in commands:
        print(f"\n=================== RUNNING: {cmd} ===================")
        stdin, stdout, stderr = ssh.exec_command(cmd)
        out = stdout.read().decode('utf-8', errors='replace')
        err = stderr.read().decode('utf-8', errors='replace')
        if out:
            print(out)
        if err:
            print("STDERR:", err)

    ssh.close()

if __name__ == "__main__":
    test_hardware()
