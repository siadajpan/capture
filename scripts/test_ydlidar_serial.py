import sys
import paramiko
import time

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8')

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('192.168.137.204', username='pi', password='raspberry', timeout=10)

# Stop edge-daemon so we have exclusive access to /dev/ttyUSB0
ssh.exec_command('sudo systemctl stop edge-daemon.service')
time.sleep(1)

test_py = """
import time
import serial

print("Testing YDLIDAR T-mini Plus on /dev/ttyUSB0...")

for baud in [230400, 115200, 128000]:
    for (dtr, rts) in [(False, False), (True, False), (False, True), (True, True)]:
        try:
            ser = serial.Serial('/dev/ttyUSB0', baudrate=baud, timeout=0.8)
            ser.dtr = dtr
            ser.rts = rts
            ser.reset_input_buffer()
            # Send stop first, then get info, then start
            ser.write(b'\\xa5\\x65')
            time.sleep(0.05)
            ser.reset_input_buffer()
            ser.write(b'\\xa5\\x90')
            time.sleep(0.1)
            info_resp = ser.read(20)

            # Try start scan
            ser.write(b'\\xa5\\x60')
            time.sleep(0.15)
            scan_resp = ser.read(50)

            ser.close()

            if info_resp or scan_resp:
                print(f"--> SUCCESS at baud {baud}, DTR={dtr}, RTS={rts}:")
                if info_resp:
                    print(f"    Info ({len(info_resp)}B): {info_resp.hex()}")
                if scan_resp:
                    print(f"    Scan ({len(scan_resp)}B): {scan_resp[:30].hex()}")
                break
        except Exception as e:
            print(f"Error at {baud}: {e}")
            break
"""

sftp = ssh.open_sftp()
with sftp.file('/home/pi/test_lidar.py', 'w') as f:
    f.write(test_py)
sftp.close()

stdin, stdout, stderr = ssh.exec_command('python3 /home/pi/test_lidar.py')
out = stdout.read().decode('utf-8', errors='replace')
err = stderr.read().decode('utf-8', errors='replace')
print("OUTPUT:\n", out)
if err:
    print("ERR:\n", err)

ssh.close()
