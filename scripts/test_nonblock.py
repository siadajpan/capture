import sys
import os
import time
import termios
import tty

print("Opening /dev/ttyUSB0 with O_NONBLOCK...")
try:
    fd = os.open('/dev/ttyUSB0', os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
except Exception as e:
    print("Failed to open:", e)
    sys.exit(1)

# Configure raw 230400
attrs = termios.tcgetattr(fd)
tty.setraw(fd)
speed = getattr(termios, 'B230400', termios.B115200)
attrs[4] = speed
attrs[5] = speed
attrs[2] &= ~termios.CRTSCTS  # NO hardware flow control
termios.tcsetattr(fd, termios.TCSANOW, attrs)

# Send stop, then start scan command
print("Sending start scan command 0xA5 0x60...")
os.write(fd, b"\xa5\x60")

total_bytes = 0
chunks = []
t0 = time.time()
while time.time() - t0 < 3.0:
    try:
        data = os.read(fd, 256)
        if data:
            total_bytes += len(data)
            chunks.append(data)
            if len(chunks) == 1:
                print(f"First data received: {len(data)} bytes! Hex: {data[:20].hex()}")
    except BlockingIOError:
        time.sleep(0.01)
    except Exception as e:
        print("Read error:", e)
        break

print(f"Total bytes received in 3s: {total_bytes}")
if chunks:
    sample = b"".join(chunks)[:100]
    print(f"Hex dump: {sample.hex()}")

os.close(fd)
