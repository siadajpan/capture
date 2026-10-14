import os, fcntl, errno

I2C_SLAVE = 0x0703
I2C_SLAVE_FORCE = 0x0706

for bus in [0, 1, 10, 11, 2]:
    path = f"/dev/i2c-{bus}"
    if not os.path.exists(path):
        continue
    print(f"--- Scanning {path} ---")
    try:
        fd = os.open(path, os.O_RDWR)
        for addr in range(0x03, 0x78):
            try:
                # Force slave address even if driver is bound
                fcntl.ioctl(fd, I2C_SLAVE_FORCE, addr)
                # Try SMBus quick write (write 0 bytes)
                os.write(fd, b"")
                print(f"  [ACK] Device responded at 0x{addr:02x} on {path}!")
            except OSError as err:
                if err.errno == errno.EBUSY:
                    print(f"  [BUSY] Address 0x{addr:02x} is in use by kernel driver on {path}!")
                elif err.errno in (errno.EREMOTEIO, errno.ENXIO, errno.ETIMEDOUT):
                    pass
                else:
                    # Let's see other errors for 0x10
                    if addr == 0x10:
                        print(f"  [ERR] Address 0x10 gave errno {err.errno} ({err})")
        os.close(fd)
    except Exception as e:
        print(f"Error opening {path}: {e}")

print("Scan complete.")
