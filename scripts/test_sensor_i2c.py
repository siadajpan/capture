import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('192.168.137.201', username='pi', password='raspberry', timeout=15)

script = '''
import ctypes, os, struct, time

I2C_M_RD = 0x0001
I2C_RDWR = 0x0707

class i2c_msg(ctypes.Structure):
    _fields_ = [
        ("addr", ctypes.c_ushort),
        ("flags", ctypes.c_ushort),
        ("len", ctypes.c_ushort),
        ("buf", ctypes.POINTER(ctypes.c_char))
    ]

class i2c_rdwr_ioctl_data(ctypes.Structure):
    _fields_ = [
        ("msgs", ctypes.POINTER(i2c_msg)),
        ("nmsgs", ctypes.c_uint)
    ]

def read_reg(fd, addr, reg, nbytes):
    # reg is 16-bit big endian
    reg_buf = struct.pack(">H", reg)
    val_buf = ctypes.create_string_buffer(nbytes)
    
    msgs = (i2c_msg * 2)(
        i2c_msg(addr=addr, flags=0, len=2, buf=ctypes.cast(reg_buf, ctypes.POINTER(ctypes.c_char))),
        i2c_msg(addr=addr, flags=I2C_M_RD, len=nbytes, buf=val_buf)
    )
    
    data = i2c_rdwr_ioctl_data(msgs=msgs, nmsgs=2)
    libc = ctypes.CDLL(None, use_errno=True)
    ret = libc.ioctl(fd, I2C_RDWR, ctypes.byref(data))
    if ret < 0:
        err = ctypes.get_errno()
        raise OSError(err, os.strerror(err))
    return bytes(val_buf[:nbytes])

def write_reg(fd, addr, reg, val_bytes):
    reg_buf = struct.pack(">H", reg) + val_bytes
    msgs = (i2c_msg * 1)(
        i2c_msg(addr=addr, flags=0, len=len(reg_buf), buf=ctypes.cast(reg_buf, ctypes.POINTER(ctypes.c_char)))
    )
    data = i2c_rdwr_ioctl_data(msgs=msgs, nmsgs=1)
    ret = ctypes.CDLL(None).ioctl(fd, I2C_RDWR, ctypes.byref(data))
    if ret < 0:
        raise OSError(f"ioctl write failed: {ret}")

fd = os.open("/dev/i2c-10", os.O_RDWR)
print("Reading registers at 0x10 on /dev/i2c-10:")

for reg in [0x0000, 0x0002, 0x0004, 0x0012, 0x001c, 0x002c, 0x0200, 0x0670]:
    try:
        val = read_reg(fd, 0x10, reg, 4 if reg in [0x0000, 0x0670] else 2)
        hex_val = " ".join(f"0x{b:02x}" for b in val)
        print(f"  Reg 0x{reg:04x}: raw=[{hex_val}]")
    except Exception as e:
        print(f"  Reg 0x{reg:04x} read error: {e}")

os.close(fd)
'''

sftp = ssh.open_sftp()
with sftp.open('/home/pi/test_sensor.py', 'w') as f:
    f.write(script)
sftp.close()

stdin, stdout, stderr = ssh.exec_command('echo raspberry | sudo -S pinctrl set 44 dh && echo raspberry | sudo -S python3 /home/pi/test_sensor.py')
print(stdout.read().decode('utf-8'))
print(stderr.read().decode('utf-8'))
ssh.close()
