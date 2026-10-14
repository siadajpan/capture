import paramiko

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect('192.168.137.201', username='pi', password='raspberry', timeout=15)

sftp = client.open_sftp()
with sftp.open('/home/pi/vd55g0-linux-driver/vd55g0.c', 'r') as f:
    code = f.read().decode('utf-8')

# Revert previous edits if any
code = code.replace('\\tdev_info(&client->dev, " i2c_transfer returned %d val=0x%02x 0x%02x\\\n, ret, val[0], val[1]);\n', '')
code = code.replace('\t/* debug */\n', '')

target = 'ret = i2c_transfer(client->adapter, msg, 2);'
addition = '\n\tdev_info(&client->dev, "vd55g0 i2c_transfer reg=0x%04x ret=%d val[0]=0x%02x\\n", reg, ret, val[0]);'

if addition not in code:
    code = code.replace(target, target + addition)

with sftp.open('/home/pi/vd55g0-linux-driver/vd55g0.c', 'w') as f:
    f.write(code.encode('utf-8'))

sftp.close()
client.close()
print("Patched vd55g0.c successfully.")
