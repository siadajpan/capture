import os
import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('192.168.137.201', username='pi', password='raspberry', timeout=15)
sftp = ssh.open_sftp()

local_dir = r'c:\Users\kmisiarz\projects\capture\vd55g1-linux-driver'
remote_dir = '/home/pi/vd55g1-linux-driver'

def mkdir_p(remote_path):
    dirs = []
    p = remote_path
    while p not in ('', '/', '.'):
        dirs.append(p)
        p = os.path.dirname(p)
    dirs.reverse()
    for d in dirs:
        try:
            sftp.mkdir(d)
        except IOError:
            pass

mkdir_p(remote_dir)

for root, dirs, files in os.walk(local_dir):
    if '.git' in root:
        continue
    rel = os.path.relpath(root, local_dir)
    rem_root = remote_dir if rel == '.' else remote_dir + '/' + rel.replace('\\', '/')
    mkdir_p(rem_root)
    for f in files:
        loc_f = os.path.join(root, f)
        rem_f = rem_root + '/' + f
        print(f"Uploading {rel}/{f} -> {rem_f}")
        sftp.put(loc_f, rem_f)

sftp.close()
ssh.close()
print("Uploaded vd55g1-linux-driver to Pi 2 successfully.")
