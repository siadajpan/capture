import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('192.168.137.204', username='pi', password='raspberry', timeout=10)
stdin, stdout, stderr = ssh.exec_command("python3 -c 'from PIL import Image, ImageDraw; img = Image.new(\"RGB\", (100, 100)); draw = ImageDraw.Draw(img); draw.ellipse([10, 10, 90, 90], fill=(255, 0, 0)); print(\"ImageDraw test passed!\")'")
print(stdout.read().decode())
print(stderr.read().decode())
ssh.close()
