import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('192.168.137.204', username='pi', password='raspberry', timeout=10)

stdin, stdout, stderr = ssh.exec_command("ps -ef | grep python; systemctl is-active edge-daemon.service")
print("PROCS & STATUS:\n", stdout.read().decode())
print("ERR:\n", stderr.read().decode())
ssh.close()
