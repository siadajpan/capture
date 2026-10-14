import sys
import paramiko

def run(cmd):
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect('192.168.137.214', username='pi', password='raspberry', timeout=5)
    stdin, stdout, stderr = client.exec_command(cmd)
    out = stdout.read().decode()
    err = stderr.read().decode()
    client.close()
    if out:
        print("STDOUT:\n" + out)
    if err:
        print("STDERR:\n" + err)

if __name__ == '__main__':
    run(" ".join(sys.argv[1:]))
