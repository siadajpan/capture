import sys
import paramiko

def run(cmd):
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect('192.168.137.201', username='pi', password='raspberry', timeout=15)
    
    # If command uses sudo, pass password via -S
    if 'sudo ' in cmd and 'echo raspberry' not in cmd:
        cmd = cmd.replace('sudo ', 'echo raspberry | sudo -S ')
        
    stdin, stdout, stderr = client.exec_command(cmd)
    out = stdout.read().decode('utf-8', errors='replace')
    err = stderr.read().decode('utf-8', errors='replace')
    client.close()
    if out:
        sys.stdout.buffer.write(out.encode('utf-8'))
        sys.stdout.buffer.write(b'\n')
    if err:
        sys.stderr.buffer.write(err.encode('utf-8'))
        sys.stderr.buffer.write(b'\n')

if __name__ == '__main__':
    run(" ".join(sys.argv[1:]))
