import urllib.request
import json
import base64
import hashlib
import time

FLEETS = [
    {"name": "fleet1", "url": "https://fleet1.capture.me.uk", "dest": "/home/pi/edge_agent/edge_daemon.py", "service": "edge-agent"},
    {"name": "fleet2", "url": "https://fleet2.capture.me.uk", "dest": "/home/pi/edge_agent/edge_daemon.py", "service": "edge-agent"},
    {"name": "fleet3", "url": "https://fleet3.capture.me.uk", "dest": "/home/pi/edge_daemon.py", "service": "edge-daemon"},
]

def exec_cmd(url, cmd, max_retries=3):
    for attempt in range(max_retries):
        try:
            req = urllib.request.Request(
                f"{url}/api/exec",
                data=json.dumps({"cmd": cmd}).encode(),
                headers={
                    "Content-Type": "application/json",
                    "User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64)"
                }
            )
            with urllib.request.urlopen(req, timeout=30) as r:
                return json.loads(r.read().decode())
        except Exception as e:
            if attempt == max_retries - 1:
                raise e
            time.sleep(2)

with open("edge_agent/edge_daemon.py", "rb") as f:
    daemon_bytes = f.read()

expected_hash = hashlib.sha256(daemon_bytes).hexdigest()
b64_content = base64.b64encode(daemon_bytes).decode("ascii")
chunk_size = 30000
chunks = [b64_content[i:i + chunk_size] for i in range(0, len(b64_content), chunk_size)]
print(f"Total size: {len(daemon_bytes)} bytes, {len(chunks)} chunks, sha256: {expected_hash[:16]}")

for fleet in FLEETS:
    fname = fleet["name"]
    furl = fleet["url"]
    print(f"\nDeploying to {fname} ({furl})...")
    
    dest_path = fleet.get("dest", "/home/pi/edge_daemon.py")
    svc_name = fleet.get("service", "edge-daemon")

    # 1. Clear target temp file
    exec_cmd(furl, f"rm -f {dest_path}.b64 {dest_path}.new")
    
    # 2. Append chunks
    for idx, c in enumerate(chunks):
        cmd = f"printf '%s' '{c}' >> {dest_path}.b64"
        res = exec_cmd(furl, cmd)
        if res.get("returncode") != 0:
            print(f"Error uploading chunk {idx}: {res}")
            break
        print(f"  Chunk {idx+1}/{len(chunks)} uploaded")

    # 3. Decode base64 and verify sha256
    verify_cmd = f"base64 -d {dest_path}.b64 > {dest_path}.new && sha256sum {dest_path}.new"
    res = exec_cmd(furl, verify_cmd)
    remote_out = res.get("stdout", "").strip()
    print(f"  Verification: {remote_out}")

    if expected_hash in remote_out:
        print(f"  Hash MATCH on {fname}! Moving to {dest_path} and restarting service...")
        swap_cmd = f"mv {dest_path}.new {dest_path} && rm -f {dest_path}.b64 && sudo systemctl restart {svc_name}"
        try:
            res2 = exec_cmd(furl, swap_cmd)
            print(f"  Service restart result: {res2}")
        except Exception as restart_ex:
            print(f"  Restart triggered (daemon exited cleanly as expected: {restart_ex})")
        
        # Poll status until back online
        print(f"  Awaiting {fname} service recovery...")
        recovered = False
        for attempt in range(15):
            time.sleep(2)
            try:
                s_req = urllib.request.Request(f"{furl}/api/status", headers={"User-Agent": "Mozilla/5.0"})
                with urllib.request.urlopen(s_req, timeout=5) as sr:
                    if sr.status == 200:
                        print(f"  {fname} is back ONLINE and serving requests!")
                        recovered = True
                        break
            except Exception:
                pass
        if not recovered:
            print(f"  WARNING: {fname} took longer than 30s to respond")
    else:
        print(f"  ERROR: Hash mismatch on {fname}! Aborting swap.")

print("\nDeployment complete!")
