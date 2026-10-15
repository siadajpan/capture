import urllib.request
import re

for mod, pool in [('pillow', 'p/pillow/'), ('pyserial', 'p/pyserial/')]:
    url = f"http://raspbian.raspberrypi.com/raspbian/pool/main/{pool}"
    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            html = resp.read().decode('utf-8', errors='replace')
            pkgs = re.findall(r'href="([^"]*\.deb)"', html)
            print(f"=== {mod} ===")
            for p in pkgs:
                if 'armhf' in p or 'all' in p:
                    print(f"  http://raspbian.raspberrypi.com/raspbian/pool/main/{pool}{p}")
    except Exception as e:
        print(f"Error {mod}:", e)
