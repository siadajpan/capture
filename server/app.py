#!/usr/bin/env python3
"""
Central Hub & Web Server for Multi-Sensor Edge Collection.
Mobile-First Dashboard & Edge Orchestrator.
"""

import os
import sys
import time
import json
import socket
import logging
import asyncio
from typing import Dict, Any, List, Optional
from datetime import datetime

import aiohttp
from fastapi import FastAPI, HTTPException, Request, BackgroundTasks
from fastapi.responses import HTMLResponse, StreamingResponse, JSONResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [%(levelname)s] [HubServer] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)
logger = logging.getLogger("hub_server")

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(BASE_DIR, "static")
REGISTRY_FILE = os.path.join(BASE_DIR, "edges_registry.json")
CACHE_DIR = os.path.join(BASE_DIR, "cache", "recordings")
os.makedirs(CACHE_DIR, exist_ok=True)

app = FastAPI(title="Edge Multi-Sensor Collector Hub", version="1.0.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.middleware("http")
async def add_pna_header(request: Request, call_next):
    if request.method == "OPTIONS":
        response = JSONResponse(content={})
        response.headers["Access-Control-Allow-Origin"] = "*"
        response.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS, DELETE"
        response.headers["Access-Control-Allow-Headers"] = "*"
        response.headers["Access-Control-Allow-Private-Network"] = "true"
        return response
    response = await call_next(request)
    response.headers["Access-Control-Allow-Private-Network"] = "true"
    return response

# Registry of known edge devices
# key: device_id
known_edges: Dict[str, Dict[str, Any]] = {}
display_enabled: bool = True


def get_local_ip():
    """Detect local LAN IP for easy phone connection."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Doesn't need to be reachable
        s.connect(('10.255.255.255', 1))
        ip = s.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
    finally:
        s.close()
    return ip


def load_registry():
    global known_edges
    if os.path.exists(REGISTRY_FILE):
        try:
            with open(REGISTRY_FILE, "r") as f:
                data = json.load(f)
                known_edges = data.get("edges", {})
                logger.info(f"Loaded {len(known_edges)} edges from registry.")
        except Exception as e:
            logger.warning(f"Could not load registry: {e}")
    if not known_edges:
        # Default starter edge: Pi Zero
        known_edges = {
            "edge-pi-zero-01": {
                "device_id": "edge-pi-zero-01",
                "device_name": "Raspberry Pi Zero v1.1",
                "host": "127.0.0.1",
                "port": 5001,
                "url": "http://127.0.0.1:5001",
                "status": "initializing",
                "last_seen": 0,
                "sensors": [],
                "collection": {}
            }
        }


def save_registry():
    try:
        with open(REGISTRY_FILE, "w") as f:
            json.dump({"edges": known_edges}, f, indent=2)
    except Exception as e:
        logger.error(f"Failed to save registry: {e}")


load_registry()


class AddEdgeRequest(BaseModel):
    device_id: str
    device_name: Optional[str] = "Edge Node"
    host: str
    port: int = 5001


class CollectionActionRequest(BaseModel):
    label: Optional[str] = "Collection Session"
    target_devices: Optional[List[str]] = None


async def ping_edge(device_id: str, edge_data: dict, session: aiohttp.ClientSession):
    url = f"http://{edge_data['host']}:{edge_data['port']}/api/status"
    t0 = time.time()
    try:
        async with session.get(url, timeout=aiohttp.ClientTimeout(total=2.0)) as resp:
            if resp.status == 200:
                data = await resp.json()
                edge_data["latency_ms"] = round((time.time() - t0) * 1000, 1)
                edge_data["online"] = True
                edge_data["last_seen"] = time.time()
                edge_data["device_name"] = data.get("device_name", edge_data.get("device_name"))
                edge_data["sensors"] = data.get("sensors", [])
                edge_data["modalities"] = data.get("modalities", [])
                edge_data["collection"] = data.get("collection", {})
                edge_data["network"] = data.get("network", {})
                return
    except Exception:
        pass
    edge_data["online"] = False


def get_hotspot_client_ips():
    """Finds all client IPs in ARP table on the mobile hotspot subnet (192.168.137.x)."""
    ips = []
    try:
        import subprocess
        out = subprocess.check_output("arp -a", shell=True, text=True, timeout=2)
        in_hotspot_section = False
        for line in out.splitlines():
            line_str = line.strip()
            if "192.168.137." in line_str and "Interface:" in line_str:
                in_hotspot_section = True
                continue
            elif "Interface:" in line_str:
                in_hotspot_section = False
            
            if in_hotspot_section:
                parts = line_str.split()
                if len(parts) >= 2 and parts[0].startswith("192.168.137."):
                    ip = parts[0]
                    if ip not in ["192.168.137.1", "192.168.137.255"]:
                        ips.append(ip)
    except Exception:
        pass
    return ips


async def background_poll_edges():
    """Periodically queries all edge devices for status and telemetry with auto-discovery."""
    while True:
        try:
            async with aiohttp.ClientSession() as session:
                tasks = [
                    ping_edge(did, edata, session)
                    for did, edata in list(known_edges.items())
                ]
                if tasks:
                    await asyncio.gather(*tasks)

                # Check hotspot subnet for any active edge daemons
                hotspot_ips = get_hotspot_client_ips()
                for hip in hotspot_ips:
                    already_online = any(e.get("host") == hip and e.get("online") for e in known_edges.values())
                    if already_online:
                        continue
                    
                    probe_url = f"http://{hip}:5001/api/status"
                    try:
                        async with session.get(probe_url, timeout=aiohttp.ClientTimeout(total=1.2)) as probe_resp:
                            if probe_resp.status == 200:
                                pdata = await probe_resp.json()
                                dev_id = pdata.get("device_id")
                                if dev_id:
                                    if dev_id in known_edges:
                                        if known_edges[dev_id].get("host") != hip:
                                            logger.info(f"Auto-updating {dev_id} to new hotspot IP: {hip}")
                                            known_edges[dev_id]["host"] = hip
                                            known_edges[dev_id]["url"] = f"http://{hip}:5001"
                                            save_registry()
                                    else:
                                        logger.info(f"Auto-registering new edge discovered on hotspot: {dev_id} ({hip})")
                                        known_edges[dev_id] = {
                                            "device_id": dev_id,
                                            "device_name": pdata.get("device_name", dev_id),
                                            "host": hip,
                                            "port": 5001,
                                            "url": f"http://{hip}:5001",
                                            "online": True,
                                            "last_seen": time.time(),
                                            "sensors": pdata.get("sensors", []),
                                            "modalities": pdata.get("modalities", []),
                                            "collection": pdata.get("collection", {}),
                                            "network": pdata.get("network", {})
                                        }
                                        save_registry()
                    except Exception:
                        pass
        except Exception as e:
            logger.debug(f"Poll cycle error: {e}")
        await asyncio.sleep(2.0)


@app.on_event("startup")
async def on_startup():
    asyncio.create_task(background_poll_edges())


# ==============================================================================
# REST API Endpoints
# ==============================================================================

@app.get("/api/info")
async def get_system_info():
    ip = get_local_ip()
    tunnel_url = "https://marvel-reconstruction-atm-planning.trycloudflare.com"
    cfg_file = os.path.join(STATIC_DIR, "config.json")
    if os.path.exists(cfg_file):
        try:
            with open(cfg_file, "r") as f:
                c = json.load(f)
                tunnel_url = c.get("default_hub_url", tunnel_url)
        except Exception:
            pass
    return {
        "server_ip": ip,
        "server_port": 8000,
        "mobile_url": f"http://{ip}:8000",
        "tunnel_url": tunnel_url,
        "display_enabled": display_enabled,
        "total_edges": len(known_edges),
        "online_edges": sum(1 for e in known_edges.values() if e.get("online")),
        "time": datetime.now().isoformat()
    }


@app.get("/api/edges")
async def list_edges():
    return {
        "display_enabled": display_enabled,
        "edges": list(known_edges.values())
    }


@app.post("/api/edges")
async def add_edge(req: AddEdgeRequest):
    edge_url = f"http://{req.host}:{req.port}"
    known_edges[req.device_id] = {
        "device_id": req.device_id,
        "device_name": req.device_name or req.device_id,
        "host": req.host,
        "port": req.port,
        "url": edge_url,
        "online": False,
        "last_seen": 0,
        "sensors": [],
        "collection": {}
    }
    save_registry()
    # Ping immediately
    async with aiohttp.ClientSession() as session:
        await ping_edge(req.device_id, known_edges[req.device_id], session)
    return {"status": "added", "edge": known_edges[req.device_id]}


@app.delete("/api/edges/{device_id}")
async def remove_edge(device_id: str):
    if device_id in known_edges:
        del known_edges[device_id]
        save_registry()
        return {"status": "removed", "device_id": device_id}
    raise HTTPException(status_code=404, detail="Edge not found")


@app.post("/api/display/toggle")
async def toggle_display(req: Optional[dict] = None):
    global display_enabled
    if req and "enabled" in req:
        display_enabled = bool(req["enabled"])
    else:
        display_enabled = not display_enabled
    return {"display_enabled": display_enabled}


@app.post("/api/collection/start")
async def start_collection(req: CollectionActionRequest):
    """
    Broadcasts recording start command to all connected edge devices.
    Each edge device stores data locally in: <storage_root>/<date>/<time>/<modality>/
    """
    targets = req.target_devices or list(known_edges.keys())
    results = {}

    async with aiohttp.ClientSession() as session:
        for did in targets:
            edge = known_edges.get(did)
            if not edge or not edge.get("online"):
                results[did] = {"status": "skipped_offline"}
                continue
            url = f"http://{edge['host']}:{edge['port']}/api/collection/start"
            try:
                payload = {"label": req.label}
                async with session.post(url, json=payload, timeout=aiohttp.ClientTimeout(total=4.0)) as resp:
                    if resp.status == 200:
                        data = await resp.json()
                        results[did] = data
                        edge["collection"]["is_recording"] = True
                    else:
                        results[did] = {"status": "error", "code": resp.status}
            except Exception as e:
                results[did] = {"status": "exception", "error": str(e)}

    return {
        "status": "start_broadcasted",
        "action": "start_collection",
        "timestamp": datetime.now().isoformat(),
        "results": results
    }


@app.post("/api/collection/stop")
async def stop_collection(req: Optional[CollectionActionRequest] = None):
    """
    Broadcasts recording stop command to all connected edge devices.
    """
    targets = (req.target_devices if req else None) or list(known_edges.keys())
    results = {}

    async with aiohttp.ClientSession() as session:
        for did in targets:
            edge = known_edges.get(did)
            if not edge or not edge.get("online"):
                results[did] = {"status": "skipped_offline"}
                continue
            url = f"http://{edge['host']}:{edge['port']}/api/collection/stop"
            try:
                async with session.post(url, timeout=aiohttp.ClientTimeout(total=4.0)) as resp:
                    if resp.status == 200:
                        data = await resp.json()
                        results[did] = data
                        edge["collection"]["is_recording"] = False
                    else:
                        results[did] = {"status": "error", "code": resp.status}
            except Exception as e:
                results[did] = {"status": "exception", "error": str(e)}

    return {
        "status": "stop_broadcasted",
        "action": "stop_collection",
        "timestamp": datetime.now().isoformat(),
        "results": results
    }


@app.get("/proxy/stream/{device_id}/{target}")
async def proxy_mjpeg_stream(device_id: str, target: str):
    """
    Proxies live MJPEG stream from the edge device directly to the user's phone browser.
    Shields phone from CORS, network NAT, and internal firewall configurations.
    """
    edge = known_edges.get(device_id)
    if not edge:
        raise HTTPException(status_code=404, detail="Edge device not registered")

    stream_url = f"http://{edge['host']}:{edge['port']}/api/stream/{target}"

    async def stream_generator():
        timeout = aiohttp.ClientTimeout(total=None, connect=5.0)
        async with aiohttp.ClientSession(timeout=timeout) as session:
            try:
                async with session.get(stream_url) as resp:
                    if resp.status != 200:
                        logger.warning(f"Failed to connect to edge stream {stream_url}: status {resp.status}")
                        return
                    async for chunk in resp.content.iter_chunked(4096):
                        yield chunk
            except Exception as e:
                logger.info(f"Stream client disconnected for {device_id}/{target}: {e}")

    return StreamingResponse(
        stream_generator(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )


@app.get("/proxy/snapshot/{device_id}/{target}")
async def proxy_snapshot(device_id: str, target: str):
    """Fetch a single snapshot image from edge."""
    edge = known_edges.get(device_id)
    if not edge:
        raise HTTPException(status_code=404, detail="Edge device not registered")

    snapshot_url = f"http://{edge['host']}:{edge['port']}/api/snapshot/{target}"
    async with aiohttp.ClientSession() as session:
        try:
            async with session.get(snapshot_url, timeout=aiohttp.ClientTimeout(total=3.0)) as resp:
                if resp.status == 200:
                    data = await resp.read()
                    return StreamingResponse(io_bytes_gen(data), media_type="image/jpeg")
        except Exception:
            pass
    raise HTTPException(status_code=502, detail="Edge snapshot unreachable")


async def io_bytes_gen(data: bytes):
    yield data


# ==============================================================================
# Recordings & Historical Frame Playback Endpoints
# ==============================================================================

@app.get("/api/recordings/sessions")
async def list_aggregated_sessions():
    """
    Queries all active edge devices for recorded sessions,
    clusters simultaneous recording runs across the fleet,
    and returns a clean list of playback sessions.
    """
    raw_sessions = []
    async with aiohttp.ClientSession() as session:
        for did, edge in list(known_edges.items()):
            if not edge.get("online"):
                continue
            url = f"http://{edge['host']}:{edge['port']}/api/sessions"
            try:
                async with session.get(url, timeout=aiohttp.ClientTimeout(total=2.0)) as resp:
                    if resp.status == 200:
                        data = await resp.json()
                        for s in data.get("sessions", []):
                            raw_sessions.append({
                                "device_id": did,
                                "device_name": edge.get("device_name", did),
                                "date": s.get("date"),
                                "time": s.get("time"),
                                "manifest": s.get("manifest", {})
                            })
            except Exception:
                pass

    clusters = []
    for item in raw_sessions:
        manifest = item["manifest"]
        date = item["date"]
        time_str = item["time"]
        label = manifest.get("label", "Recording Session")
        duration = manifest.get("duration_seconds", 0)
        frames_rec = manifest.get("frames_recorded", {})
        sensors = manifest.get("sensors", [])

        # Match with an existing cluster if date matches and (times match or duration & label match)
        matched_cluster = None
        for c in clusters:
            if c["date"] == date:
                if c["time"] == time_str:
                    matched_cluster = c
                    break
                if label and c["label"] == label and abs(c["duration_seconds"] - duration) <= 3.0:
                    matched_cluster = c
                    break

        if not matched_cluster:
            matched_cluster = {
                "id": f"session_{date}_{time_str}",
                "date": date,
                "time": time_str,
                "label": label,
                "duration_seconds": duration,
                "max_frames": 0,
                "devices": []
            }
            clusters.append(matched_cluster)

        mods = []
        for s in sensors:
            sid = s.get("id")
            smod = s.get("modality", "rgb")
            sname = s.get("name", sid)
            fcount = frames_rec.get(sid, 0) if isinstance(frames_rec, dict) else 0
            if fcount == 0 and isinstance(frames_rec, dict):
                fcount = frames_rec.get(smod, 0)
            mods.append({
                "id": sid,
                "modality": smod,
                "name": sname,
                "frames_count": fcount
            })
            matched_cluster["max_frames"] = max(matched_cluster["max_frames"], fcount)

        matched_cluster["devices"].append({
            "device_id": item["device_id"],
            "device_name": item["device_name"],
            "date": date,
            "time": time_str,
            "modalities": mods
        })

    clusters.sort(key=lambda c: (c["date"], c["time"]), reverse=True)
    return {"sessions": clusters}


@app.get("/api/recordings/{device_id}/{date}/{time_str}/{modality}/frames")
async def get_device_session_frames(device_id: str, date: str, time_str: str, modality: str):
    """Fetches the frame list and timestamps for a specific device, session, and modality."""
    cache_meta = os.path.join(CACHE_DIR, device_id, date, time_str, modality, "frames.json")
    if os.path.isfile(cache_meta):
        try:
            with open(cache_meta, "r") as f:
                cached_data = json.load(f)
            if cached_data.get("timestamps"):
                return cached_data
        except Exception:
            pass

    edge = known_edges.get(device_id)
    if not edge or not edge.get("online"):
        # Fallback to local cache directory if edge is offline
        disk_dir = os.path.join(CACHE_DIR, device_id, date, time_str, modality)
        if os.path.isdir(disk_dir):
            entries = sorted([e for e in os.scandir(disk_dir) if e.name.endswith(".jpg")], key=lambda e: e.name)
            if entries:
                ts = [round(e.stat().st_mtime, 4) for e in entries]
                return {
                    "date": date,
                    "time": time_str,
                    "modality": modality,
                    "total_frames": len(entries),
                    "frames": [e.name for e in entries],
                    "timestamps": ts,
                    "start_ts": ts[0],
                    "end_ts": ts[-1],
                    "duration": round(ts[-1] - ts[0], 3)
                }
        raise HTTPException(status_code=404, detail="Edge device not registered or offline")

    url = f"http://{edge['host']}:{edge['port']}/api/session/{date}/{time_str}/{modality}/frames"
    async with aiohttp.ClientSession() as session:
        try:
            async with session.get(url, timeout=aiohttp.ClientTimeout(total=4.0)) as resp:
                if resp.status == 200:
                    data = await resp.json()
                    # Cache to disk for offline playback
                    try:
                        os.makedirs(os.path.dirname(cache_meta), exist_ok=True)
                        with open(cache_meta, "w") as f:
                            json.dump(data, f)
                    except Exception:
                        pass
                    return data
                raise HTTPException(status_code=resp.status, detail="Error fetching session frames from edge")
        except HTTPException:
            raise
        except Exception as e:
            # Fallback to local cache files if network hit fails
            disk_dir = os.path.join(CACHE_DIR, device_id, date, time_str, modality)
            if os.path.isdir(disk_dir):
                entries = sorted([e for e in os.scandir(disk_dir) if e.name.endswith(".jpg")], key=lambda e: e.name)
                if entries:
                    ts = [round(e.stat().st_mtime, 4) for e in entries]
                    return {
                        "date": date,
                        "time": time_str,
                        "modality": modality,
                        "total_frames": len(entries),
                        "frames": [e.name for e in entries],
                        "timestamps": ts,
                        "start_ts": ts[0],
                        "end_ts": ts[-1],
                        "duration": round(ts[-1] - ts[0], 3)
                    }
            raise HTTPException(status_code=502, detail=f"Failed to reach edge device: {e}")


sync_status: Dict[str, Any] = {}

async def _cache_session_worker(session_id: str):
    """Background worker to download and cache all frames of a session to the Hub PC disk."""
    try:
        sessions_res = await list_aggregated_sessions()
        session = next((s for s in sessions_res.get("sessions", []) if s["id"] == session_id), None)
        if not session:
            sync_status[session_id] = {"in_progress": False, "error": "Session not found", "done": False}
            return

        total_frames = sum(
            mod.get("frames_count", 0)
            for dev in session.get("devices", [])
            for mod in dev.get("modalities", [])
        )
        sync_status[session_id] = {
            "session_id": session_id,
            "in_progress": True,
            "cached": 0,
            "total": total_frames,
            "percent": 0,
            "done": False,
            "error": None
        }

        cached_count = 0
        async with aiohttp.ClientSession() as client:
            for dev in session.get("devices", []):
                did = dev["device_id"]
                edge = known_edges.get(did)
                if not edge or not edge.get("online"):
                    continue
                date_s = dev["date"]
                time_s = dev["time"]
                for mod in dev.get("modalities", []):
                    mod_id = mod["id"]
                    fcount = mod.get("frames_count", 0)
                    try:
                        await get_device_session_frames(did, date_s, time_s, mod_id)
                    except Exception:
                        pass
                    for idx in range(1, fcount + 1):
                        fname = f"frame_{idx:06d}.jpg"
                        cpath = os.path.join(CACHE_DIR, did, date_s, time_s, mod_id, fname)
                        if os.path.isfile(cpath):
                            cached_count += 1
                        else:
                            furl = f"http://{edge['host']}:{edge['port']}/api/session/{date_s}/{time_s}/{mod_id}/frame/{idx}"
                            try:
                                async with client.get(furl, timeout=aiohttp.ClientTimeout(total=4.0)) as resp:
                                    if resp.status == 200:
                                        data = await resp.read()
                                        os.makedirs(os.path.dirname(cpath), exist_ok=True)
                                        with open(cpath, "wb") as f:
                                            f.write(data)
                                        cached_count += 1
                            except Exception:
                                pass

                        sync_status[session_id]["cached"] = cached_count
                        sync_status[session_id]["percent"] = round((cached_count / max(1, total_frames)) * 100, 1)

        sync_status[session_id]["in_progress"] = False
        sync_status[session_id]["done"] = True
    except Exception as e:
        logger.error(f"Error in cache worker for {session_id}: {e}")
        sync_status[session_id] = {"in_progress": False, "error": str(e), "done": False}


@app.post("/api/recordings/{session_id}/cache-all")
async def trigger_cache_session(session_id: str, background_tasks: BackgroundTasks):
    """Starts background sync to cache all frames of a session to the Hub PC disk."""
    st = sync_status.get(session_id)
    if st and st.get("in_progress"):
        return st
    background_tasks.add_task(_cache_session_worker, session_id)
    return {"status": "started", "session_id": session_id}


@app.get("/api/recordings/{session_id}/cache-status")
async def get_cache_status(session_id: str):
    """Returns the current background caching progress for a session."""
    return sync_status.get(session_id, {
        "session_id": session_id,
        "in_progress": False,
        "cached": 0,
        "total": 0,
        "percent": 0,
        "done": False
    })


@app.get("/proxy/recording/{device_id}/{date}/{time_str}/{modality}/{frame_id}")
async def proxy_recording_frame(device_id: str, date: str, time_str: str, modality: str, frame_id: str):
    """
    Proxies a recorded JPEG frame image from an edge device with disk-backed Hub caching.
    1. Checks if frame is cached on local SSD (0ms response, works offline).
    2. If missing, fetches from edge device, saves to disk cache, and returns it.
    """
    fname = f"frame_{int(frame_id):06d}.jpg" if frame_id.isdigit() else os.path.basename(frame_id)
    cache_path = os.path.join(CACHE_DIR, device_id, date, time_str, modality, fname)

    # 1. Serve immediately from local disk cache if available (<1ms, no network hit, works offline)
    if os.path.isfile(cache_path):
        return FileResponse(
            cache_path,
            media_type="image/jpeg",
            headers={
                "Cache-Control": "public, max-age=86400",
                "Access-Control-Allow-Origin": "*",
                "Access-Control-Allow-Private-Network": "true"
            }
        )

    # 2. Fetch from edge device and store in cache
    edge = known_edges.get(device_id)
    if not edge:
        raise HTTPException(status_code=404, detail="Edge device not registered and frame not in disk cache")

    frame_url = f"http://{edge['host']}:{edge['port']}/api/session/{date}/{time_str}/{modality}/frame/{frame_id}"
    async with aiohttp.ClientSession() as session:
        try:
            async with session.get(frame_url, timeout=aiohttp.ClientTimeout(total=5.0)) as resp:
                if resp.status == 200:
                    data = await resp.read()
                    try:
                        os.makedirs(os.path.dirname(cache_path), exist_ok=True)
                        with open(cache_path, "wb") as f:
                            f.write(data)
                    except Exception as ce:
                        logger.warning(f"Failed to write cache file: {ce}")

                    response = StreamingResponse(io_bytes_gen(data), media_type="image/jpeg")
                    response.headers["Cache-Control"] = "public, max-age=86400"
                    response.headers["Access-Control-Allow-Origin"] = "*"
                    response.headers["Access-Control-Allow-Private-Network"] = "true"
                    return response
        except Exception:
            pass

    raise HTTPException(status_code=502, detail="Recording frame unreachable")



# Static Files & Mobile Dashboard UI
os.makedirs(STATIC_DIR, exist_ok=True)
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")
if os.path.exists(os.path.join(STATIC_DIR, "_astro")):
    app.mount("/_astro", StaticFiles(directory=os.path.join(STATIC_DIR, "_astro")), name="astro")


@app.get("/config.json")
async def serve_config():
    cfg_file = os.path.join(STATIC_DIR, "config.json")
    if os.path.exists(cfg_file):
        return FileResponse(cfg_file, media_type="application/json")
    return JSONResponse({"default_hub_url": "https://marvel-reconstruction-atm-planning.trycloudflare.com"})


@app.get("/favicon.svg")
async def serve_favicon_svg():
    f = os.path.join(STATIC_DIR, "favicon.svg")
    if os.path.exists(f):
        return FileResponse(f, media_type="image/svg+xml")
    raise HTTPException(status_code=404)


@app.get("/favicon.ico")
async def serve_favicon_ico():
    f = os.path.join(STATIC_DIR, "favicon.ico")
    if os.path.exists(f):
        return FileResponse(f, media_type="image/x-icon")
    raise HTTPException(status_code=404)


@app.get("/", response_class=HTMLResponse)
async def serve_dashboard():
    index_path = os.path.join(STATIC_DIR, "index.html")
    if os.path.exists(index_path):
        with open(index_path, "r", encoding="utf-8") as f:
            return HTMLResponse(content=f.read())
    return HTMLResponse("<h2>Edge Collector Hub starting...</h2>")


if __name__ == "__main__":
    import uvicorn
    local_ip = get_local_ip()
    port = int(os.environ.get("PORT", 8000))
    logger.info("=" * 60)
    logger.info(" Multi-Sensor Edge Hub Server")
    logger.info(f" Local Web UI:    http://127.0.0.1:{port}")
    logger.info(f" Phone Access UI: http://{local_ip}:{port}")
    logger.info("=" * 60)
    uvicorn.run(app, host="0.0.0.0", port=port, access_log=False)
