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

app = FastAPI(title="Edge Multi-Sensor Collector Hub", version="1.0.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

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
    try:
        async with session.get(url, timeout=aiohttp.ClientTimeout(total=2.0)) as resp:
            if resp.status == 200:
                data = await resp.json()
                edge_data["online"] = True
                edge_data["last_seen"] = time.time()
                edge_data["device_name"] = data.get("device_name", edge_data.get("device_name"))
                edge_data["sensors"] = data.get("sensors", [])
                edge_data["modalities"] = data.get("modalities", [])
                edge_data["collection"] = data.get("collection", {})
                return
    except Exception:
        pass
    edge_data["online"] = False


async def background_poll_edges():
    """Periodically queries all edge devices for status and telemetry."""
    while True:
        try:
            async with aiohttp.ClientSession() as session:
                tasks = [
                    ping_edge(did, edata, session)
                    for did, edata in known_edges.items()
                ]
                if tasks:
                    await asyncio.gather(*tasks)
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
    return {
        "server_ip": ip,
        "server_port": 8000,
        "mobile_url": f"http://{ip}:8000",
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


# Static Files & Mobile Dashboard UI
os.makedirs(STATIC_DIR, exist_ok=True)
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


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
