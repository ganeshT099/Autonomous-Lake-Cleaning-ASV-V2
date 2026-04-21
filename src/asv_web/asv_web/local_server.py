"""
ASV Local Server  —  runs on Jetson, port 8080
────────────────────────────────────────────────
• Full WebSocket relay  (device / client / auto_pilot / health)
• Serves built frontend at http://<jetson-ip>:8080
• Cloud bridge: bidirectional tunnel to Render relay when internet available
  → Local browser  http://192.168.0.50:8080  (direct, zero-latency)
  → Remote browser https://asv-server-1.onrender.com  (tunnelled via bridge)
"""

import asyncio, json, os
from typing import Optional, List

import uvicorn, websockets
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

CLOUD = "wss://asv-server-1.onrender.com"
PORT  = 8080
DIST  = os.path.expanduser("~/asv_web_dist")

app = FastAPI()
app.add_middleware(CORSMiddleware, allow_origins=["*"],
                   allow_methods=["*"], allow_headers=["*"])

# ── Shared state ──────────────────────────────────────────────────────────────
device:     Optional[WebSocket] = None
auto_pilot: Optional[WebSocket] = None
clients:    List[WebSocket]     = []
_mission: dict = {}
_health:  dict = {}

# Single merged queue: items are (str | bytes) forwarded to cloud device bridge
_q_cloud_dev:  asyncio.Queue = asyncio.Queue(maxsize=80)
# Auto-pilot status → cloud
_q_cloud_auto: asyncio.Queue = asyncio.Queue(maxsize=50)
# Local client mission commands → cloud mission_worker
_q_cloud_cmd:  asyncio.Queue = asyncio.Queue(maxsize=20)


async def _broadcast_text(msg: str):
    dead = []
    for c in clients:
        try:    await c.send_text(msg)
        except: dead.append(c)
    for c in dead:
        if c in clients: clients.remove(c)


async def _broadcast_bytes(data: bytes):
    dead = []
    for c in clients:
        try:    await c.send_bytes(data)
        except: dead.append(c)
    for c in dead:
        if c in clients: clients.remove(c)


# ── /ws/device  (ws_bridge connects here) ────────────────────────────────────
@app.websocket("/ws/device")
async def ws_device(ws: WebSocket):
    global device
    await ws.accept()
    device = ws
    print("🚤 Device connected", flush=True)
    try:
        while True:
            msg = await ws.receive()
            if msg.get("type") == "websocket.disconnect":
                break
            text = msg.get("text")
            data = msg.get("bytes")

            if text is not None:
                try:
                    mtype = json.loads(text).get("type", "")
                except Exception:
                    mtype = ""
                if mtype == "ping":
                    await ws.send_text(json.dumps({"type": "pong"}))
                    continue
                await _broadcast_text(text)
                try: _q_cloud_dev.put_nowait(text)
                except asyncio.QueueFull: pass

            elif data is not None:
                await _broadcast_bytes(data)
                # For binary camera frames: keep only the latest — drop oldest
                while _q_cloud_dev.full():
                    try: _q_cloud_dev.get_nowait()
                    except: break
                try: _q_cloud_dev.put_nowait(data)
                except: pass

    except Exception as e:
        print(f"❌ Device: {e}", flush=True)
    finally:
        device = None
        print("⚠️  Device disconnected", flush=True)


# ── /ws/client  (browser connects here) ──────────────────────────────────────
@app.websocket("/ws/client")
async def ws_client(ws: WebSocket):
    await ws.accept()
    clients.append(ws)
    print(f"🌐 Client connected ({len(clients)})", flush=True)
    if _mission:
        try: await ws.send_text(json.dumps({"type": "mission", **_mission}))
        except: pass
    if _health:
        try: await ws.send_text(json.dumps(_health))
        except: pass
    try:
        while True:
            msg = await ws.receive()
            if msg.get("type") == "websocket.disconnect":
                break
            text = msg.get("text")
            if not text:
                continue
            try:
                parsed = json.loads(text)
                mtype  = parsed.get("type", "")
            except Exception:
                mtype, parsed = "", {}

            if mtype == "ping":
                await ws.send_text(json.dumps({"type": "pong", "ts": parsed.get("ts")}))
                continue

            if mtype in ("start_autonomous", "stop_autonomous"):
                if auto_pilot:
                    try: await auto_pilot.send_text(text)
                    except: pass
                else:
                    await ws.send_text(json.dumps({
                        "type": "autonomous_status", "status": "error",
                        "log": "Autonomous pilot not connected", "level": "error",
                    }))
                continue

            # Mission planning → forward to cloud mission_worker via bridge
            if mtype in ("mission_request", "regenerate_path",
                         "start_simulation", "stop_simulation"):
                try: _q_cloud_cmd.put_nowait(text)
                except asyncio.QueueFull: pass
                continue

            # Joystick / manual → local device
            if device:
                try: await device.send_text(text)
                except: pass

    except WebSocketDisconnect:
        pass
    except Exception as e:
        print(f"❌ Client: {e}", flush=True)
    finally:
        if ws in clients: clients.remove(ws)
        print(f"🔌 Client removed ({len(clients)} left)", flush=True)


# ── /ws/auto_pilot  (autonomous_pilot connects here) ─────────────────────────
@app.websocket("/ws/auto_pilot")
async def ws_auto_pilot(ws: WebSocket):
    global auto_pilot
    await ws.accept()
    auto_pilot = ws
    print("🤖 Auto-pilot connected", flush=True)
    try:
        while True:
            msg = await ws.receive()
            if msg.get("type") == "websocket.disconnect":
                break
            text = msg.get("text")
            if text:
                await _broadcast_text(text)
                try: _q_cloud_auto.put_nowait(text)
                except asyncio.QueueFull: pass
    except Exception as e:
        print(f"❌ Auto-pilot: {e}", flush=True)
    finally:
        auto_pilot = None
        print("🤖 Auto-pilot disconnected", flush=True)


# ── /ws/health ────────────────────────────────────────────────────────────────
@app.websocket("/ws/health")
async def ws_health(ws: WebSocket):
    global _health
    await ws.accept()
    try:
        while True:
            msg = await ws.receive()
            if msg.get("type") == "websocket.disconnect":
                break
            text = msg.get("text")
            if text:
                try: _health = json.loads(text)
                except: continue
                await _broadcast_text(text)
    except Exception as e:
        print(f"❌ Health: {e}", flush=True)


# ── /api/status ───────────────────────────────────────────────────────────────
@app.get("/api/status")
def api_status():
    return {"status": "ASV local server",
            "device": device is not None,
            "auto_pilot": auto_pilot is not None,
            "clients": len(clients)}


# ── Cloud bridge: device (GPS/camera → Render → remote browsers) ──────────────
async def _bridge_device():
    while True:
        try:
            async with websockets.connect(
                    f"{CLOUD}/ws/device", ping_interval=20,
                    open_timeout=15) as ws:
                print("[cloud] Device bridge UP", flush=True)

                async def _send():
                    while True:
                        item = await _q_cloud_dev.get()
                        await ws.send(item)

                async def _recv():
                    # Joystick / manual commands from remote browsers
                    async for raw in ws:
                        if isinstance(raw, bytes):
                            continue
                        try:    parsed = json.loads(raw)
                        except: continue
                        mtype = parsed.get("type", "")
                        if mtype == "joy" and device:
                            try: await device.send_text(raw)
                            except: pass

                await asyncio.gather(_send(), _recv())

        except Exception as e:
            print(f"[cloud] Device bridge DOWN: {e} — retry 10s", flush=True)
            await asyncio.sleep(10)


# ── Cloud bridge: auto_pilot (status ↑  /  start-stop ↓) ─────────────────────
async def _bridge_auto():
    while True:
        try:
            async with websockets.connect(
                    f"{CLOUD}/ws/auto_pilot", ping_interval=20,
                    open_timeout=15) as ws:
                print("[cloud] Auto-pilot bridge UP", flush=True)

                async def _send():
                    while True:
                        text = await _q_cloud_auto.get()
                        await ws.send(text)

                async def _recv():
                    # start/stop commands from remote browsers
                    async for raw in ws:
                        if auto_pilot:
                            try: await auto_pilot.send_text(raw)
                            except: pass

                await asyncio.gather(_send(), _recv())

        except Exception as e:
            print(f"[cloud] Auto-pilot bridge DOWN: {e} — retry 10s", flush=True)
            await asyncio.sleep(10)


# ── Cloud bridge: client (mission results from cloud mission_worker) ──────────
async def _bridge_client():
    global _mission
    while True:
        try:
            async with websockets.connect(
                    f"{CLOUD}/ws/client", ping_interval=20,
                    open_timeout=15) as ws:
                print("[cloud] Client bridge UP", flush=True)

                async def _send_cmds():
                    while True:
                        cmd = await _q_cloud_cmd.get()
                        await ws.send(cmd)

                async def _recv_results():
                    async for raw in ws:
                        if not isinstance(raw, str):
                            continue
                        try:    parsed = json.loads(raw)
                        except: continue
                        mtype = parsed.get("type", "")
                        # Only forward mission results to local clients
                        # (GPS/telemetry already arrives via ws_bridge directly)
                        if mtype in ("mission", "mission_status",
                                     "mission_worker_status"):
                            if mtype == "mission":
                                _mission = {k: v for k, v in parsed.items()
                                            if k != "type"}
                            await _broadcast_text(raw)

                await asyncio.gather(_send_cmds(), _recv_results())

        except Exception as e:
            print(f"[cloud] Client bridge DOWN: {e} — retry 15s", flush=True)
            await asyncio.sleep(15)


# ── Startup ───────────────────────────────────────────────────────────────────
@app.on_event("startup")
async def startup():
    asyncio.create_task(_bridge_device())
    asyncio.create_task(_bridge_auto())
    asyncio.create_task(_bridge_client())
    print(f"🚀 ASV local server on port {PORT}", flush=True)
    if os.path.isdir(DIST):
        print(f"📦 Serving frontend from {DIST}", flush=True)
    else:
        print(f"⚠️  No frontend at {DIST} — UI not served locally", flush=True)


# ── Serve built frontend ──────────────────────────────────────────────────────
if os.path.isdir(DIST):
    app.mount("/", StaticFiles(directory=DIST, html=True), name="static")


def main():
    uvicorn.run("asv_web.local_server:app",
                host="0.0.0.0", port=PORT,
                log_level="warning", workers=1)


if __name__ == "__main__":
    main()
