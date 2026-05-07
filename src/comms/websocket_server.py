"""
WebSocket server — thin mode dispatcher.

Delegates all manual control logic to run_manual() and autonomous drive to
run_autonomous(). Handles mode-switch messages while in autonomous mode and
cleans up on disconnect.

Mode switching:
    {"type": "mode", "action": "autonomous"} — hand off to autonomous loop
    {"type": "mode", "action": "manual"}     — cancel autonomous, return to manual
"""

import asyncio
import json
from src.perception.vision.object_detection import ObstacleDetector
import websockets
from src.core.config import WS_CFG
from src.core.modes.manual import run_manual
from src.core.modes.autonomous import run_autonomous

_RECV_TIMEOUT = 0.3  # poll interval while waiting for mode-switch in autonomous mode

async def on_connect(websocket, controller):
    print(f"Client connected: {websocket.remote_address}")

    current_mode = "manual"
    autonomous_task: asyncio.Task | None = None
    obstacle: ObstacleDetector | None = None

    try:
        while True:
            if current_mode == "manual":
                # run_manual owns the recv loop; returns the requested mode on switch
                requested = await run_manual(websocket, controller)

                if requested == "autonomous":
                    current_mode = "autonomous"
                    controller.center_steering()
                    if not controller.is_stopped():
                        asyncio.create_task(controller.smooth_stop())
                    obstacle = ObstacleDetector()
                    autonomous_task = asyncio.create_task(run_autonomous(controller, obstacle))
                    print("[mode] Switched to autonomous")

            else:  # autonomous — only watch for a switch back to manual
                try:
                    raw = await asyncio.wait_for(websocket.recv(), timeout=_RECV_TIMEOUT)
                    data = json.loads(raw)
                    if data.get("type") == "mode" and data.get("action") == "manual":
                        current_mode = "manual"
                        if autonomous_task and not autonomous_task.done():
                            autonomous_task.cancel()
                        autonomous_task = None
                        if obstacle is not None:
                            obstacle.cleanup()
                            obstacle = None
                        controller.center_steering()
                        asyncio.create_task(controller.smooth_stop())
                        print("[mode] Switched to manual")
                except (asyncio.TimeoutError, json.JSONDecodeError):
                    pass

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        if autonomous_task and not autonomous_task.done():
            autonomous_task.cancel()
        if obstacle is not None:
            obstacle.cleanup()
        controller.center_steering()
        print(f"Client disconnected: {websocket.remote_address}")
        print("[!] Stopping robot due to disconnection...")
        await controller.smooth_stop()
        print("[!] Robot stopped.")


async def start_server(controller):
    host = WS_CFG["host"]
    port = WS_CFG["port"]

    async with websockets.serve(
        lambda ws: on_connect(ws, controller),
        host,
        port
    ):
        print(f"WebSocket server listening on ws://{host}:{port}")
        await asyncio.Future()  # keeps the server running forever