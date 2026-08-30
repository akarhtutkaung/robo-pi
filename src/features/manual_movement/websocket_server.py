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
import websockets
from src.components.core.config import WS_CFG
from src.components.comms.base import build_response
from src.features.manual_movement.manual import run_manual
from src.features.autonomous_movement.autonomous import run_autonomous

_RECV_TIMEOUT = 0.3  # poll interval while waiting for mode-switch in autonomous mode

async def on_connect(websocket, controller, camera, obstacle):
    print(f"Client connected: {websocket.remote_address}")

    current_mode = "manual"
    autonomous_task: asyncio.Task | None = None

    try:
        while True:
            if current_mode == "manual":
                # run_manual owns the recv loop; returns the requested mode on switch
                requested = await run_manual(websocket, controller, camera)

                if requested == "autonomous":
                    current_mode = "autonomous"
                    controller.center_steering()
                    if not controller.is_stopped():
                        asyncio.create_task(controller.smooth_stop())
                    autonomous_task = asyncio.create_task(
                        run_autonomous(controller, obstacle, camera, websocket)
                    )
                    print("[mode] Switched to autonomous")

            else:  # autonomous — watch for a switch back to manual, or the loop dying on its own
                if autonomous_task.done():
                    # run_autonomous exited on its own (wedged, or too many consecutive
                    # navigation failures) — retrieve the exception so asyncio doesn't
                    # log "Task exception was never retrieved", tell the client, and
                    # fall back to manual instead of sitting halted and unresponsive.
                    exc = autonomous_task.exception()
                    if exc is not None:
                        print(f"[mode] Autonomous loop exited: {exc}")
                        try:
                            await websocket.send(build_response("error", f"autonomous halted: {exc}"))
                        except websockets.exceptions.ConnectionClosed:
                            pass
                    current_mode = "manual"
                    autonomous_task = None
                    controller.center_steering()
                    continue

                try:
                    raw = await asyncio.wait_for(websocket.recv(), timeout=_RECV_TIMEOUT)
                    data = json.loads(raw)
                    if data.get("type") == "mode" and data.get("action") == "manual":
                        current_mode = "manual"
                        if autonomous_task and not autonomous_task.done():
                            autonomous_task.cancel()
                        autonomous_task = None
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
        controller.center_steering()
        print(f"Client disconnected: {websocket.remote_address}")
        print("[!] Stopping robot due to disconnection...")
        await controller.smooth_stop()
        print("[!] Robot stopped.")


async def start_server(controller, camera, obstacle):
    host = WS_CFG["host"]
    port = WS_CFG["port"]

    async with websockets.serve(
        lambda ws: on_connect(ws, controller, camera, obstacle),
        host,
        port
    ) as server:
        print(f"WebSocket server listening on ws://{host}:{port}")
        try:
            await asyncio.Future()
        except asyncio.CancelledError:
            server.close()
            await server.wait_closed()
            raise