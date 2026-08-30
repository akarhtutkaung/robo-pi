"""
WebSocket server — thin mode dispatcher.

Delegates all manual control logic to run_manual() and non-manual modes to
the coroutine factories in _MODE_RUNNERS. Handles mode-switch messages while
in a non-manual mode and cleans up on disconnect.

Mode switching:
    {"type": "mode", "action": "autonomous"}      — hand off to the autonomous drive loop
    {"type": "mode", "action": "facial_tracking"} — hand off to the facial tracking loop
    {"type": "mode", "action": "manual"}          — cancel the active mode task, return to manual
"""

import asyncio
import json
import logging
import websockets
from src.components.core.config import WS_CFG
from src.components.comms.base import build_response
from src.features.manual_movement.manual import run_manual
from src.features.autonomous_movement.autonomous import run_autonomous
from src.features.facial_tracking.tracker import run_facial_tracking

log = logging.getLogger(__name__)

_RECV_TIMEOUT = 0.3  # poll interval while waiting for mode-switch in a non-manual mode

# Each factory takes (controller, obstacle, camera, websocket) so every mode task is
# created the same way, even though a given mode may not need every argument.
_MODE_RUNNERS = {
    "autonomous": run_autonomous,
    "facial_tracking": lambda controller, _obstacle, camera, websocket: run_facial_tracking(
        controller, camera, websocket
    ),
}

async def on_connect(websocket, controller, camera, obstacle):
    log.info("Client connected: %s", websocket.remote_address)

    current_mode = "manual"
    mode_task: asyncio.Task | None = None

    try:
        while True:
            if current_mode == "manual":
                # run_manual owns the recv loop; returns the requested mode on switch
                requested = await run_manual(websocket, controller, camera)

                runner = _MODE_RUNNERS.get(requested)
                if runner is not None:
                    current_mode = requested
                    controller.center_steering()
                    if not controller.is_stopped():
                        asyncio.create_task(controller.smooth_stop())
                    mode_task = asyncio.create_task(runner(controller, obstacle, camera, websocket))
                    log.info("[mode] Switched to %s", current_mode)

            else:  # non-manual mode — watch for a switch back to manual, or the loop dying on its own
                if mode_task.done():
                    # The mode loop exited on its own (e.g. autonomous wedged, or too many
                    # consecutive failures) — retrieve the exception so asyncio doesn't log
                    # "Task exception was never retrieved", tell the client, and fall back
                    # to manual instead of sitting halted and unresponsive.
                    exc = mode_task.exception()
                    if exc is not None:
                        log.error("[mode] %s loop exited: %s", current_mode, exc, exc_info=exc)
                        try:
                            await websocket.send(build_response("error", f"{current_mode} halted: {exc}"))
                        except websockets.exceptions.ConnectionClosed:
                            pass
                    current_mode = "manual"
                    mode_task = None
                    controller.center_steering()
                    continue

                try:
                    raw = await asyncio.wait_for(websocket.recv(), timeout=_RECV_TIMEOUT)
                    data = json.loads(raw)
                    if data.get("type") == "mode" and data.get("action") == "manual":
                        current_mode = "manual"
                        if mode_task and not mode_task.done():
                            mode_task.cancel()
                        mode_task = None
                        controller.center_steering()
                        asyncio.create_task(controller.smooth_stop())
                        log.info("[mode] Switched to manual")
                except (asyncio.TimeoutError, json.JSONDecodeError):
                    pass

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        if mode_task and not mode_task.done():
            mode_task.cancel()
        controller.center_steering()
        log.info("Client disconnected: %s", websocket.remote_address)
        log.info("[!] Stopping robot due to disconnection...")
        await controller.smooth_stop()
        log.info("[!] Robot stopped.")


async def start_server(controller, camera, obstacle):
    host = WS_CFG["host"]
    port = WS_CFG["port"]

    async with websockets.serve(
        lambda ws: on_connect(ws, controller, camera, obstacle),
        host,
        port
    ) as server:
        log.info("WebSocket server listening on ws://%s:%s", host, port)
        try:
            await asyncio.Future()
        except asyncio.CancelledError:
            server.close()
            await server.wait_closed()
            raise