"""
Runs a WebSocket server on the Pi that other devices connect to.
Receives messages, passes each one to the movement handler.
Stays alive as long as the program runs.
"""

import asyncio
import json
import websockets
from src.comms.handlers.dispatch import handle
from src.core.config import WS_CFG

IDLE_TIMEOUT = 0.3  # seconds before an action is considered stale


def _action_key(raw: str) -> str:
    """Extract a stable cancellation key from a raw message without full parsing."""
    try:
        data = json.loads(raw)
        return f"{data.get('type', 'unknown')}:{data.get('action', 'unknown')}"
    except (json.JSONDecodeError, AttributeError):
        return "unknown:unknown"


async def on_connect(websocket, controller):
    print(f"Client connected: {websocket.remote_address}")

    # Per-action tracking: action_key -> (handler_task, timeout_task)
    active: dict[str, tuple[asyncio.Task, asyncio.Task]] = {}

    def cancel_action(key: str):
        pair = active.pop(key, None)
        if pair:
            for task in pair:
                if not task.done():
                    task.cancel()

    def cancel_all():
        for key in list(active.keys()):
            cancel_action(key)

    async def action_idle(key: str, handler: asyncio.Task):
        """Fires IDLE_TIMEOUT seconds after the last message for this action key."""
        await asyncio.sleep(IDLE_TIMEOUT)
        if not handler.done():
            handler.cancel()
        active.pop(key, None)
        action = key.split(":", 1)[-1]
        if action == "throttle" and not controller.is_stopped():
            await controller.smooth_stop()
        elif action == "steer":
            controller.center_steering()  # recenter steering servo

    try:
        while True:
            try:
                raw = await asyncio.wait_for(websocket.recv(), timeout=IDLE_TIMEOUT)
                print(f"Received message: {raw}")

                key = _action_key(raw)
                cancel_action(key)  # cancel previous task for this action only

                handler = asyncio.create_task(handle(websocket, raw, controller))
                watcher = asyncio.create_task(action_idle(key, handler))
                active[key] = (handler, watcher)

            except asyncio.TimeoutError:
                # No messages at all — cancel everything and stop
                cancel_all()
                if not controller.is_stopped():
                    await controller.smooth_stop()

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        cancel_all()
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
