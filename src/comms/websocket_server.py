"""
Runs a WebSocket server on the Pi that other devices connect to.
Receives messages, passes each one to the movement handler.
Stays alive as long as the program runs.
"""

import asyncio
import websockets
from src.comms.handlers.dispatch import handle
from src.core.config import WS_CFG

IDLE_TIMEOUT = 0.3  # seconds before stopping if no message received

async def on_connect(websocket, controller):
    print(f"Client connected: {websocket.remote_address}")
    current_task: asyncio.Task | None = None

    def cancel_current():
        nonlocal current_task
        if current_task and not current_task.done():
            current_task.cancel()

    try:
        while True:
            try:
                raw_message = await asyncio.wait_for(websocket.recv(), timeout=IDLE_TIMEOUT)
                print(f"Received message: {raw_message}")
                cancel_current()
                current_task = asyncio.create_task(handle(websocket, raw_message, controller))
            except asyncio.TimeoutError:
                if not controller.is_stopped():
                    cancel_current()
                    current_task = asyncio.create_task(controller.smooth_stop())
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        cancel_current()
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

