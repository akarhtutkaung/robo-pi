"""
Runs a WebSocket server on the Pi that other devices connect to.
Receives messages, passes each one to the movement handler.
Stays alive as long as the program runs.
"""

import asyncio
import websockets
from src.comms.handlers.movement import handle
from src.core.config import WS_CFG

async def on_connect(websocket, controller):
    print(f"Client connected: {websocket.remote_address}")
    try:
        async for raw_message in websocket:
            print(f"Received message: {raw_message}")
            await handle(websocket, raw_message, controller)
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        controller.stop()  # ensure robot stops if client disconnects
        print(f"Client disconnected: {websocket.remote_address}")

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

