"""
Wires everything together for remote (WebSocket-driven) mode.
This is the top-level setup for the current use case:
    Other devices → WebSocket → Pi → motors + servos

Creates the RobotController, starts the WebSocket server, and handles
clean shutdown on Ctrl+C or SIGTERM.
"""

import asyncio
import signal
from src.comms.websocket_server import start_server
from src.navigation.controller import RobotController

def run():
    controller = RobotController()
    cleaned_up = False

    def on_shutdown(sig, frame):
        nonlocal cleaned_up
        if not cleaned_up:
            cleaned_up = True
            controller.cleanup()
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, on_shutdown)

    try:
        asyncio.run(start_server(controller))
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        if not cleaned_up:
            cleaned_up = True
            controller.cleanup()
        print("Robot stopped safely.")