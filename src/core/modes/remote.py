"""
Wires everything together for remote (WebSocket-driven) mode.

Two servers run concurrently:
    Port 8765 — control WebSocket  (movement, servos)
    Port 8766 — WebRTC signaling WS (camera stream negotiation)

Creates the RobotController, starts both servers, and handles
clean shutdown on Ctrl+C or SIGTERM.
"""

import asyncio
import signal
from src.comms.websocket_server import start_server
from src.navigation.controller import RobotController
from src.comms.webrtc_server import start_webrtc_server

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
        async def _run_all():
            await asyncio.gather(
                start_server(controller),
                start_webrtc_server(),
            )
        asyncio.run(_run_all())
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        if not cleaned_up:
            cleaned_up = True
            controller.cleanup()
        print("Robot stopped safely.")