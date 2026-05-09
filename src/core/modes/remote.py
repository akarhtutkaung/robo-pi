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
from src.perception.camera import make_camera, CameraSwitch
from src.core.config import CAMERA_CFG

def run():
    controller = RobotController()

    fc = CAMERA_CFG["front"]
    bc = CAMERA_CFG["back"]
    front_camera = make_camera(fc["index"], fc["main_width"], fc["main_height"],
                               fc["lores_width"], fc["lores_height"], fc["framerate"])
    back_camera  = make_camera(bc["index"], bc["main_width"], bc["main_height"],
                               bc["lores_width"], bc["lores_height"], bc["framerate"])
    cameras = CameraSwitch(front_camera, back_camera)

    cleaned_up = False

    def on_shutdown(sig, frame):
        nonlocal cleaned_up
        if not cleaned_up:
            cleaned_up = True
            controller.cleanup()
            cameras.stop()
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, on_shutdown)
    try:
        async def _run_all():
            await asyncio.gather(
                start_server(controller, cameras),
                start_webrtc_server(cameras),
            )
        asyncio.run(_run_all())
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        if not cleaned_up:
            cleaned_up = True
            controller.cleanup()
            cameras.stop()
        print("Robot stopped safely.")