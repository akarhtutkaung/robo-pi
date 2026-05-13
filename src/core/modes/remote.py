"""
Wires everything together for remote (WebSocket-driven) mode.

Servers run concurrently:
    Port 8765 — control WebSocket  (movement, servos)
    Port 8766 — WebRTC signaling WS (camera stream negotiation)
    Port 8080 — MJPEG debug stream  (free-space overlay, if enabled in hardware.yaml)

Creates the RobotController, starts all servers, and handles
clean shutdown on Ctrl+C or SIGTERM.
"""

import asyncio
import signal
from src.comms.websocket_server import start_server
from src.navigation.controller import RobotController
from src.comms.webrtc_server import start_webrtc_server
from src.comms.debug_stream_server import run_debug_stream
from src.perception.camera import make_camera, CameraSwitch
from src.perception.vision.object_detection import ObstacleDetector
from src.core.config import CAMERA_CFG, DEBUG_STREAM_CFG

def run():
    controller = RobotController()
    obstacle   = ObstacleDetector()

    fc = CAMERA_CFG["front"]
    bc = CAMERA_CFG["back"]
    front_camera = make_camera(fc["index"], fc["main_width"], fc["main_height"],
                               fc["lores_width"], fc["lores_height"], fc["framerate"],
                               fc.get("rotate_180", False))
    back_camera  = make_camera(bc["index"], bc["main_width"], bc["main_height"],
                               bc["lores_width"], bc["lores_height"], bc["framerate"],
                               bc.get("rotate_180", False))
    cameras = CameraSwitch(front_camera, back_camera)

    cleaned_up = False

    def on_shutdown(sig, frame):
        nonlocal cleaned_up
        if not cleaned_up:
            cleaned_up = True
            controller.cleanup()
            obstacle.cleanup()
            cameras.stop()
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, on_shutdown)
    try:
        async def _run_all():
            tasks = [
                start_server(controller, cameras, obstacle),
                start_webrtc_server(cameras, fc["main_width"], fc["main_height"]),
            ]
            if DEBUG_STREAM_CFG.get("enabled", False):
                tasks.append(run_debug_stream(
                    cameras,
                    obstacle,
                    port=DEBUG_STREAM_CFG.get("port", 8080),
                    fps=DEBUG_STREAM_CFG.get("fps", 10),
                ))
            await asyncio.gather(*tasks)
        asyncio.run(_run_all())
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        if not cleaned_up:
            cleaned_up = True
            controller.cleanup()
            obstacle.cleanup()
            cameras.stop()
        print("Robot stopped safely.")