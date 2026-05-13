"""
MJPEG debug stream server — streams a combined free-space + YOLO overlay
while the robot is running. Open http://<pi-ip>:8080 in a browser.

Overlay (bottom to top, drawn in order):
  1. Free-space passability bars and ROI box   (draw_debug)
  2. YOLO bounding boxes with label/confidence (draw_detections)

Shares the existing CameraSwitch with the main system — no second camera
instance is created. Frame capture and processing runs in a thread-pool
executor so the asyncio event loop is not blocked.

Enabled/disabled and port are set in config/hardware.yaml under debug_stream.
"""

import asyncio
import threading
import time
import cv2
from http.server import BaseHTTPRequestHandler, HTTPServer

from src.perception.camera import capture_bgr
from src.perception.vision.free_space import detect, draw_debug
from src.perception.vision.object_detection import (
    detect_obstacles, draw_detections, STOP_CM, TURN_CM,
)

_JPEG_QUALITY = 75


def _capture_and_encode(camera, obstacle) -> bytes:
    frame       = capture_bgr(camera)
    error, conf = detect(frame)
    vis         = draw_debug(frame, error, conf)   # free-space overlay (returns copy)
    detections  = detect_obstacles(frame)
    draw_detections(vis, detections)               # YOLO boxes on top (in-place)

    dist_cm = obstacle.distance_cm()
    if dist_cm <= STOP_CM:
        colour = (0, 0, 255)    # red   — blocked
    elif dist_cm <= TURN_CM:
        colour = (0, 165, 255)  # orange — approaching
    else:
        colour = (0, 255, 0)    # green  — clear
    cv2.putText(vis, f"{dist_cm:.1f} cm",
                (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 1, cv2.LINE_AA)

    _, jpg = cv2.imencode(".jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, _JPEG_QUALITY])
    return jpg.tobytes()


def _make_handler(shared: dict, lock: threading.Lock, fps: int):
    interval = 1.0 / fps

    class _Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with lock:
                        jpg = shared["jpg"]
                    if jpg is None:
                        time.sleep(0.1)
                        continue
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                        + jpg + b"\r\n"
                    )
                    time.sleep(interval)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *_):
            pass

    return _Handler


async def run_debug_stream(camera, obstacle, port: int = 8080, fps: int = 10):
    """Async task — starts the MJPEG HTTP server and continuously updates
    the frame buffer. Runs until cancelled alongside the other servers.
    """
    shared: dict = {"jpg": None}
    lock   = threading.Lock()

    server = HTTPServer(("0.0.0.0", port), _make_handler(shared, lock, fps))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"Debug stream: http://<pi-ip>:{port}")

    loop = asyncio.get_running_loop()
    sleep = 1.0 / fps
    try:
        while True:
            jpg = await loop.run_in_executor(None, _capture_and_encode, camera, obstacle)
            with lock:
                shared["jpg"] = jpg
            await asyncio.sleep(sleep)
    except asyncio.CancelledError:
        raise  # daemon thread exits with the process — no explicit shutdown needed
