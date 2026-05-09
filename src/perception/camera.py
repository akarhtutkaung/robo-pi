"""
Camera capture as an aiortc VideoStreamTrack.

Two cameras are supported:
  front — Pi Camera V3 Wide Angle (CSI port 0) — main 1920×1080, lores 640×480
  back  — rear camera            (CSI port 1) — main 640×480,   lores 320×240

CameraSwitch holds both instances and exposes the active one for streaming
and OpenCV capture. Call use_back() before reversing and use_front() otherwise.

The Picamera2 instances are created and owned externally (remote.py) so the
devices are acquired once and shared across WebRTC streaming and autonomous vision.

Dependencies: aiortc, av, picamera2, opencv-python
"""

import cv2
import numpy as np
from aiortc import VideoStreamTrack
from av import VideoFrame
from picamera2 import Picamera2


def make_camera(index: int, width: int, height: int,
                lores_width: int, lores_height: int,
                framerate: float = 30.0) -> Picamera2:
    """Create, configure, and start a Picamera2 instance on the given CSI index."""
    camera = Picamera2(index)
    cfg = camera.create_video_configuration(
        main={"size": (width, height), "format": "YUV420"},
        lores={"size": (lores_width, lores_height), "format": "YUV420"},
        controls={"FrameRate": framerate},
    )
    camera.configure(cfg)
    camera.start()
    return camera


class CameraSwitch:
    """Holds front and back cameras; exposes the active one for capture and streaming.

    Call use_back() before the rover reverses and use_front() when going forward
    or stopped — both the WebRTC stream and OpenCV vision will follow automatically.
    """

    def __init__(self, front: Picamera2, back: Picamera2):
        self._front = front
        self._back = back
        self._active = front

    def use_front(self):
        self._active = self._front

    def use_back(self):
        self._active = self._back

    def capture_array(self, name: str = "main") -> np.ndarray:
        return self._active.capture_array(name)

    def stop(self):
        self._front.stop()
        self._back.stop()


def capture_bgr(camera) -> np.ndarray:
    """Return a BGR frame from the lores stream for OpenCV processing.

    Accepts either a Picamera2 instance or a CameraSwitch.
    """
    yuv = camera.capture_array("lores")
    return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)


class CameraVideoTrack(VideoStreamTrack):
    """aiortc video track that streams from whichever camera is currently active."""

    kind = "video"

    def __init__(self, camera):
        super().__init__()
        self._camera = camera  # Picamera2 or CameraSwitch

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        arr = self._camera.capture_array()
        frame = VideoFrame.from_ndarray(arr, format="yuv420p")
        frame.pts = pts
        frame.time_base = time_base
        return frame
