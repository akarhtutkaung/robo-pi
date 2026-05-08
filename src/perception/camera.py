"""
Camera capture as an aiortc VideoStreamTrack.
This track is added to an RTCPeerConnection in webrtc_server.py so the
Pi's camera stream is sent to the remote client over WebRTC.

The Picamera2 instance is created and owned externally (remote.py) so the
device is acquired once and shared across WebRTC streaming and autonomous
vision — both call capture_array() on the same instance concurrently.

Dependencies: aiortc, av, picamera2, opencv-python
"""

import cv2
from aiortc import VideoStreamTrack
from av import VideoFrame
from picamera2 import Picamera2
import numpy as np


def make_camera(width=640, height=480, lores_width=320, lores_height=240) -> Picamera2:
    """Create, configure, and start a Picamera2 instance. Call once at startup.

    Two streams are configured:
      main  (640×480 YUV420) — fed to CameraVideoTrack for WebRTC
      lores (320×240 YUV420) — consumed by capture_bgr() for OpenCV
    """
    camera = Picamera2()
    cfg = camera.create_video_configuration(
        main={"size": (width, height), "format": "YUV420"},
        lores={"size": (lores_width, lores_height), "format": "YUV420"},
    )
    camera.configure(cfg)
    camera.start()
    return camera


def capture_bgr(camera: Picamera2) -> np.ndarray:
    """Return a 320×240 BGR frame from the lores stream for OpenCV processing."""
    yuv = camera.capture_array("lores")
    return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)


class CameraVideoTrack(VideoStreamTrack):
    kind = "video"

    def __init__(self, camera: Picamera2):
        super().__init__()
        self._camera = camera

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        arr = self._camera.capture_array()
        frame = VideoFrame.from_ndarray(arr, format="yuv420p")
        frame.pts = pts
        frame.time_base = time_base
        return frame
