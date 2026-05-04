"""
Camera capture as an aiortc VideoStreamTrack.
This track is added to an RTCPeerConnection in webrtc_server.py so the
Pi's camera stream is sent to the remote client over WebRTC.

The Picamera2 instance is created and owned externally (webrtc_server.py)
so the device is acquired once and shared across reconnections.

Dependencies: aiortc, av, picamera2
"""

from aiortc import VideoStreamTrack
from av import VideoFrame
from picamera2 import Picamera2
import numpy as np


def make_camera(width=640, height=480) -> Picamera2:
    """Create, configure, and start a Picamera2 instance. Call once at startup."""
    camera = Picamera2()
    cfg = camera.create_video_configuration(
        main={"size": (width, height), "format": "YUV420"}
    )
    camera.configure(cfg)
    camera.start()
    return camera


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
