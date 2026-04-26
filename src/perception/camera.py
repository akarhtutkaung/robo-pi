"""
Camera capture as an aiortc VideoStreamTrack.
This track is added to an RTCPeerConnection in webrtc_server.py so the
Pi's camera stream is sent to the remote client over WebRTC.

Dependencies: aiortc, av, picamera2
"""

from aiortc import VideoStreamTrack
from av import VideoFrame
from picamera2 import Picamera2
import numpy as np
from src.core import config

class CameraVideoTrack(VideoStreamTrack):
    kind = "video"

    def __init__(self, width=640, height=480):
        super().__init__()
        self._camera = Picamera2()
        self._camera.create_video_configuration(
            main={"size": (width, height), "format": "YUV420"}
        )
        self._camera.configure(config)
        self._camera.start()

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        arr = self._camera.capture_array()
        frame = VideoFrame.from_ndarray(arr, format="yuv420p")
        frame.pts = pts
        frame.time_base = time_base
        return frame

    def stop(self):
        super().stop()
        self._camera.stop()