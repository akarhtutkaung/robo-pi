"""
H.264 codec configuration for WebRTC streaming.

aiortc negotiates VP8 by default. This module forces H.264 on the
RTCPeerConnection so the Pi's hardware-encoded H.264 frames are sent
directly without a software re-encode step.

Usage (in webrtc_server.py, after pc.addTrack()):
    from src.perception.vision.stream import configure_h264

    pc.addTrack(track)
    configure_h264(pc)       ← call this before setRemoteDescription
"""

from aiortc import RTCPeerConnection, RTCRtpSender

def configure_h264(pc: RTCPeerConnection) -> None:
    caps = RTCRtpSender.getCapabilities("video")
    h264 = [c for c in caps.codecs if c.mimeType == "video/H264"]
    for transceiver in pc.getTransceivers():
        if transceiver.kind == "video":
            transceiver.setCodecPreferences(h264)