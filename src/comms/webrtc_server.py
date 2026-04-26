"""
WebRTC signaling server — runs on a separate WebSocket port from the
control server (8765). The client connects here to negotiate a WebRTC
peer connection; the actual camera video travels peer-to-peer over WebRTC
after signaling completes.

Signaling flow (vanilla ICE — Pi waits for full ICE gathering):
    1. Client  → Pi   : {"type": "offer",  "sdp": "<SDP string>"}
    2. Pi creates RTCPeerConnection, adds CameraVideoTrack
    3. Pi generates answer, waits until ICE gathering is complete
    4. Pi    → Client : {"type": "answer", "sdp": "<SDP with candidates>"}
    (Client does not need to send ICE candidates separately in this model.)

Dependencies: aiortc, websockets
"""

import asyncio
import json
import websockets
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.sdp import candidate_from_sdp
from src.core.config import WEBRTC_CFG
from src.perception.camera import CameraVideoTrack
from src.perception.vision.stream import configure_h264

async def on_signaling(websocket):
    pc = RTCPeerConnection()
    track = CameraVideoTrack()
    pc.addTrack(track)
    configure_h264(pc)   # force H.264 — must be called before setRemoteDescription

    ice_done = asyncio.Event()
    @pc.on("icegatheringstatechange")
    def _on_gathering():
        if pc.iceGatheringState == "complete":
            ice_done.set()

    try:
        async for raw in websocket:
            msg = json.loads(raw)
            if msg["type"] == "offer":
                offer = RTCSessionDescription(sdp=msg["sdp"], type="offer")
                await pc.setRemoteDescription(offer)
                answer = await pc.createAnswer()
                await pc.setLocalDescription(answer)
                await ice_done.wait()
                response = {"type": "answer", "sdp": pc.localDescription.sdp}
                await websocket.send(json.dumps(response))
            elif msg["type"] == "ice-candidate":
                c = msg.get("candidate", {})
                raw_sdp = c.get("candidate", "")
                if raw_sdp:
                    if raw_sdp.startswith("candidate:"):
                        raw_sdp = raw_sdp[len("candidate:"):]
                    candidate = candidate_from_sdp(raw_sdp)
                    candidate.sdpMid = c.get("sdpMid")
                    candidate.sdpMLineIndex = c.get("sdpMLineIndex")
                    await pc.addIceCandidate(candidate)
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        track.stop()
        await pc.close()

async def start_webrtc_server():
    host = WEBRTC_CFG["host"]
    port = WEBRTC_CFG["port"]

    async with websockets.serve(on_signaling, host, port):
        print(f"WebRTC signaling server listening on ws://{host}:{port}")
        await asyncio.Future()  # keeps the server running forever
