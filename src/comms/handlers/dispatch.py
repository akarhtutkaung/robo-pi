"""
Central dispatcher — routes incoming messages to the correct domain handler
based on the "type" field. Add new handlers here as new capabilities are built.

Message format:
    {"type": "movement", "action": "throttle", "direction": "forward", "speed": 50}
    {"type": "movement", "action": "steer", "angle": 72}
    {"type": "vision",   "action": "camera-x", "angle": 90}
    {"type": "vision",   "action": "camera-y", "angle": 90}
    {"type": "vision",   "action": "gesture",  ...}  # future
    {"type": "voice",    "action": "command",   ...}  # future
"""

import json
import websockets
from src.comms.handlers import movement, vision
from src.comms.protocols.base import build_response

HANDLERS = {
    "movement": movement.handle,
    "vision":   vision.handle,
}

async def handle(websocket, raw: str, controller):
    try:
        data = json.loads(raw)
    except json.JSONDecodeError:
        await websocket.send(build_response("error", "Invalid JSON"))
        return

    handler_type = data.get("type")
    handler = HANDLERS.get(handler_type)

    if handler is None:
        await websocket.send(build_response("error", f"Unknown type: {handler_type!r}"))
        return

    try:
        await handler(websocket, raw, controller)
    except websockets.exceptions.ConnectionClosed:
        pass