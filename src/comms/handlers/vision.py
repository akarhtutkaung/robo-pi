"""
Handles camera pan/tilt commands routed from dispatch under type "vision".
Bridge between the WebSocket layer and camera controls on the controller.
"""
from src.comms.protocols.vision import parse_message
from src.comms.protocols.base import build_response
from src.navigation.controller import RobotController

async def handle(websocket, raw: str, controller: RobotController):
    try:
        msg = parse_message(raw)
    except ValueError as e:
        await websocket.send(build_response("error", str(e)))
        return

    action = msg["action"]
    angle = msg.get("angle", 90)

    if action == "camera-x":
        controller.move_camera_x(angle)
    elif action == "camera-y":
        controller.move_camera_y(angle)
    elif action == "center":
        controller.center_camera()

    await websocket.send(build_response("ok", f"executed: {action}"))
