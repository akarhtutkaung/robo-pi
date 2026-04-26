"""
Handles drive commands routed from dispatch under type "movement".
Bridge between the WebSocket layer and the navigation layer.
Never touches hardware directly.
"""
from src.comms.protocols.movement import parse_message
from src.comms.protocols.base import build_response
from src.navigation.controller import RobotController

async def handle(websocket, raw: str, controller: RobotController):
    try:
        msg = parse_message(raw)
    except ValueError as e:
        await websocket.send(build_response("error", str(e)))
        return

    action = msg["action"]

    if action == "throttle":
        speed = msg.get("speed", 0)
        if speed != 0:
            controller.setSpeed(speed)
        else:
            await controller.smooth_stop()
    elif action == "steer":
        controller.steer(msg.get("angle", 90))
    elif action == "stop":
        await controller.smooth_stop()

    await websocket.send(build_response("ok", f"executed: {action}"))
