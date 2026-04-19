"""
Receives a parsed message dict and translates it into a controller call.
This is the bridge between the WebSocket layer and the navigation layer.
It knows about actions and speed but never touches hardware directly.
"""

from src.comms.protocol import parse_message, build_response
from src.navigation.controller import RobotController

async def handle(websocket, raw: str, controller: RobotController):
    try:
        msg = parse_message(raw)
    except ValueError as e:
        await websocket.send(build_response("error", str(e)))
        return

    action = msg["action"]
    speed  = msg.get("speed", 0)

    if action == "throttle":
        if speed != 0:
            controller.setSpeed(speed)
        else:
            await controller.smooth_stop()
    elif action == "steer":
        angle = msg.get("angle", 90)
        controller.steer(angle)
    elif action == "stop":
        await controller.smooth_stop()

    await websocket.send(build_response("ok", f"executed: {action}"))