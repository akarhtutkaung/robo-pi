"""
Protocol for WebSocket communication between the client and the robot.
Defines the message format, parsing logic, and response building.
Message Format:
    All messages are JSON strings with at least an "action" field.
    Depending on the action, other fields may be required.
Examples:
    {"action": "throttle", "direction": "forward", "speed": 50}
    {"action": "steer", "angle": 30}
    {"action": "stop"}
"""
import json

VALID_ACTIONS = {"throttle", "steer", "camera-x", "camera-y", "stop"}
VALID_DIRECTIONS = {"forward", "backward"}

def parse_message(raw: str) -> dict:
    try:
        data = json.loads(raw)
    except json.JSONDecodeError as e:
        raise ValueError(f"Invalid JSON: {e}")

    action = data.get("action")
    if action not in VALID_ACTIONS:
        print(f"Received unknown action: {action!r}")
        raise ValueError(f"Unknown action: {action!r}")

    if action == "throttle":
        direction = data.get("direction")
        if direction not in VALID_DIRECTIONS:
            print(f"Received unknown direction: {direction!r}")
            raise ValueError(f"Unknown direction: {direction!r}")
        speed = abs(int(data.get("speed", 50))) 
        if direction == "backward":
            speed = -speed
        return {
            "action": "throttle",
            "speed": speed,
        }

    if action == "steer":
        angle = int(data.get("angle", 90)) 
        return {"action": "steer", "angle": angle}

    if action == "camera-x":
        angle = int(data.get("angle", 90))
        print("Parsed camera-x angle: %d", angle)
        return {"action": "camera-x", "angle": angle}

    if action == "camera-y":
        angle = int(data.get("angle", 90))
        return {"action": "camera-y", "angle": angle}

    return {"action": "stop"}

def build_response(status: str, message: str = "") -> str:
    return json.dumps({"status": status, "message": message})