"""
Movement protocol — throttle, steer, stop.
Message fields:
    {"action": "throttle", "direction": "forward"|"backward", "speed": 0-100}
    {"action": "steer", "angle": <int>}
    {"action": "stop"}
"""
import json

VALID_ACTIONS = {"throttle", "steer", "stop"}
VALID_DIRECTIONS = {"forward", "backward"}

def parse_message(raw: str) -> dict:
    try:
        data = json.loads(raw)
    except json.JSONDecodeError as e:
        raise ValueError(f"Invalid JSON: {e}")

    action = data.get("action")
    if action not in VALID_ACTIONS:
        raise ValueError(f"Unknown movement action: {action!r}")

    if action == "throttle":
        direction = data.get("direction")
        if direction not in VALID_DIRECTIONS:
            raise ValueError(f"Unknown direction: {direction!r}")
        speed = abs(int(data.get("speed", 50)))
        if direction == "backward":
            speed = -speed
        return {"action": "throttle", "speed": speed}

    if action == "steer":
        return {"action": "steer", "angle": int(data.get("angle", 90))}

    return {"action": "stop"}
