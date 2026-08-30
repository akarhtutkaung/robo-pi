"""
Vision protocol — camera pan/tilt, future: gesture, object detection stream.
Message fields:
    {"action": "move", "axis": "<x|y>", "angle": <+|-int>}
    {"action": "center"}
"""
import json

VALID_ACTIONS = {"move", "center"}

def parse_message(raw: str) -> dict:
    try:
        data = json.loads(raw)
    except json.JSONDecodeError as e:
        raise ValueError(f"Invalid JSON: {e}")

    action = data.get("action")
    if action not in VALID_ACTIONS:
        raise ValueError(f"Unknown vision action: {action!r}")

    if action != "center":
        return {"action": action, "axis": data.get("axis"), "angle": int(data.get("angle", 90))}
    return {"action": "center"}