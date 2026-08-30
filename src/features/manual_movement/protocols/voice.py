"""
Voice protocol — speech command parsing. Placeholder for sherpa-ncnn integration.
Message fields:
    {"action": "command", "text": "<recognized speech>"}
"""
import json

VALID_ACTIONS = {"command"}

def parse_message(raw: str) -> dict:
    try:
        data = json.loads(raw)
    except json.JSONDecodeError as e:
        raise ValueError(f"Invalid JSON: {e}")

    action = data.get("action")
    if action not in VALID_ACTIONS:
        raise ValueError(f"Unknown voice action: {action!r}")

    return {"action": "command", "text": str(data.get("text", ""))}
