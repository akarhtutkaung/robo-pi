"""
TUTORIAL: src/comms/protocol.py
=================================
PURPOSE:
    Define and validate the message format exchanged between the Mac and the Pi.
    Centralizing this means if the format changes you update one file, not every handler.

STEP 1 — Decide on a message format
    Use JSON — it is human-readable and easy to parse on both Python sides.
    Agree on this structure with your Mac-side code:

        {
            "action": "forward",   # Required. See VALID_ACTIONS below.
            "speed": 50            # Optional. Integer 0–100. Default 50 if omitted.
        }

    VALID_ACTIONS:
        "forward"   — move forward
        "backward"  — move backward
        "left"      — steer left
        "right"     — steer right
        "stop"      — stop all movement

STEP 2 — Write a parse_message(raw) function
    raw is a string received from the WebSocket.
    The function should:
      a) Use json.loads(raw) to parse it.
      b) Check that "action" key exists.
      c) Check that the action value is one of VALID_ACTIONS.
      d) Extract speed (default to 50 if not present).
      e) Return a clean dict: {"action": ..., "speed": ...}
      f) Raise a ValueError with a clear message if validation fails.

    Example:
        import json

        VALID_ACTIONS = {"forward", "backward", "left", "right", "stop"}

        def parse_message(raw: str) -> dict:
            try:
                data = json.loads(raw)
            except json.JSONDecodeError as e:
                raise ValueError(f"Invalid JSON: {e}")

            action = data.get("action")
            if action not in VALID_ACTIONS:
                raise ValueError(f"Unknown action: {action!r}")

            return {
                "action": action,
                "speed": int(data.get("speed", 50))
            }

STEP 3 — Optionally write a build_response(status, message) function
    For sending acknowledgement back to the Mac after each command.

        def build_response(status: str, message: str = "") -> str:
            return json.dumps({"status": status, "message": message})

    Usage examples:
        build_response("ok")
        build_response("error", "Unknown action: spin")

NOTE:
    The WebSocket handler (src/comms/handlers/movement.py) calls parse_message()
    on every incoming message. If ValueError is raised, the handler should
    send an error response back to the Mac and skip the command.
"""
