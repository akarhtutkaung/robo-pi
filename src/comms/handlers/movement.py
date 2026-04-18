"""
TUTORIAL: src/comms/handlers/movement.py
==========================================
PURPOSE:
    Receives a parsed message dict and translates it into a controller call.
    This is the bridge between the WebSocket layer and the navigation layer.
    It knows about actions and speed but never touches hardware directly.

STEP 1 — Import what you need
    from src.comms.protocol import parse_message, build_response
    from src.navigation.controller import RobotController

STEP 2 — Write an async handle(websocket, raw, controller) function
    This function is called by the WebSocket server for every incoming message.
    Parameters:
        websocket  — the active WebSocket connection (to send responses back)
        raw        — the raw string message received
        controller — the shared RobotController instance

    Example skeleton:
        async def handle(websocket, raw: str, controller: RobotController):
            ...

STEP 3 — Parse the message inside handle()
    Wrap parse_message(raw) in a try/except ValueError.
    If it fails, send an error response back and return early.

        try:
            msg = parse_message(raw)
        except ValueError as e:
            await websocket.send(build_response("error", str(e)))
            return

STEP 4 — Route the action to the correct controller method
    Use the "action" field from the parsed message to call the right method.

        action = msg["action"]
        speed  = msg["speed"]

        if action == "forward":
            controller.forward(speed)
        elif action == "backward":
            controller.backward(speed)
        elif action == "left":
            controller.steer("left")
        elif action == "right":
            controller.steer("right")
        elif action == "stop":
            controller.stop()

STEP 5 — Send an acknowledgement back to the Mac
    After executing, let the Mac know the command was received and run.

        await websocket.send(build_response("ok", f"executed: {action}"))

STEP 6 — Add logging (optional but helpful while testing)
    Use Python's built-in logging module to print what command was received.

        import logging
        logger = logging.getLogger(__name__)
        logger.info(f"Received action={action} speed={speed}")

NOTE:
    The RobotController instance is created once in remote.py and passed in
    here — this handler must not create its own controller instance.
"""
