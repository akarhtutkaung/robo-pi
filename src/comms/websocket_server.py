"""
TUTORIAL: src/comms/websocket_server.py
=========================================
PURPOSE:
    Runs a WebSocket server on the Pi that the Mac connects to.
    Receives messages, passes each one to the movement handler.
    Stays alive as long as the program runs.

STEP 1 — Install the websockets library (on the Pi)
    pip install websockets

STEP 2 — Import what you need
    import asyncio
    import websockets
    from src.comms.handlers.movement import handle
    from src.core.config import WS_CFG

STEP 3 — Write an async connection handler: on_connect(websocket)
    This function is called once per client connection (your Mac).
    It should loop forever, reading messages and calling handle().

    Example:
        async def on_connect(websocket, controller):
            print(f"Client connected: {websocket.remote_address}")
            try:
                async for raw_message in websocket:
                    await handle(websocket, raw_message, controller)
            except websockets.exceptions.ConnectionClosed:
                print("Client disconnected")

    The `async for raw_message in websocket` loop automatically handles
    waiting for the next message and exits cleanly when the Mac disconnects.

STEP 4 — Write a start_server(controller) coroutine
    This is the function that remote.py will call to launch the server.
    Use websockets.serve() to bind to host and port from WS_CFG.

    Example:
        async def start_server(controller):
            host = WS_CFG["host"]
            port = WS_CFG["port"]

            # Use a lambda to pass `controller` into on_connect,
            # because websockets.serve() only passes websocket automatically.
            async with websockets.serve(
                lambda ws: on_connect(ws, controller),
                host,
                port
            ):
                print(f"WebSocket server listening on ws://{host}:{port}")
                await asyncio.Future()  # keeps the server running forever

STEP 5 — Find the Pi's IP address
    The Mac needs to know the Pi's local IP to connect.
    On the Pi, run:  hostname -I
    Then on the Mac, connect to:  ws://<pi-ip>:8765

STEP 6 — Test with a simple Mac-side script before wiring up OpenCV
    On the Mac, open a Python terminal and try:

        import asyncio, websockets, json

        async def test():
            async with websockets.connect("ws://<pi-ip>:8765") as ws:
                await ws.send(json.dumps({"action": "forward", "speed": 50}))
                response = await ws.recv()
                print(response)

        asyncio.run(test())

NOTE:
    This server handles one client at a time (your Mac).
    If you need multiple concurrent clients in the future, use
    asyncio.gather() to fan out, but that is not needed now.
"""
