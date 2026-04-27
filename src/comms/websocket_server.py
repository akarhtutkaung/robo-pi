"""
Runs a WebSocket server on the Pi that other devices connect to.
Receives messages, passes each one to the movement handler.
Stays alive as long as the program runs.
"""

import asyncio
import json
import websockets
from src.comms.handlers.dispatch import handle as dispatch_handle
from src.core.config import WS_CFG

IDLE_TIMEOUT = 0.3  # seconds before an action is considered stale


async def on_connect(websocket, controller):
    print(f"Client connected: {websocket.remote_address}")

    # One idle task per action key — reset on every message, fires when silent
    idle_tasks: dict[str, asyncio.Task] = {}
    # Latest desired value per action key (may not yet be written to hardware)
    last_values: dict[str, object] = {}
    # Last value actually written to hardware per action key — value-based dedup
    applied: dict[str, object] = {}

    async def action_idle(key: str):
        await asyncio.sleep(IDLE_TIMEOUT)
        idle_tasks.pop(key, None)
        action = key.split(":", 1)[-1]
        if action == "throttle":
            last_values.pop(key, None)
            applied.pop(key, None)
            if not controller.is_stopped():
                await controller.smooth_stop()
        elif action == "steer":
            # flush any desired angle that was skipped while buffer was non-empty
            desired = last_values.pop(key, None)
            was_applied = applied.pop(key, None)
            if desired is not None and desired != was_applied:
                controller.steer(desired)
            controller.center_steering()

    def reset_idle(key: str):
        old = idle_tasks.pop(key, None)
        if old and not old.done():
            old.cancel()
        idle_tasks[key] = asyncio.create_task(action_idle(key))

    def cancel_all():
        for task in list(idle_tasks.values()):
            if not task.done():
                task.cancel()
        idle_tasks.clear()
        last_values.clear()
        applied.clear()

    try:
        while True:
            try:
                raw = await asyncio.wait_for(websocket.recv(), timeout=IDLE_TIMEOUT)
                print(f"Received message: {raw}")

                try:
                    data = json.loads(raw)
                except json.JSONDecodeError:
                    continue

                msg_type = data.get("type", "movement")
                action = data.get("action", "")
                key = f"{msg_type}:{action}"

                if msg_type == "movement":
                    if action == "steer":
                        angle = int(data.get("angle", 90))
                        last_values[key] = angle  # always record latest desired
                        # latest-wins: skip I2C write if more messages are buffered
                        if not websocket.messages and applied.get(key) != angle:
                            controller.steer(angle)
                            applied[key] = angle
                        reset_idle(key)

                    elif action == "throttle":
                        speed = abs(int(data.get("speed", 0)))
                        if data.get("direction") == "backward":
                            speed = -speed
                        last_values[key] = speed  # always record latest desired
                        # latest-wins: skip hardware write if more messages are buffered
                        if not websocket.messages and applied.get(key) != speed:
                            if speed != 0:
                                controller.setSpeed(speed)
                            else:
                                asyncio.create_task(controller.smooth_stop())
                            applied[key] = speed
                        reset_idle(key)

                    elif action == "stop":
                        asyncio.create_task(controller.smooth_stop())
                        controller.center_steering()
                        for k in ("movement:steer", "movement:throttle"):
                            t = idle_tasks.pop(k, None)
                            if t and not t.done():
                                t.cancel()
                            last_values.pop(k, None)
                            applied.pop(k, None)

                else:
                    # Vision, voice, future types — dispatch to domain handler
                    asyncio.create_task(dispatch_handle(websocket, raw, controller))

            except asyncio.TimeoutError:
                cancel_all()
                controller.center_steering()
                if not controller.is_stopped():
                    await controller.smooth_stop()

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        cancel_all()
        controller.center_steering()
        print(f"Client disconnected: {websocket.remote_address}")
        print("[!] Stopping robot due to disconnection...")
        await controller.smooth_stop()
        print("[!] Robot stopped.")


async def start_server(controller):
    host = WS_CFG["host"]
    port = WS_CFG["port"]

    async with websockets.serve(
        lambda ws: on_connect(ws, controller),
        host,
        port
    ):
        print(f"WebSocket server listening on ws://{host}:{port}")
        await asyncio.Future()  # keeps the server running forever
