"""
Manual control loop — owns the WebSocket recv loop and all idle-timeout logic
while the robot is in manual mode.

Returns the requested mode string (e.g. "autonomous") when a mode-switch
message is received, so the caller (websocket_server.py) can transition.
"""

import asyncio
import json
from src.comms.handlers.dispatch import handle as dispatch_handle

IDLE_TIMEOUT = 0.3  # seconds before an action is considered stale


async def run_manual(websocket, controller) -> str:
    idle_tasks: dict[str, asyncio.Task] = {}
    last_values: dict[str, object] = {}
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

                if msg_type == "mode":
                    cancel_all()
                    return data.get("action", "manual")

                elif msg_type == "movement":
                    if action == "steer":
                        angle = int(data.get("angle", 90))
                        last_values[key] = angle
                        if not websocket.messages and applied.get(key) != angle:
                            controller.steer(angle)
                            applied[key] = angle
                        reset_idle(key)

                    elif action == "throttle":
                        speed = abs(int(data.get("speed", 0)))
                        if data.get("direction") == "backward":
                            speed = -speed
                        last_values[key] = speed
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
                    asyncio.create_task(dispatch_handle(websocket, raw, controller))

            except asyncio.TimeoutError:
                cancel_all()
                controller.center_steering()
                if not controller.is_stopped():
                    await controller.smooth_stop()

    finally:
        cancel_all()