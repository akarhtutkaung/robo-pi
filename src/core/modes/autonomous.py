"""
Autonomous drive loop — runs while the robot is in autonomous mode.
Triggered by a {"type": "mode", "action": "autonomous"} WebSocket message.

Sends drive_state messages to the client each tick so the UI can display
current phase, steering direction, error, and confidence without any
overlay on the video stream.

drive_state message format:
    {
        "type":       "drive_state",
        "phase":      "clear" | "approaching" | "blocked" | "avoiding",
        "direction":  "CENTER" | "SLIGHT LEFT" | "LEFT" | "HARD LEFT"
                      | "SLIGHT RIGHT" | "RIGHT" | "HARD RIGHT",
        "error":      float [-1, 1],   # negative = left, positive = right
        "confidence": float [0, 1]
    }
"""

import asyncio
import json
from src.perception.vision.free_space import detect, MIN_CONFIDENCE
from src.perception.camera import capture_bgr
from src.core.config import MOTOR_CFG, AUTONOMOUS_CFG, SERVO_CFG

AUTONOMOUS_SPEED    = AUTONOMOUS_CFG["speed"]
REVERSE_SPEED       = AUTONOMOUS_CFG["reverse_speed"]
APPROACH_SPEED      = AUTONOMOUS_CFG["approach_speed"]
_CM_PER_SPEED_UNIT  = MOTOR_CFG["rear"]["cm_per_speed_unit"]
_STOP_TARGET_MARGIN = 5.0  # cm

_STEER_RIGHT      = SERVO_CFG["servo0"]["max_angle"]    # 50  — full right
_STEER_LEFT       = SERVO_CFG["servo0"]["min_angle"]    # 140 — full left
_CENTER_ANGLE     = SERVO_CFG["servo0"]["center_angle"] # 94.68
_STEER_HALF_RANGE = min(
    _CENTER_ANGLE - _STEER_RIGHT,
    _STEER_LEFT   - _CENTER_ANGLE,
)

_DIRECTION_THRESHOLDS = [
    (0.08, "CENTER"),
    (0.25, "SLIGHT"),
    (0.55, ""),
    (2.0,  "HARD"),
]


def _direction_label(error: float) -> str:
    a = abs(error)
    for threshold, prefix in _DIRECTION_THRESHOLDS:
        if a < threshold:
            if prefix == "CENTER":
                return "CENTER"
            side = "LEFT" if error < 0 else "RIGHT"
            return f"{prefix} {side}".strip() if prefix else side
    return "LEFT" if error < 0 else "RIGHT"


async def _send(websocket, phase: str, error: float = 0.0, confidence: float = 0.0):
    try:
        await websocket.send(json.dumps({
            "type":       "drive_state",
            "phase":      phase,
            "direction":  _direction_label(error),
            "error":      round(error, 3),
            "confidence": round(confidence, 2),
        }))
    except Exception:
        pass  # client may have disconnected


async def setup(controller):
    controller.center_camera()


async def navigate_step(controller, obstacle, camera, websocket):
    if obstacle.is_blocked():
        distance = obstacle.distance_cm()

        if obstacle.is_sudden_stop():
            print(f"Sudden obstacle at {distance:.1f} cm — hard stop.")
            controller.force_stop()
        else:
            v = controller.current_speed
            d_target = distance - _STOP_TARGET_MARGIN
            if d_target > 0 and abs(v) > 0.1:
                required_rate = (v ** 2) * _CM_PER_SPEED_UNIT / (2.0 * d_target)
                required_rate = max(required_rate, MOTOR_CFG["rear"]["decelerate_rate"])
            else:
                required_rate = None
            print(f"Obstacle at {distance:.1f} cm — smooth stop (rate={required_rate}).")
            await controller.smooth_stop(rate=required_rate)

        error, conf = detect(capture_bgr(camera))
        if conf >= MIN_CONFIDENCE and error > 0:
            print(f"Camera: free space right (err={error:+.2f}) — turning right")
            steer_angle, opposite_angle = _STEER_RIGHT, _STEER_LEFT
        elif conf >= MIN_CONFIDENCE and error < 0:
            print(f"Camera: free space left (err={error:+.2f}) — turning left")
            steer_angle, opposite_angle = _STEER_LEFT, _STEER_RIGHT
        else:
            print(f"Camera: low confidence ({conf:.2f}) — defaulting right")
            error = 0.5  # report as slight-right since that's the default
            steer_angle, opposite_angle = _STEER_RIGHT, _STEER_LEFT

        await _send(websocket, "avoiding", error, conf)

        # K-turn: steer → back → opposite steer → forward → centre
        controller.steer(steer_angle)
        await asyncio.sleep(0.3)
        controller.backward(REVERSE_SPEED)
        await asyncio.sleep(1.5)
        await controller.smooth_stop()
        controller.steer(opposite_angle)
        await asyncio.sleep(0.3)
        controller.forward(AUTONOMOUS_SPEED)
        await asyncio.sleep(1.0)
        controller.steer_center()
        await asyncio.sleep(0.5)
        await controller.smooth_stop()

    elif obstacle.should_turn():
        await _send(websocket, "approaching", 0.0, 0.0)
        controller.forward(APPROACH_SPEED)

    else:
        error, conf = detect(capture_bgr(camera))
        if conf >= MIN_CONFIDENCE:
            steer_angle = round(_CENTER_ANGLE - error * _STEER_HALF_RANGE)
            controller.steer(int(steer_angle))
        else:
            error = 0.0
            controller.steer_center()
        controller.forward(AUTONOMOUS_SPEED)
        await _send(websocket, "clear", error, conf)

    await asyncio.sleep(0.1)


async def run_autonomous(controller, obstacle, camera, websocket):
    await setup(controller)
    try:
        while True:
            await navigate_step(controller, obstacle, camera, websocket)
    except asyncio.CancelledError:
        await controller.smooth_stop()
