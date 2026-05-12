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
import logging

from src.perception.vision.free_space import detect, MIN_CONFIDENCE
from src.perception.vision.object_detection import (
    detect_obstacles, select_primary_obstacle, classify_width_threat,
    sweep_obstacle, calculate_real_width,
)
from src.perception.camera import capture_bgr
from src.core.config import MOTOR_CFG, AUTONOMOUS_CFG, SERVO_CFG, OBSTACLE_AVOIDANCE_CFG

log = logging.getLogger(__name__)

_FOCAL_LENGTH_PX    = OBSTACLE_AVOIDANCE_CFG["focal_length_px"]

AUTONOMOUS_SPEED    = AUTONOMOUS_CFG["speed"]
REVERSE_SPEED       = AUTONOMOUS_CFG["reverse_speed"]
APPROACH_SPEED      = AUTONOMOUS_CFG["approach_speed"]
_CM_PER_SPEED_UNIT  = MOTOR_CFG["rear"]["cm_per_speed_unit"]
_STOP_TARGET_MARGIN = 5.0   # cm — target stop distance in front of obstacle
_MIN_DECEL_DIST_CM  = 1.0   # lower bound on d_target; prevents ÷0 in decel formula

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

_ROBOT_WIDTH_CM  = OBSTACLE_AVOIDANCE_CFG["robot_width_cm"]
_CLEARANCE_CM    = OBSTACLE_AVOIDANCE_CFG["clearance_buffer_cm"]
_MIN_PASS_GAP_CM = _ROBOT_WIDTH_CM + _CLEARANCE_CM

_FRAME_W     = 640   # lores stream width fed to YOLO and free-space
_MIN_SPEED   = 0.1   # below this throttle the robot is considered stopped
_LOOP_PERIOD = 0.1   # target seconds per navigation tick


# ---------------------------------------------------------------------------
# Pure helpers
# ---------------------------------------------------------------------------

def _direction_label(error: float) -> str:
    a = abs(error)
    for threshold, prefix in _DIRECTION_THRESHOLDS:
        if a < threshold:
            if prefix == "CENTER":
                return "CENTER"
            side = "LEFT" if error < 0 else "RIGHT"
            return f"{prefix} {side}".strip() if prefix else side
    return "LEFT" if error < 0 else "RIGHT"


def decide_avoidance(width_threat: str, sweep: dict) -> str:
    """Return the avoidance maneuver to execute given threat class and sweep data.

    width_threat — "WIDE" | "MEDIUM" | "NARROW"  (from classify_width_threat)
    sweep        — {"left": cm, "center": cm, "right": cm}  (from sweep_obstacle)

    Returns "TURN_LEFT" | "TURN_RIGHT" | "REVERSE_AND_TURN"
    """
    if width_threat == "WIDE":
        return "REVERSE_AND_TURN"

    left_cm   = sweep["left"]
    right_cm  = sweep["right"]
    best_side = "TURN_LEFT" if left_cm >= right_cm else "TURN_RIGHT"
    best_cm   = max(left_cm, right_cm)

    if width_threat == "NARROW":
        return best_side

    # MEDIUM — only attempt to pass if the winning side has enough clearance
    return best_side if best_cm >= _MIN_PASS_GAP_CM else "REVERSE_AND_TURN"


# ---------------------------------------------------------------------------
# WebSocket helper
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# Avoidance maneuvers
# ---------------------------------------------------------------------------

async def execute_avoidance(controller, camera, decision: str):
    """Execute a steering maneuver based on the avoidance decision.

    "TURN_LEFT"        — steer left → forward 0.8 s → centre
    "TURN_RIGHT"       — steer right → forward 0.8 s → centre
    "REVERSE_AND_TURN" — K-turn: steer → back → opposite steer → forward → centre
    All variants end with smooth_stop() + steer_center().
    """
    if decision == "TURN_LEFT":
        controller.steer(_STEER_LEFT)
        controller.forward(AUTONOMOUS_SPEED)
        await asyncio.sleep(0.8)
        await controller.smooth_stop()
        controller.steer_center()

    elif decision == "TURN_RIGHT":
        controller.steer(_STEER_RIGHT)
        controller.forward(AUTONOMOUS_SPEED)
        await asyncio.sleep(0.8)
        await controller.smooth_stop()
        controller.steer_center()

    else:  # REVERSE_AND_TURN — pick turn direction from free-space signal
        loop = asyncio.get_running_loop()
        try:
            frame       = await loop.run_in_executor(None, capture_bgr, camera)
            error, conf = await loop.run_in_executor(None, detect, frame)
        except Exception:
            log.exception("free_space capture failed in K-turn — defaulting left.")
            error, conf = -1.0, 0.0

        if conf >= MIN_CONFIDENCE and error > 0:
            steer_angle, opposite_angle = _STEER_RIGHT, _STEER_LEFT
        else:
            steer_angle, opposite_angle = _STEER_LEFT, _STEER_RIGHT

        controller.steer(steer_angle)
        await asyncio.sleep(0.3)
        async with camera.reverse_cam():
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


# ---------------------------------------------------------------------------
# Phase handlers
# ---------------------------------------------------------------------------

async def _handle_approaching(controller, websocket):
    await _send(websocket, "approaching", 0.0, 0.0)
    controller.forward(APPROACH_SPEED)


async def _handle_clear(controller, camera, websocket):
    loop = asyncio.get_running_loop()
    try:
        frame       = await loop.run_in_executor(None, capture_bgr, camera)
        error, conf = await loop.run_in_executor(None, detect, frame)
    except Exception:
        log.exception("free_space capture failed in clear phase — centering steering.")
        error, conf = 0.0, 0.0

    if conf >= MIN_CONFIDENCE:
        steer_angle = round(_CENTER_ANGLE - error * _STEER_HALF_RANGE)
        controller.steer(int(steer_angle))
    else:
        error = 0.0
        controller.steer_center()
    controller.forward(AUTONOMOUS_SPEED)
    await _send(websocket, "clear", error, conf)


async def _free_space_avoid(controller, camera, websocket):
    """Free-space fallback used when YOLO finds no detection or sweep fails."""
    loop = asyncio.get_running_loop()
    try:
        frame       = await loop.run_in_executor(None, capture_bgr, camera)
        error, conf = await loop.run_in_executor(None, detect, frame)
    except Exception:
        log.exception("free_space capture failed in avoidance — reversing straight.")
        error, conf = 0.0, 0.0

    if conf >= MIN_CONFIDENCE:
        decision = "TURN_RIGHT" if error > 0 else "TURN_LEFT"
        log.info("free_space: %s (err=%+.2f) → %s",
                 "right" if error > 0 else "left", error, decision)
        await _send(websocket, "avoiding", error, conf)
        await execute_avoidance(controller, camera, decision)
    else:
        log.info("free_space: blocked (conf=%.2f) — reversing straight.", conf)
        await _send(websocket, "blocked", 0.0, conf)
        controller.steer_center()
        async with camera.reverse_cam():
            controller.backward(REVERSE_SPEED)
            await asyncio.sleep(2.0)
            await controller.smooth_stop()


async def _handle_blocked(controller, obstacle, camera, websocket):
    loop     = asyncio.get_running_loop()
    distance = obstacle.distance_cm()

    if obstacle.is_sudden_stop():
        log.warning("Sudden obstacle at %.1f cm — hard stop.", distance)
        controller.force_stop()
    else:
        v        = controller.current_speed
        d_target = max(distance - _STOP_TARGET_MARGIN, _MIN_DECEL_DIST_CM)
        if abs(v) > _MIN_SPEED:
            required_rate = (v ** 2) * _CM_PER_SPEED_UNIT / (2.0 * d_target)
            required_rate = max(required_rate, MOTOR_CFG["rear"]["decelerate_rate"])
        else:
            required_rate = None
        log.info("Obstacle at %.1f cm — smooth stop (rate=%s).", distance, required_rate)
        await controller.smooth_stop(rate=required_rate)

    # Layer 2: YOLO detection
    try:
        frame      = await loop.run_in_executor(None, capture_bgr, camera)
        detections = await loop.run_in_executor(None, detect_obstacles, frame)
    except Exception:
        log.exception("Camera/YOLO failure in blocked phase — forcing stop.")
        controller.force_stop()
        return

    primary = select_primary_obstacle(detections, frame_width=_FRAME_W)
    if primary is not None:
        threat = classify_width_threat(primary, frame_width=_FRAME_W)
        try:
            # Layer 3: servo sweep (blocks ~300–450 ms — run in thread pool)
            sweep = await loop.run_in_executor(
                None, sweep_obstacle, controller, obstacle.sensor,
                primary["x1"], primary["x2"],
            )
        except Exception:
            log.exception("Servo sweep failed — falling back to free-space.")
        else:
            width_cm = calculate_real_width(
                primary["x2"] - primary["x1"], sweep["center"], _FOCAL_LENGTH_PX
            )
            decision = decide_avoidance(threat, sweep)
            log.info("YOLO: %s obstacle ~%.1f cm wide, %.1f cm away → %s",
                     threat, width_cm, sweep["center"], decision)
            await _send(websocket, "avoiding", 0.0, 1.0)
            await execute_avoidance(controller, camera, decision)
            return

    # No YOLO detection or sweep failed — free-space fallback
    await _free_space_avoid(controller, camera, websocket)


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

async def setup(controller):
    controller.center_camera()


async def navigate_step(controller, obstacle, camera, websocket):
    if obstacle.is_blocked():
        await _handle_blocked(controller, obstacle, camera, websocket)
    elif obstacle.should_turn():
        await _handle_approaching(controller, websocket)
    else:
        await _handle_clear(controller, camera, websocket)


async def run_autonomous(controller, obstacle, camera, websocket):
    await setup(controller)
    loop = asyncio.get_running_loop()
    try:
        while True:
            deadline = loop.time() + _LOOP_PERIOD
            await navigate_step(controller, obstacle, camera, websocket)
            remaining = deadline - loop.time()
            if remaining > 0:
                await asyncio.sleep(remaining)
    except asyncio.CancelledError:
        await controller.smooth_stop()
