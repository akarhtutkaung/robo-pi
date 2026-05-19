"""
Obstacle avoidance maneuvers for autonomous navigation.

Provides:
  decide_avoidance  — pick TURN_LEFT / TURN_RIGHT / REVERSE_AND_TURN from YOLO threat + sweep
  execute_avoidance — drive the chosen maneuver
  _reverse_with_obstacle_check — timed reverse with optional rear-sensor gate (sensor not yet installed)
"""

import asyncio
import logging

from src.perception.vision.free_space import detect, MIN_CONFIDENCE
from src.perception.camera import capture_bgr
from src.core.config import AUTONOMOUS_CFG, SERVO_CFG, OBSTACLE_AVOIDANCE_CFG

log = logging.getLogger(__name__)

AUTONOMOUS_SPEED    = AUTONOMOUS_CFG["speed"]
REVERSE_SPEED       = AUTONOMOUS_CFG["reverse_speed"]

_TURN_DRIVE_S         = AUTONOMOUS_CFG["turn_drive_s"]
_KTURN_STEER_SETTLE_S = AUTONOMOUS_CFG["kturn_steer_settle_s"]
_KTURN_REVERSE_S      = AUTONOMOUS_CFG["kturn_reverse_s"]
_KTURN_FORWARD_S      = AUTONOMOUS_CFG["kturn_forward_s"]
_KTURN_FINAL_SETTLE_S = AUTONOMOUS_CFG["kturn_final_settle_s"]

_STEER_RIGHT = SERVO_CFG["servo0"]["max_angle"]    # 50  — full right
_STEER_LEFT  = SERVO_CFG["servo0"]["min_angle"]    # 140 — full left

_ROBOT_WIDTH_CM  = OBSTACLE_AVOIDANCE_CFG["robot_width_cm"]
_CLEARANCE_CM    = OBSTACLE_AVOIDANCE_CFG["clearance_buffer_cm"]
_MIN_PASS_GAP_CM = _ROBOT_WIDTH_CM + _CLEARANCE_CM

# --- Rear ultrasonic (sensor not installed; re-enable when attached) ---
# from src.core.config import ULTRASONIC_REAR_CFG
# _REAR_STOP_CM = ULTRASONIC_REAR_CFG.get("stop_cm", 20)


def decide_avoidance(width_threat: str, sweep: dict) -> str:
    """Return the avoidance maneuver to execute given threat class and sweep data.

    width_threat — "WIDE" | "MEDIUM" | "NARROW"  (from classify_width_threat)
    sweep        — {"left": cm, "center": cm, "right": cm}  (from sweep_obstacle)

    Returns "TURN_LEFT" | "TURN_RIGHT" | "REVERSE_AND_TURN"
    """
    if width_threat == "WIDE":
        return "REVERSE_AND_TURN"

    left_cm  = sweep.get("left",  0.0)
    right_cm = sweep.get("right", 0.0)
    if left_cm == 0.0 and right_cm == 0.0:
        log.warning("decide_avoidance: sweep returned zero on both sides — reversing.")
        return "REVERSE_AND_TURN"

    best_side = "TURN_LEFT" if left_cm >= right_cm else "TURN_RIGHT"
    best_cm   = max(left_cm, right_cm)

    if width_threat == "NARROW":
        return best_side

    # MEDIUM — only attempt to pass if the winning side has enough clearance
    return best_side if best_cm >= _MIN_PASS_GAP_CM else "REVERSE_AND_TURN"


async def _reverse_with_obstacle_check(controller, rear_sensor, max_s: float) -> bool:
    """Start reversing for max_s seconds, then smooth-stop.

    rear_sensor parameter reserved for when the rear ultrasonic is installed.
    Currently always runs a plain timed reverse (rear sensor not attached).

    Returns True — reverse ran the full duration.
    Calls smooth_stop() before returning.
    Caller must hold camera.reverse_cam() context.
    """
    # --- Rear-ultrasonic logic (sensor not installed; re-enable when attached) ---
    # loop = asyncio.get_running_loop()
    # if rear_sensor is not None:
    #     try:
    #         initial = await loop.run_in_executor(None, rear_sensor.distance_cm)
    #         if initial <= _REAR_STOP_CM:
    #             log.warning(
    #                 "Insufficient rear clearance (%.1f cm) — skipping reverse.", initial
    #             )
    #             return False
    #     except Exception:
    #         log.exception("Rear pre-check failed — proceeding on timer.")
    # -------------------------------------------------------------------------

    controller.backward(REVERSE_SPEED)

    # --- Sensor-gated reverse loop (re-enable with above block when attached) ---
    # if rear_sensor is not None:
    #     deadline      = asyncio.get_running_loop().time() + max_s
    #     stopped_early = False
    #     while asyncio.get_running_loop().time() < deadline:
    #         try:
    #             dist = await asyncio.get_running_loop().run_in_executor(None, rear_sensor.distance_cm)
    #             if dist <= _REAR_STOP_CM:
    #                 log.info("Rear obstacle at %.1f cm — stopping reverse early.", dist)
    #                 stopped_early = True
    #                 break
    #         except Exception:
    #             log.exception("Rear ultrasonic read failed during reverse — continuing on timer.")
    #         remaining = deadline - asyncio.get_running_loop().time()
    #         if remaining > 0:
    #             await asyncio.sleep(min(0.05, remaining))
    #     await controller.smooth_stop()
    #     return not stopped_early
    # -------------------------------------------------------------------------

    await asyncio.sleep(max_s)
    await controller.smooth_stop()
    return True


async def execute_avoidance(controller, camera, decision: str, rear_sensor=None) -> bool:
    """Execute a steering maneuver based on the avoidance decision.

    "TURN_LEFT"        — steer left → forward 0.8 s → centre
    "TURN_RIGHT"       — steer right → forward 0.8 s → centre
    "REVERSE_AND_TURN" — K-turn: steer → back → opposite steer → forward → centre

    Returns True  — manoeuvre completed normally.
    Returns False — K-turn reverse was blocked by a rear obstacle; forward phase was
                    skipped. Caller must not assume the robot has moved clear.
    """
    if decision == "TURN_LEFT":
        controller.steer(_STEER_LEFT)
        controller.forward(AUTONOMOUS_SPEED)
        await asyncio.sleep(_TURN_DRIVE_S)
        await controller.smooth_stop()
        controller.steer_center()
        return True

    elif decision == "TURN_RIGHT":
        controller.steer(_STEER_RIGHT)
        controller.forward(AUTONOMOUS_SPEED)
        await asyncio.sleep(_TURN_DRIVE_S)
        await controller.smooth_stop()
        controller.steer_center()
        return True

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
        await asyncio.sleep(_KTURN_STEER_SETTLE_S)
        async with camera.reverse_cam():
            reversed_ok = await _reverse_with_obstacle_check(controller, rear_sensor, _KTURN_REVERSE_S)
        if not reversed_ok:
            log.warning("K-turn reverse blocked — skipping forward phase.")
            controller.steer_center()
            return False
        controller.steer(opposite_angle)
        await asyncio.sleep(_KTURN_STEER_SETTLE_S)
        controller.forward(AUTONOMOUS_SPEED)
        await asyncio.sleep(_KTURN_FORWARD_S)
        controller.steer_center()
        await asyncio.sleep(_KTURN_FINAL_SETTLE_S)
        await controller.smooth_stop()
        return True