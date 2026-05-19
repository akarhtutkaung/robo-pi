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

from websockets.exceptions import ConnectionClosed

from src.perception.vision.free_space import detect, MIN_CONFIDENCE
from src.perception.vision.object_detection import (
    detect_obstacles, select_primary_obstacle, classify_width_threat,
    sweep_obstacle, calculate_real_width,
)
from src.perception.camera import capture_bgr
from src.core.config import MOTOR_CFG, AUTONOMOUS_CFG, SERVO_CFG

from src.perception.sweep_cache import (
    _SweepCache,
    _SERVO1_CENTER, _ULTRASONIC_HEIGHT_CM,
    _STOP_CM, _WARN_CM, _SWEEP_POSITIONS, _FOCAL_LENGTH_PX, _ROBOT_WIDTH_CM,
    _SWEEP_SIDE_CORRIDOR_CM, _FRAME_W, _YOLO_BLOCK_RATIO,
)
from src.navigation.avoidance import (
    decide_avoidance, execute_avoidance, _reverse_with_obstacle_check,
    AUTONOMOUS_SPEED, REVERSE_SPEED, _STEER_LEFT, _STEER_RIGHT,
)

log = logging.getLogger(__name__)


class _WedgeError(RuntimeError):
    """Robot is confirmed wedged (front and rear both blocked). Triggers immediate halt."""


APPROACH_SPEED      = AUTONOMOUS_CFG["approach_speed"]
_CM_PER_SPEED_UNIT  = MOTOR_CFG["rear"]["cm_per_speed_unit"]
_REVERSE_FALLBACK_S = AUTONOMOUS_CFG["reverse_fallback_s"]
# from src.core.config import ULTRASONIC_REAR_CFG  # rear ultrasonic not installed
# _REAR_STOP_CM       = ULTRASONIC_REAR_CFG.get("stop_cm", 20)  # rear ultrasonic not installed
_STOP_TARGET_MARGIN = 5.0   # cm — target stop distance in front of obstacle
_MIN_DECEL_DIST_CM  = 1.0   # lower bound on d_target; prevents ÷0 in decel formula

_CENTER_ANGLE     = SERVO_CFG["servo0"]["center_angle"]  # 94.68
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

_MIN_SPEED   = 0.1   # below this throttle the robot is considered stopped
_LOOP_PERIOD = 0.1   # target seconds per navigation tick
_HEAD_SETTLE_S   = 0.15  # seconds — covers 80-120 ms servo travel + camera pipeline latency
_MAX_CONSECUTIVE_ERRORS = 5


# ---------------------------------------------------------------------------
# WebSocket helper
# ---------------------------------------------------------------------------

def _direction_label(error: float) -> str:
    a = abs(error)
    for threshold, prefix in _DIRECTION_THRESHOLDS:
        if a < threshold:
            if prefix == "CENTER":
                return "CENTER"
            side = "LEFT" if error < 0 else "RIGHT"
            return f"{prefix} {side}".strip() if prefix else side
    # a >= 2.0: unreachable for error in [-1, 1] but saturates to HARD
    return "HARD LEFT" if error < 0 else "HARD RIGHT"


async def _send(websocket, phase: str, error: float = 0.0, confidence: float = 0.0):
    try:
        await websocket.send(json.dumps({
            "type":       "drive_state",
            "phase":      phase,
            "direction":  _direction_label(error),
            "error":      round(error, 3),
            "confidence": round(confidence, 2),
        }))
    except (ConnectionClosed, OSError):
        pass  # client disconnected — not an error
    except Exception:
        log.exception("Unexpected error sending drive_state")


# ---------------------------------------------------------------------------
# Phase handlers
# ---------------------------------------------------------------------------

async def _handle_approaching(controller, websocket):
    # controller.forward() starts the internal ramp loop (accelerate_rate in hardware.yaml)
    # so APPROACH_SPEED is reached gradually — no lurch on entry to this phase.
    await _send(websocket, "approaching", 0.0, 0.0)
    controller.forward(APPROACH_SPEED)


async def _handle_clear(controller, camera, obstacle, websocket, sweep_cache):
    """One tick of the clear phase.

    Advances the sweep index, points the head servo to the next position, then
    concurrently captures a frame and reads the ultrasonic sensor (which now
    measures in the direction the head is pointing).

    On center ticks: YOLO + free-space run concurrently; steering is updated.
    On left/right ticks: YOLO only (free-space is invalid when the head is angled).

    Speed is reduced to APPROACH_SPEED if the sweep cache shows a YOLO-confirmed
    obstacle inside the warning zone; otherwise full AUTONOMOUS_SPEED.
    """
    loop = asyncio.get_running_loop()
    name, head_angle = sweep_cache.advance()
    controller.move_camera_to("x", head_angle)

    # Capture frame and read ultrasonic concurrently — both block the calling thread.
    try:
        frame, dist = await asyncio.gather(
            loop.run_in_executor(None, capture_bgr, camera),
            loop.run_in_executor(None, obstacle.sensor.distance_cm),
        )
    except Exception:
        log.exception("clear phase: capture/sensor failed — skipping tick.")
        return

    sweep_cache.distances[name] = dist

    if name == "center":
        # YOLO and free-space both need the same forward frame — run them in parallel.
        try:
            dets, (error, conf) = await asyncio.gather(
                loop.run_in_executor(None, detect_obstacles, frame),
                loop.run_in_executor(None, detect, frame),
            )
        except Exception:
            log.exception("clear phase center: inference failed.")
            dets, error, conf = [], 0.0, 0.0
        sweep_cache.detections[name] = dets
        in_path = sweep_cache.in_corridor()
        if in_path:
            controller.steer_center()
            error = 0.0
        elif conf >= MIN_CONFIDENCE:
            controller.steer(int(round(_CENTER_ANGLE - error * _STEER_HALF_RANGE)))
        else:
            error = 0.0
            controller.steer_center()
        sweep_cache.last_error = error
        sweep_cache.last_conf  = conf
    else:
        # Off-center: free-space result would be invalid; YOLO still gives early warning.
        try:
            sweep_cache.detections[name] = await loop.run_in_executor(
                None, detect_obstacles, frame
            )
        except Exception:
            log.exception("clear phase: YOLO failed at %s.", name)
            sweep_cache.detections[name] = []

    speed = APPROACH_SPEED if sweep_cache.should_slow() else AUTONOMOUS_SPEED
    controller.forward(speed)
    await _send(websocket, "clear", sweep_cache.last_error, sweep_cache.last_conf)


async def _free_space_avoid(controller, camera, websocket, rear_sensor=None) -> bool:
    """Free-space fallback used when YOLO finds no detection or sweep fails.

    Returns True  — a turn or timed reverse ran; caller must check the forward sensor.
    Returns False — straight reverse was blocked by a rear obstacle; caller should
                    treat this as a confirmed wedge without relying on the forward sensor.
    """
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
        await execute_avoidance(controller, camera, decision, rear_sensor)
        return True  # turn ran; caller verifies forward sensor
    else:
        log.info("free_space: blocked (conf=%.2f) — reversing straight.", conf)
        await _send(websocket, "blocked", 0.0, conf)
        controller.steer_center()
        async with camera.reverse_cam():
            reversed_ok = await _reverse_with_obstacle_check(controller, rear_sensor, _REVERSE_FALLBACK_S)
        if not reversed_ok:
            log.warning("Straight-reverse fallback blocked by rear obstacle.")
        return reversed_ok


async def _handle_blocked(controller, obstacle, camera, websocket, rear_sensor=None):
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

    # Layer 2: YOLO detection — frame captured after smooth_stop() completes,
    # so bounding box positions match the robot's actual stopped position.
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
            avoidance_ok = await execute_avoidance(controller, camera, decision, rear_sensor)
            # avoidance_ok is False  → K-turn reverse blocked; robot barely moved, escalate immediately.
            # avoidance_ok is True   → manoeuvre ran; re-check forward sensor to confirm clear.
            # avoidance_ok is None   → stub/unexpected; treat as success (forward sensor is authoritative).
            if avoidance_ok is False or obstacle.is_blocked():
                reason = "K-turn reverse blocked" if avoidance_ok is False else decision
                log.warning("Still blocked after %s — free-space fallback.", reason)
                freed = await _free_space_avoid(controller, camera, websocket, rear_sensor)
                if not freed or obstacle.is_blocked():
                    log.critical("Wedged: blocked after all avoidance attempts — halting.")
                    controller.force_stop()
                    raise _WedgeError("robot wedged: front blocked after all avoidance attempts")
            return

    # No YOLO detection or sweep failed — free-space fallback
    freed = await _free_space_avoid(controller, camera, websocket, rear_sensor)
    if not freed or obstacle.is_blocked():
        log.critical("Wedged: blocked after free-space fallback — halting.")
        controller.force_stop()
        raise _WedgeError("robot wedged: front blocked after free-space fallback")


async def _handle_side_threat(controller, camera, websocket, sweep_cache, rear_sensor=None):
    """Steer away from a YOLO-confirmed side obstacle cached in the sweep data.

    Uses cached distances directly rather than routing through _handle_blocked —
    the forward ultrasonic may read clear while the side threat is real, so
    _handle_blocked's deceleration logic and forward-facing YOLO sweep would both
    target the wrong axis. A TURN_LEFT/TURN_RIGHT manoeuvre is sufficient since
    the robot is still in forward motion.

    Invalidates the sweep cache on exit so the same reading cannot re-trigger.
    """
    left_d  = sweep_cache.distances.get("left")
    right_d = sweep_cache.distances.get("right")

    left_threatened  = left_d  is not None and left_d  < _WARN_CM and sweep_cache.detections["left"]
    right_threatened = right_d is not None and right_d < _WARN_CM and sweep_cache.detections["right"]

    if left_threatened and (not right_threatened or left_d <= right_d):
        decision = "TURN_RIGHT"
    elif right_threatened:
        decision = "TURN_LEFT"
    else:
        decision = "TURN_RIGHT"   # fallback — should be unreachable via should_avoid_side()

    log.info("Side threat: left=%.1f right=%.1f → %s",
             left_d or -1.0, right_d or -1.0, decision)
    await _send(websocket, "avoiding", 0.0, 1.0)
    avoidance_ok = await execute_avoidance(controller, camera, decision, rear_sensor)
    if avoidance_ok is False:
        log.warning("Side-threat avoidance aborted by rear obstacle — robot may be constrained.")
    sweep_cache.invalidate()


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

async def setup(controller):
    controller.center_camera()
    geometric_floor = (_ULTRASONIC_HEIGHT_CM * _FOCAL_LENGTH_PX) / (_STOP_CM * _FRAME_W)
    log.info(
        "Autonomous: ultrasonic blind-spot < %.0f cm; "
        "YOLO-only avoidance ratio %.2f (geometric floor %.3f).",
        _ULTRASONIC_HEIGHT_CM,
        _YOLO_BLOCK_RATIO,
        geometric_floor,
    )


async def navigate_step(controller, obstacle, camera, websocket, sweep_cache, rear_sensor=None):
    if obstacle.is_blocked() or sweep_cache.any_side_blocked():
        if sweep_cache.any_side_blocked() and not obstacle.is_blocked():
            # Side hard-stop: forward sensor is clear, threat is lateral.
            # Stop, re-centre the head so the camera faces forward, settle,
            # then steer away using the cached sweep data.
            controller.force_stop()
            controller.move_camera_to("x", int(round(_SERVO1_CENTER)))
            await asyncio.sleep(_HEAD_SETTLE_S)
            await _handle_side_threat(controller, camera, websocket, sweep_cache, rear_sensor)
        else:
            # Forward blocked — invalidate stale sweep readings before avoidance
            # so old side distances cannot re-trigger side-threat checks mid-manoeuvre.
            sweep_cache.invalidate()
            await _handle_blocked(controller, obstacle, camera, websocket, rear_sensor)
    elif sweep_cache.should_avoid_side():
        # Side obstacle inside warn_cm with YOLO confirmation — steer away using
        # cached sweep data. _handle_blocked is not used here because the forward
        # ultrasonic may read clear; its deceleration logic and YOLO sweep target
        # the wrong axis for a side threat.
        await _handle_side_threat(controller, camera, websocket, sweep_cache, rear_sensor)
    elif sweep_cache.yolo_blocking():
        # Low obstacle: ultrasonic beam passed over it (obstacle shorter than
        # _ULTRASONIC_HEIGHT_CM), but YOLO sees a large in-corridor detection with
        # a clear ultrasonic reading. Re-centre the head before capturing the blocked
        # frame — bounding box pixel coordinates fed into sweep_obstacle assume the
        # camera is forward-facing; an angled frame produces wrong sweep angles.
        controller.force_stop()
        controller.move_camera_to("x", int(round(_SERVO1_CENTER)))
        await asyncio.sleep(_HEAD_SETTLE_S)
        sweep_cache.invalidate()
        await _handle_blocked(controller, obstacle, camera, websocket, rear_sensor)
    elif obstacle.should_turn():
        await _handle_approaching(controller, websocket)
    else:
        await _handle_clear(controller, camera, obstacle, websocket, sweep_cache)


async def run_autonomous(controller, obstacle, camera, websocket):
    await setup(controller)
    sweep_cache = _SweepCache()
    loop = asyncio.get_running_loop()
    consecutive_errors = 0

    rear_sensor = None  # rear ultrasonic not installed — re-enable init block below when attached
    # --- Rear ultrasonic init (sensor not installed; uncomment when attached) ---
    # from src.hardware.sensors.ultrasonic import UltrasonicSensor
    # from src.core.config import ULTRASONIC_REAR_CFG
    # _rear_trigger = ULTRASONIC_REAR_CFG.get("trigger_pin")
    # _rear_echo    = ULTRASONIC_REAR_CFG.get("echo_pin")
    # if _rear_trigger and _rear_echo:
    #     try:
    #         rear_sensor = UltrasonicSensor(trigger=_rear_trigger, echo=_rear_echo)
    #         log.info("Rear ultrasonic initialised (trigger=%d, echo=%d, stop_cm=%.0f).",
    #                  _rear_trigger, _rear_echo, _REAR_STOP_CM)
    #     except Exception:
    #         log.exception("Rear ultrasonic init failed — reversing without rear detection.")
    # else:
    #     log.info("Rear ultrasonic not configured — reverse uses timer fallback.")
    # -------------------------------------------------------------------------

    try:
        while True:
            deadline = loop.time() + _LOOP_PERIOD
            try:
                await navigate_step(controller, obstacle, camera, websocket, sweep_cache, rear_sensor)
                consecutive_errors = 0
            except asyncio.CancelledError:
                raise
            except _WedgeError:
                # Already force_stopped in _handle_blocked. Exit immediately — no retry.
                log.critical("Autonomous mode halted: robot wedged.")
                raise
            except Exception:
                log.exception("navigate_step failed.")
                consecutive_errors += 1
                if consecutive_errors >= _MAX_CONSECUTIVE_ERRORS:
                    log.critical(
                        "Too many consecutive navigation failures (%d) — forcing stop.",
                        consecutive_errors,
                    )
                    controller.force_stop()
                    raise
            remaining = deadline - loop.time()
            if remaining > 0:
                await asyncio.sleep(remaining)
    except asyncio.CancelledError:
        await controller.smooth_stop()
    finally:
        if rear_sensor is not None:
            rear_sensor.cleanup()
