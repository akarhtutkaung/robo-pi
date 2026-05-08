"""
Autonomous drive loop — runs while the robot is in autonomous mode.
Triggered by a {"type": "mode", "action": "autonomous"} WebSocket message.
"""

import asyncio
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


async def setup(controller):
    controller.center_camera()


async def navigate_step(controller, obstacle, camera):
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

        # Camera determines which side has more free space
        error, conf = detect(capture_bgr(camera))
        if conf >= MIN_CONFIDENCE and error > 0:
            print(f"Camera: free space right (err={error:+.2f}) — turning right")
            steer_angle, opposite_angle = _STEER_RIGHT, _STEER_LEFT
        elif conf >= MIN_CONFIDENCE and error < 0:
            print(f"Camera: free space left (err={error:+.2f}) — turning left")
            steer_angle, opposite_angle = _STEER_LEFT, _STEER_RIGHT
        else:
            print(f"Camera: low confidence ({conf:.2f}) — defaulting right")
            steer_angle, opposite_angle = _STEER_RIGHT, _STEER_LEFT

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
        controller.forward(APPROACH_SPEED)

    else:
        # Proportional steering toward free space
        error, conf = detect(capture_bgr(camera))
        if conf >= MIN_CONFIDENCE:
            steer_angle = round(_CENTER_ANGLE - error * _STEER_HALF_RANGE)
            controller.steer(int(steer_angle))
        else:
            controller.steer_center()
        controller.forward(AUTONOMOUS_SPEED)

    await asyncio.sleep(0.1)


async def run_autonomous(controller, obstacle, camera):
    await setup(controller)
    try:
        while True:
            await navigate_step(controller, obstacle, camera)
    except asyncio.CancelledError:
        await controller.smooth_stop()
