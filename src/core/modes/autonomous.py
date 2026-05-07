"""
Autonomous drive loop — runs while the robot is in autonomous mode.
Triggered by a {"type": "mode", "action": "autonomous"} WebSocket message.
Extend this with obstacle avoidance, SLAM, etc. as sensors are implemented.
"""

import asyncio
from src.navigation.controller import RobotController
from src.perception.vision.object_detection import ObstacleDetector
from src.core.config import MOTOR_CFG, AUTONOMOUS_CFG, SERVO_CFG

AUTONOMOUS_SPEED     = AUTONOMOUS_CFG["speed"]
REVERSE_SPEED        = AUTONOMOUS_CFG["reverse_speed"]
APPROACH_SPEED       = AUTONOMOUS_CFG["approach_speed"]
_CM_PER_SPEED_UNIT   = MOTOR_CFG["rear"]["cm_per_speed_unit"]
_STOP_TARGET_MARGIN  = 5.0  # cm — stop within this distance of the obstacle

_STEER_RIGHT = SERVO_CFG["servo0"]["max_angle"]  # _STEER_RIGHT — full right turn
_STEER_LEFT  = SERVO_CFG["servo0"]["min_angle"]  # 123   — full left turn

async def setup(controller):
    controller.center_camera()

async def navigate_step(controller, obstacle):
    if obstacle.is_blocked():
        distance = obstacle.distance_cm()

        if obstacle.is_sudden_stop():
            print(f"Sudden obstacle at {distance:.1f} cm — hard stop.")
            controller.force_stop()
        else:
            v = controller.current_speed
            d_target = distance - _STOP_TARGET_MARGIN
            if d_target > 0 and abs(v) > 0.1:
                # v² = 2*a*d → rate = v²*K / (2*d_target), K = cm_per_speed_unit
                required_rate = (v ** 2) * _CM_PER_SPEED_UNIT / (2.0 * d_target)
                required_rate = max(required_rate, MOTOR_CFG["rear"]["decelerate_rate"])
            else:
                required_rate = None
            print(f"Obstacle at {distance:.1f} cm — smooth stop (rate={required_rate}).")
            await controller.smooth_stop(rate=required_rate)

        controller.move_camera_to("x", 45)
        await asyncio.sleep(1)
        right_blocked  = obstacle.is_blocked()
        right_distance = obstacle.get_distance()
        print(f"Right: {'blocked' if right_blocked else 'clear'} at {right_distance:.1f} cm.")

        controller.move_camera_to("x", 135)
        await asyncio.sleep(1)
        left_blocked  = obstacle.is_blocked()
        left_distance = obstacle.get_distance()
        print(f"Left: {'blocked' if left_blocked else 'clear'} at {left_distance:.1f} cm.")
        controller.center_camera()

        if right_blocked and not left_blocked:
            print("Obstacle on the right — turning left")
            steer_angle    = _STEER_RIGHT
            opposite_angle = _STEER_LEFT
        elif left_blocked and not right_blocked:
            print("Obstacle on the left — turning right")
            steer_angle    = _STEER_LEFT
            opposite_angle = _STEER_RIGHT
        elif not right_blocked and not left_blocked:
            if right_distance >= left_distance:
                print(f"Both clear — going right ({right_distance:.1f} > {left_distance:.1f} cm)")
                steer_angle    = _STEER_RIGHT
                opposite_angle = _STEER_LEFT
            else:
                print(f"Both clear — going left ({left_distance:.1f} > {right_distance:.1f} cm)")
                steer_angle    = _STEER_LEFT
                opposite_angle = _STEER_RIGHT
        else:
            print(f"All blocked — right: {right_distance:.1f} cm, left: {left_distance:.1f} cm.")
            steer_angle    = None
            opposite_angle = None

        # K-turn: steer → back → opposite steer → forward → center → stop
        if steer_angle is not None:
            controller.steer(steer_angle)
            await asyncio.sleep(0.3)

        controller.backward(REVERSE_SPEED)
        await asyncio.sleep(1.5)
        await controller.smooth_stop()

        if steer_angle is not None:
            controller.steer(opposite_angle)
            await asyncio.sleep(0.3)
            controller.forward(AUTONOMOUS_SPEED)
            await asyncio.sleep(1.0)
            controller.steer_center()
            await asyncio.sleep(0.5)
            await controller.smooth_stop()
        else:
            controller.steer_center()
    elif obstacle.should_turn():
        controller.forward(APPROACH_SPEED)
    else:
        controller.forward(AUTONOMOUS_SPEED)
    await asyncio.sleep(0.1)
    

async def run_autonomous(controller, obstacle):
    await setup(controller)
    try:
        while True:
            await navigate_step(controller, obstacle)
    except asyncio.CancelledError:
        await controller.smooth_stop()
    