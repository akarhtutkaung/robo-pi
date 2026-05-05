"""
Autonomous drive loop — runs while the robot is in autonomous mode.
Triggered by a {"type": "mode", "action": "autonomous"} WebSocket message.
Extend this with obstacle avoidance, SLAM, etc. as sensors are implemented.
"""

import asyncio
from src.navigation.controller import RobotController
from src.perception.vision.object_detection import ObstacleDetector
from src.core.config import MOTOR_CFG

AUTONOMOUS_SPEED     = 6
APPROACH_SPEED       = 3    # half speed when obstacle is in the braking zone
_CM_PER_SPEED_UNIT   = MOTOR_CFG["rear"]["cm_per_speed_unit"]
_STOP_TARGET_MARGIN  = 5.0  # cm — stop within this distance of the obstacle

async def setup(controller):
    controller.center_camera()

async def obstacle_algorithm(controller, obstacle):
    if obstacle.is_blocked():
        distance = obstacle.distance_cm()

        if obstacle.is_sudden_stop():
            print(f"Sudden obstacle at {distance:.1f} cm — hard stop.")
            controller.force_stop()
        else:
            v        = controller.current_speed
            d_target = distance - _STOP_TARGET_MARGIN
            if d_target > 0 and abs(v) > 0.1:
                # v² = 2*a*d → rate = v²*K / (2*d_target), K = cm_per_speed_unit
                required_rate = (v ** 2) * _CM_PER_SPEED_UNIT / (2.0 * d_target)
                required_rate = max(required_rate, MOTOR_CFG["rear"]["decelerate_rate"])
            else:
                required_rate = None
            print(f"Obstacle at {distance:.1f} cm — smooth stop (rate={required_rate}).")
            await controller.smooth_stop(rate=required_rate)

        print("Initiating avoidance maneuvers.")

        controller.move_camera_to("x", 45)
        await asyncio.sleep(1)
        right_blocked = obstacle.is_blocked()
        right_distance = obstacle.get_distance() if right_blocked else None

        controller.move_camera_to("x", 135)
        await asyncio.sleep(1)
        left_blocked = obstacle.is_blocked()
        left_distance = obstacle.get_distance() if left_blocked else None
        controller.center_camera()

        if right_blocked and not left_blocked:
            print("Obstacle on the right, turning left")
            steer_angle = 60
        elif left_blocked and not right_blocked:
            print("Obstacle on the left, turning right")
            steer_angle = 120
        elif not right_blocked and not left_blocked:
            print("Both sides clear, defaulting right")
            steer_angle = 120
        else:
            print(f"All directions blocked — right: {right_distance}cm, left: {left_distance}cm. Backing up straight.")
            steer_angle = None

        controller.backward(AUTONOMOUS_SPEED)
        await asyncio.sleep(1.5)
        await controller.smooth_stop()

        if steer_angle is not None:
            controller.steer(steer_angle)
            await asyncio.sleep(0.3)
            controller.forward(AUTONOMOUS_SPEED)
            await asyncio.sleep(1.5)
            await controller.smooth_stop()

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
            await obstacle_algorithm(controller, obstacle)
    except asyncio.CancelledError:
        await controller.smooth_stop()
    