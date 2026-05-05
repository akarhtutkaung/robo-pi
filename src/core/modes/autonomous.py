"""
Autonomous drive loop — runs while the robot is in autonomous mode.
Triggered by a {"type": "mode", "action": "autonomous"} WebSocket message.
Extend this with obstacle avoidance, SLAM, etc. as sensors are implemented.
"""

import asyncio
from src.navigation.controller import RobotController
from src.perception.vision.object_detection import ObstacleDetector

AUTONOMOUS_SPEED = 8
APPROACH_SPEED = 4   # half speed when obstacle is close but not yet blocking

async def setup(controller):
    controller.center_camera()

async def obstacle_algorithm(controller, obstacle):
    if obstacle.is_blocked():
        print("Obstacle detected ahead! Initiating avoidance maneuvers.")
        await controller.smooth_stop()

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
        pass
    