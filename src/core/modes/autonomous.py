"""
Autonomous drive loop — runs while the robot is in autonomous mode.
Triggered by a {"type": "mode", "action": "autonomous"} WebSocket message.
Extend this with obstacle avoidance, SLAM, etc. as sensors are implemented.
"""

import asyncio
from src.navigation.controller import RobotController
from src.perception.vision.object_detection import ObstacleDetector

AUTONOMOUS_SPEED = 8  # unitless throttle scale

async def setup(controller):
    controller.center_camera()

async def run_autonomous(controller, obstacle):
    # """For now. Drive forward continuously until cancelled."""
    setup(controller)
    try:
        while True:
            if obstacle.is_blocked():
                await controller.smooth_stop()
                controller.backward(AUTONOMOUS_SPEED)  # back up a bit
                await asyncio.sleep(1)                 # back up for 1 second
                controller.smooth_stop()

                # Check left and right before turning to avoid getting stuck in a corner
                # by moving camera and checking for obstacles in each direction. This is a simple heuristic
                # and can be improved with more sophisticated logic or additional sensors.
                controller.move_camera_to("x", 30)         # look right
                await asyncio.sleep(0.5)                 # give camera time to move
                right_blocked = obstacle.is_blocked()
                
                controller.move_camera_to("x", -30)        # look left
                await asyncio.sleep(0.5)                 # give camera time to move
                left_blocked = obstacle.is_blocked()
                controller.center_camera()                # reset camera position

                if right_blocked and not left_blocked:
                    controller.steer(60)                # turn left to clear
                elif left_blocked and not right_blocked:
                    controller.steer(120)               # turn right to clear
                controller.setSpeed(AUTONOMOUS_SPEED)
    except asyncio.CancelledError:
        pass