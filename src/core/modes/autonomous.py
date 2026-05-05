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
    await setup(controller)
    try:
        while True:
            if obstacle.is_blocked():
                print("Obstacle detected ahead! Initiating avoidance maneuvers.")
                controller.smooth_stop()  # immediately cut power to motors to prevent collision
                await asyncio.sleep(2)  # brief pause to allow robot to come to a complete stop

                # Check left and right before turning to avoid getting stuck in a corner
                # by moving camera and checking for obstacles in each direction. This is a simple heuristic
                # and can be improved with more sophisticated logic or additional sensors.
                controller.move_camera_to("x", 45)         # look right
                await asyncio.sleep(1)                 # give camera time to move
                right_blocked = obstacle.is_blocked()
                right_blocked_distance = obstacle.get_distance() if right_blocked else None
                
                controller.move_camera_to("x", 135)        # look left
                await asyncio.sleep(1)                 # give camera time to move
                left_blocked = obstacle.is_blocked()
                left_blocked_distance = obstacle.get_distance() if left_blocked else None
                controller.center_camera()                # reset camera position

                if right_blocked and not left_blocked:
                    print("Obstacle on the right, turning left")
                    controller.steer(60)                # turn left to clear
                elif left_blocked and not right_blocked:
                    print("Obstacle on the left, turning right")
                    controller.steer(120)               # turn right to clear
                elif not right_blocked and not left_blocked:
                    print("Obstacle ahead, but both sides are clear. Choosing to turn right.")
                    controller.steer(120)               # turn right to clear
                else:
                    print("Obstacle ahead, but both sides are blocked. Moving back further.")
                    print(f"Right blocked: {right_blocked}, distance: {right_blocked_distance}")
                    print(f"Left blocked: {left_blocked}, distance: {left_blocked_distance}")
                controller.backward(AUTONOMOUS_SPEED)  # back up more
                await asyncio.sleep(1)                 # back up for another second
                controller.steer_center()  # reset steering after maneuver
            else:
                controller.forward(AUTONOMOUS_SPEED)  # keep moving forward
            await asyncio.sleep(0.1)  # small delay to prevent overwhelming the loop
    except asyncio.CancelledError:
        pass