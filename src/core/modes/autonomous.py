"""
Autonomous drive loop — runs while the robot is in autonomous mode.
Triggered by a {"type": "mode", "action": "autonomous"} WebSocket message.
Extend this with obstacle avoidance, SLAM, etc. as sensors are implemented.
"""

import asyncio

AUTONOMOUS_SPEED = 40  # unitless throttle scale


async def run_autonomous(controller):
    """For now. Drive forward continuously until cancelled."""
    controller.center_camera()
    controller.setSpeed(AUTONOMOUS_SPEED)
    try:
        while True:
            await asyncio.sleep(0.1)
    except asyncio.CancelledError:
        pass
