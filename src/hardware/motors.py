"""
DC Motor control using PCA9685 PWM driver. 
This module provides a class to control the rear motor of the robot, 
allowing for speed and direction adjustments. 
It uses the Adafruit Motor library for easy motor control 
and the Adafruit PCA9685 library for PWM signal generation.
"""

import asyncio
import time
import board, busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor as adafruit_motor

class RearMotor:
    def __init__(self, pca, motor_cfg):
        self._pca = pca
        self._motor_cfg = motor_cfg
        self._motor = adafruit_motor.DCMotor(
            self._pca.channels[motor_cfg["rear"]["channel_in1"]],
            self._pca.channels[motor_cfg["rear"]["channel_in2"]]
        )
        self._current_speed = 0.0
        self._last_update = time.monotonic()

    def set_speed(self, speed: int):
        max_speed = self._motor_cfg["rear"]["max_speed"]
        rate = self._motor_cfg["rear"]["accelerate_rate"]  # units per second
        speed = max(-max_speed, min(max_speed, speed))

        now = time.monotonic()
        dt = now - self._last_update
        self._last_update = now

        step = rate * dt
        if speed > self._current_speed:
            self._current_speed = min(float(speed), self._current_speed + step)
        elif speed < self._current_speed:
            self._current_speed = max(float(speed), self._current_speed - step)

        self._motor.throttle = self._current_speed / max_speed
        print(f"Current speed: {self._current_speed:.1f} (requested: {speed})")

    async def smooth_stop(self):
        max_speed = self._motor_cfg["rear"]["max_speed"]
        rate = self._motor_cfg["rear"]["decelerate_rate"]  # units per second
        while abs(self._current_speed) > 0.1:
            now = time.monotonic()
            dt = now - self._last_update
            self._last_update = now
            
            step = rate * dt
            if self._current_speed > 0:
                self._current_speed = max(0.0, self._current_speed - step)
            else:
                self._current_speed = min(0.0, self._current_speed + step)
            self._motor.throttle = self._current_speed / max_speed
            await asyncio.sleep(0.02)
        self.stop()

    def stop(self):
        self._current_speed = 0.0
        self._last_update = time.monotonic()
        self._motor.throttle = 0

    def cleanup(self):
        self._pca.deinit()

    def is_stopped(self):
        return abs(self._current_speed) < 0.1