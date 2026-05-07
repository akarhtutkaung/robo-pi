"""
DC Motor control using PCA9685 PWM driver.
This module provides a class to control the rear motor of the robot,
allowing for speed and direction adjustments.
It uses the Adafruit Motor library for easy motor control
and the Adafruit PCA9685 library for PWM signal generation.
"""

import asyncio
import time
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor as adafruit_motor

_RAMP_DT = 0.02  # ramp loop interval — 50 Hz


class RearMotor:
    def __init__(self, pca, motor_cfg):
        self._pca = pca
        self._motor_cfg = motor_cfg
        self._motor = adafruit_motor.DCMotor(
            self._pca.channels[motor_cfg["rear"]["channel_in1"]],
            self._pca.channels[motor_cfg["rear"]["channel_in2"]]
        )
        self._current_speed = 0.0
        self._target_speed = 0.0
        self._ramp_task: asyncio.Task | None = None

    @property
    def current_speed(self) -> float:
        return self._current_speed

    def set_speed(self, speed: int):
        max_speed = self._motor_cfg["rear"]["max_speed"]
        self._target_speed = float(max(-max_speed, min(max_speed, speed)))
        if self._ramp_task is None or self._ramp_task.done():
            self._ramp_task = asyncio.create_task(self._ramp_loop())

    async def _ramp_loop(self):
        max_speed = self._motor_cfg["rear"]["max_speed"]
        accel         = self._motor_cfg["rear"]["accelerate_rate"]
        reverse_accel = self._motor_cfg["rear"]["reverse_accelerate_rate"]
        decel         = self._motor_cfg["rear"]["decelerate_rate"]
        while abs(self._current_speed - self._target_speed) >= 0.05:
            toward_zero    = (self._current_speed > 0 and self._target_speed < self._current_speed) or \
                             (self._current_speed < 0 and self._target_speed > self._current_speed)
            into_reverse   = abs(self._current_speed) < 0.1 and self._target_speed < 0
            if toward_zero:
                rate = decel
            elif into_reverse:
                rate = reverse_accel  # slow creep into reverse, like a real car
            else:
                rate = accel
            step = rate * _RAMP_DT
            if self._target_speed > self._current_speed:
                self._current_speed = min(self._target_speed, self._current_speed + step)
            else:
                self._current_speed = max(self._target_speed, self._current_speed - step)
            self._motor.throttle = self._current_speed / max_speed
            await asyncio.sleep(_RAMP_DT)
        self._current_speed = self._target_speed
        self._motor.throttle = self._current_speed / max_speed

    async def smooth_stop(self, rate: float | None = None):
        self._target_speed = 0.0
        if self._ramp_task and not self._ramp_task.done():
            self._ramp_task.cancel()
            try:
                await self._ramp_task
            except asyncio.CancelledError:
                pass
            self._ramp_task = None

        max_speed = self._motor_cfg["rear"]["max_speed"]
        effective_rate = rate if rate is not None else self._motor_cfg["rear"]["decelerate_rate"]
        step = effective_rate * _RAMP_DT
        while abs(self._current_speed) > 0.1:
            if self._current_speed > 0:
                self._current_speed = max(0.0, self._current_speed - step)
            else:
                self._current_speed = min(0.0, self._current_speed + step)
            self._motor.throttle = self._current_speed / max_speed
            await asyncio.sleep(_RAMP_DT)
        self.stop()

    def stop(self):
        self._motor.throttle = 0
        self._current_speed = 0.0
        self._target_speed = 0.0
        if self._ramp_task and not self._ramp_task.done():
            self._ramp_task.cancel()
        self._ramp_task = None

    def cleanup(self):
        self._pca.deinit()

    def is_stopped(self):
        return abs(self._current_speed) < 0.1
