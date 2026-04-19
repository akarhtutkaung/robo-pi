"""
DC Motor control using PCA9685 PWM driver. 
This module provides a class to control the rear motor of the robot, 
allowing for speed and direction adjustments. 
It uses the Adafruit Motor library for easy motor control 
and the Adafruit PCA9685 library for PWM signal generation.
"""

import board, busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor as adafruit_motor

class RearMotor:
    def __init__(self, pca, motor_cfg):
        self._pca = pca
        self._motor = adafruit_motor.DCMotor(
            self._pca.channels[motor_cfg["rear"]["channel_in1"]],
            self._pca.channels[motor_cfg["rear"]["channel_in2"]]
        )

    def set_speed(self, speed: int):
        speed = max(-100, min(100, speed))
        throttle = speed / 100.0
        self._motor.throttle = throttle

    def stop(self):
        self._motor.throttle = 0

    def cleanup(self):
        self._pca.deinit()