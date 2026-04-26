"""
Low-level driver for all three servos on the PCA9685.
    Servo 0 — Front wheel steering (left/right)
    Servo 1 — Head rotation left/right
    Servo 2 — Head rotation up/down
This file only knows about hardware angles — no direction labels.
"""
import board, busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo as adafruit_servo

class ServoController:
    def __init__(self, pca, servo_cfg):
        self._servos = {
            name: adafruit_servo.Servo(
                pca.channels[cfg["channel"]],
                actuation_range=180,
                min_pulse=500, 
                max_pulse=2500
            )
            for name, cfg in servo_cfg.items()
        }
        self._cfg = servo_cfg

    def increase_angle(self, servo_name: str, degree: int):
        cfg = self._cfg[servo_name]
        current_angle = self._servos[servo_name].angle
        new_angle = max(cfg["max_angle"], min(cfg["min_angle"], current_angle + degree))
        self._servos[servo_name].angle = new_angle
        
    def decrease_angle(self, servo_name: str, degree: int):
        cfg = self._cfg[servo_name]
        current_angle = self._servos[servo_name].angle
        new_angle = max(cfg["max_angle"], min(cfg["min_angle"], current_angle - degree))
        self._servos[servo_name].angle = new_angle
        
    def set_angle(self, servo_name: str, angle: int):
        angle = max(self._cfg[servo_name]["max_angle"], min(self._cfg[servo_name]["min_angle"], angle))
        self._servos[servo_name].angle = angle

    def center(self, servo_name: str):
        self.set_angle(servo_name, self._cfg[servo_name]["center_angle"])

    def center_all(self):
        for name in self._servos:
            self.center(name)

    def is_stopped(self, servo_name: str):
        return abs(self._servos[servo_name].angle - self._cfg[servo_name]["center_angle"]) < 1

    def cleanup(self):
        self.center_all()
        for servo in self._servos.values():
            servo.angle = None  # release the servo