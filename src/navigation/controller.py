"""
The single, unified API for moving the robot.
All callers (WebSocket handlers, autonomous mode, speech commands) use
only this file to command movement — they never touch hardware directly.

Maps human-readable commands → hardware calls:
    forward / backward  →  RearMotor.set_speed(speed, direction)
    steer               →  ServoController.set_angle("servo0", ...)
    stop                →  RearMotor.stop() + ServoController.center("servo0")
"""
import board, busio
from adafruit_pca9685 import PCA9685
from src.core.config import PCA_CFG, MOTOR_CFG, SERVO_CFG
from src.hardware.motors import RearMotor
from src.hardware.servos import ServoController

class RobotController:
    def __init__(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        self._pca = PCA9685(i2c, address=PCA_CFG["i2c_address"])
        self._pca.frequency = PCA_CFG["frequency"]

        self._motor = RearMotor(self._pca, MOTOR_CFG)
        self._servo = ServoController(self._pca, SERVO_CFG)
        self._servo.center_all()  # safe starting position
    
    def setSpeed(self, speed: int):
        self._motor.set_speed(speed)

    def forward(self, speed: int = 50):
        self._motor.set_speed(speed)

    def backward(self, speed: int = 50):
        self._motor.set_speed(-speed)

    def steer(self, angle: int):
        cfg = SERVO_CFG["servo0"]
        angle = max(cfg["max_angle"], min(cfg["min_angle"], angle))
        self._servo.set_angle("servo0", angle)

    def move_camera(self, axis: str, angle: int):
        servo_name = "servo1" if axis == "x" else "servo2"
        abs_angle = abs(angle)  # convert to positive for hardware call
        if angle >= 0:
            self._servo.increase_angle(servo_name, abs_angle)
        else:
            self._servo.decrease_angle(servo_name, abs_angle)
    
    def move_camera_to(self, axis: str, angle: int):
        servo_name = "servo1" if axis == "x" else "servo2"
        self._servo.set_angle(servo_name, angle)

    def center_camera(self):
        self._servo.center("servo1")
        self._servo.center("servo2")

    def center_steering(self):
        self._servo.center("servo0")

    async def smooth_stop(self):
        await self._motor.smooth_stop()

    def force_stop(self):
        self._motor.stop()
        self._servo.center("servo0")
        self._servo.center("servo1")
        self._servo.center("servo2")
    
    def stop(self):
        self.smooth_stop()
        self.center_camera()
    
    def force_stop_motors(self):
        self._motor.stop()

    def is_stopped(self):
        return self._motor.is_stopped() and self._servo.is_stopped("servo0")

    def cleanup(self):
        self.stop()
        self._servo.cleanup()
        self._motor.cleanup()
        self._pca.channels[0].duty_cycle = 0
        self._pca.deinit() 