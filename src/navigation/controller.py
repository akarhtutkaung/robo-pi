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

    def move_camera_x(self, angle: int):
        # cfg = SERVO_CFG["servo1"]
        # current_angle = self._servo._servos["servo1"].angle or SERVO_CFG["servo1"]["center_angle"]
        # new_angle = max(cfg["min_angle"], min(cfg["max_angle"], current_angle + degree))
        # self._servo.set_angle("servo1", new_angle)
        cfg = SERVO_CFG["servo1"]
        angle = max(cfg["max_angle"], min(cfg["min_angle"], angle))
        self._servo.set_angle("servo1", angle)

    def move_camera_y(self, angle: int):
        # cfg = SERVO_CFG["servo2"]
        # current_angle = self._servo._servos["servo2"].angle or SERVO_CFG["servo2"]["center_angle"]
        # new_angle = max(cfg["min_angle"], min(cfg["max_angle"], current_angle + degree))
        # self._servo.set_angle("servo2", new_angle)
        cfg = SERVO_CFG["servo2"]
        angle = max(cfg["max_angle"], min(cfg["min_angle"], angle))
        self._servo.set_angle("servo2", angle)

    async def smooth_stop(self):
        await self._motor.smooth_stop()
        self._servo.center("servo0")

    def stop(self):
        self._motor.stop()
        self._servo.center("servo0")

    def is_stopped(self):
        return self._motor.is_stopped() and self._servo.is_stopped("servo0")

    def cleanup(self):
        self.stop()
        self._servo.cleanup()
        self._motor.cleanup()
        self._pca.channels[0].duty_cycle = 0
        self._pca.deinit() 