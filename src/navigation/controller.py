"""
TUTORIAL: src/navigation/controller.py
========================================
PURPOSE:
    The single, unified API for moving the robot.
    All callers (WebSocket handlers, autonomous mode, speech commands) use
    only this file to command movement — they never touch hardware directly.

    Maps human-readable commands → hardware calls:
        forward / backward  →  RearMotor.set_speed()
        left / right        →  ServoController.set_angle("servo0", ...)
        stop                →  RearMotor.stop() + ServoController.center("servo0")

STEP 1 — Import hardware modules
    from src.hardware.motors import RearMotor
    from src.hardware.servos import ServoController
    from src.core.config import PCA_CFG, MOTOR_CFG, SERVO_CFG

STEP 2 — Initialize shared PCA9685 here (not in motors/servos)
    Because both motors and servos share the same physical PCA9685 chip on
    the same I2C bus, create one PCA9685 instance here and pass it to both.

    Example:
        import board, busio
        from adafruit_pca9685 import PCA9685

        class RobotController:
            def __init__(self):
                i2c = busio.I2C(board.SCL, board.SDA)
                pca = PCA9685(i2c, address=PCA_CFG["i2c_address"])
                pca.frequency = PCA_CFG["frequency"]

                self._motor = RearMotor(pca, MOTOR_CFG)
                self._servo = ServoController(pca, SERVO_CFG)
                self._servo.center_all()  # safe starting position

STEP 3 — Add forward(speed) and backward(speed) methods
    speed is a value from 0 to 100 (percent). Convert to 0.0–1.0 before
    passing to the motor.

        def forward(self, speed: int = 50):
            self._motor.set_speed(speed / 100.0)

        def backward(self, speed: int = 50):
            self._motor.set_speed(-(speed / 100.0))

STEP 4 — Add steer(direction) method
    direction is "left", "right", or "center".
    Read the angle limits from SERVO_CFG["servo0"].

        def steer(self, direction: str):
            cfg = SERVO_CFG["servo0"]
            if direction == "left":
                self._servo.set_angle("servo0", cfg["max_left"])
            elif direction == "right":
                self._servo.set_angle("servo0", cfg["max_right"])
            else:  # "center" or anything else
                self._servo.center("servo0")

STEP 5 — Add stop() method
    Stops the motor and centers steering. This is the safe idle state.

        def stop(self):
            self._motor.stop()
            self._servo.center("servo0")

STEP 6 — Add cleanup() method
    Called when the program exits (KeyboardInterrupt or shutdown signal).
    Should stop the motor, center all servos, then cleanup the motor.

        def cleanup(self):
            self.stop()
            self._servo.center_all()
            self._motor.cleanup()

NOTE:
    Use a single shared instance of RobotController across the whole program.
    Create it once in src/core/modes/remote.py and pass it to handlers.
    Do not create multiple instances — it would fight over the I2C bus.
"""
