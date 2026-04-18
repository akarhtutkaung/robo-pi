"""
TUTORIAL: src/hardware/servos.py
==================================
PURPOSE:
    Low-level driver for all three servos on the PCA9685.
      Servo 0 — Front wheel steering (left/right)
      Servo 1 — Head rotation left/right
      Servo 2 — Head rotation up/down
    This file only knows about hardware angles — no direction labels.

STEP 1 — Dependencies
    Same libraries as motors.py — already installed:
        adafruit-circuitpython-pca9685
        adafruit-circuitpython-motor

STEP 2 — Import libraries
    import board, busio
    from adafruit_pca9685 import PCA9685
    from adafruit_motor import servo as adafruit_servo

    Reference: examples/03_servo.py shows the full setup pattern.
    Note: examples/03_servo.py uses address 0x5f — make sure you match this.

STEP 3 — Create a ServoController class
    The class should:
      a) Accept pca_cfg and servo_cfg dicts in __init__.
      b) Initialize the same I2C bus and PCA9685 as motors.py.
         IMPORTANT: The Pi only has one I2C bus. If motors.py and servos.py
         both create a PCA9685, they must share the same bus instance.
         The cleanest approach: create the PCA9685 once in controller.py and
         pass it into both motors.py and servos.py. Update STEP 3 of
         motors.py to accept an existing pca object instead of creating one.
      c) Create a Servo object for each channel:
            adafruit_servo.Servo(pca.channels[channel], actuation_range=180)

    Example:
        class ServoController:
            def __init__(self, pca, servo_cfg):
                self._servos = {
                    name: adafruit_servo.Servo(
                        pca.channels[cfg["channel"]],
                        actuation_range=180
                    )
                    for name, cfg in servo_cfg.items()
                }
                self._cfg = servo_cfg

STEP 4 — Add a set_angle(servo_name, angle) method
    Sets a servo to an absolute angle (0–180 degrees).
    Clamp the value to [0, 180] before setting.

        def set_angle(self, servo_name: str, angle: int):
            angle = max(0, min(180, angle))
            self._servos[servo_name].angle = angle

STEP 5 — Add a center(servo_name) method
    Resets a servo to its center_angle from config.

        def center(self, servo_name: str):
            self.set_angle(servo_name, self._cfg[servo_name]["center_angle"])

STEP 6 — Add a center_all() method
    Calls center() for every servo. Used on startup and shutdown.

NOTE:
    Steering uses servo0. The controller (src/navigation/controller.py) maps
    "left"/"right" to actual angles — servos.py only receives a numeric angle.
"""
