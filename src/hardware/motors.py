"""
TUTORIAL: src/hardware/motors.py
==================================
PURPOSE:
    Low-level driver for the rear DC motor.
    This file only knows about hardware — no movement logic, no WebSocket.
    It wraps the Adafruit PCA9685 + DCMotor libraries.

STEP 1 — Install dependencies (on the Pi)
    pip install adafruit-circuitpython-pca9685
    pip install adafruit-circuitpython-motor

STEP 2 — Import libraries
    You need:
        import board, busio
        from adafruit_pca9685 import PCA9685
        from adafruit_motor import motor as adafruit_motor

    Reference: examples/04_motor.py shows the full setup pattern.

STEP 3 — Create a RearMotor class
    The class should:
      a) Accept a config dict (from src.core.config.MOTOR_CFG and PCA_CFG)
         in __init__ so it is not hardcoded.
      b) Initialize the I2C bus using board.SCL and board.SDA.
      c) Create a PCA9685 instance at the configured I2C address.
      d) Set pca.frequency to the configured value (50 Hz).
      e) Create a DCMotor using adafruit_motor.DCMotor(
             pca.channels[channel_in1],
             pca.channels[channel_in2]
         )

    Example class skeleton:
        class RearMotor:
            def __init__(self, pca_cfg, motor_cfg):
                i2c = busio.I2C(board.SCL, board.SDA)
                self._pca = PCA9685(i2c, address=pca_cfg["i2c_address"])
                self._pca.frequency = pca_cfg["frequency"]
                self._motor = adafruit_motor.DCMotor(
                    self._pca.channels[motor_cfg["rear"]["channel_in1"]],
                    self._pca.channels[motor_cfg["rear"]["channel_in2"]]
                )

STEP 4 — Add a set_speed(speed) method
    speed is a float from -1.0 to 1.0:
        +1.0 = full forward
        -1.0 = full backward
         0.0 = stop
    Assign it directly to self._motor.throttle.
    Clamp the value to [-1.0, 1.0] before assigning to avoid hardware errors.

    Example:
        def set_speed(self, speed: float):
            self._motor.throttle = max(-1.0, min(1.0, speed))

STEP 5 — Add a stop() method
    Sets throttle to 0. Used for clean stops.

        def stop(self):
            self._motor.throttle = 0

STEP 6 — Add a cleanup() method
    Deinitializes the PCA9685 when the program exits.

        def cleanup(self):
            self._pca.deinit()

NOTE:
    The controller (src/navigation/controller.py) is the only caller of this class.
    Do not call RearMotor directly from handlers or WebSocket code.
"""
