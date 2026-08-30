"""
Stub out hardware-only packages so the full test suite can run on any host
(Mac, CI, Pi without connected hardware). Must be imported before any src.*
module that transitively imports gpiozero, picamera2, aiortc, etc.

pytest auto-discovers and runs this file before any test module.
"""
import sys
from unittest.mock import MagicMock

_HARDWARE_STUBS = [
    # GPIO / lgpio
    "gpiozero",
    "gpiozero.pins",
    "gpiozero.pins.lgpio",
    "lgpio",
    "rpi_lgpio",
    # Camera
    "picamera2",
    "libcamera",
    # WebRTC
    "aiortc",
    "av",
    # Adafruit / I2C
    "board",
    "busio",
    "adafruit_pca9685",
    "adafruit_motor",
    "adafruit_motor.motor",
    "adafruit_circuitpython_motor",
    "adafruit_circuitpython_pca9685",
]

for _name in _HARDWARE_STUBS:
    sys.modules.setdefault(_name, MagicMock())
