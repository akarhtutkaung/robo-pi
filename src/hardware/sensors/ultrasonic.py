# Hardware: HC-SR04 on GPIO trigger=23, echo=24 (see config/hardware.yaml)
# gpiozero.DistanceSensor drives the sensor in a background thread — reading
# .distance is non-blocking and always returns the latest measurement.

from gpiozero import DistanceSensor
from gpiozero.pins.lgpio import LGPIOFactory
from src.core.config import ULTRASONIC_CFG

class UltrasonicSensor:
    def __init__(self, trigger=ULTRASONIC_CFG["trigger_pin"], echo=ULTRASONIC_CFG["echo_pin"]):
        self._sensor = DistanceSensor(echo=echo, trigger=trigger, max_distance=ULTRASONIC_CFG["max_distance"], pin_factory=LGPIOFactory())

    def distance_cm(self) -> float:
        return self._sensor.distance * 100  # gpiozero returns metres
    
    def cleanup(self):
        self._sensor.close()