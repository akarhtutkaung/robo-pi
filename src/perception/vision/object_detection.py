# Interprets sensor data into meaningful signals for the autonomous loop — raw hardware reading
# lives one layer down in hardware/sensors/ultrasonic.py.

from src.core.config import ULTRASONIC_CFG
from src.hardware.sensors.ultrasonic import UltrasonicSensor

STOP_CM = ULTRASONIC_CFG["stop_cm"]
TURN_CM = ULTRASONIC_CFG["turn_cm"]

class ObstacleDetector:
    def __init__(self):
        self._sensor = UltrasonicSensor()

    def distance_cm(self) -> float:
        return self._sensor.distance_cm()

    def is_blocked(self) -> bool:
        # """True when an obstacle is within STOP_CM."""
        return self.distance_cm() < STOP_CM

    def should_turn(self) -> bool:
        # """True when an obstacle is close enough to start drifting."""
        return self.distance_cm() < TURN_CM

    def cleanup(self):
        self._sensor.cleanup()

    def get_distance(self) -> float:
        return self.distance_cm()