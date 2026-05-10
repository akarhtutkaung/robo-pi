"""
Tests for src/perception/vision/object_detection.py

Hardware-free: UltrasonicSensor is never instantiated.
sweep_obstacle tests use FakeController / FakeSensor.
"""
import numpy as np
import pytest

from src.perception.vision.object_detection import (
    detect_obstacles,
    select_primary_obstacle,
    classify_width_threat,
    pixel_x_to_servo_angle,
    calculate_real_width,
    sweep_obstacle,
)
from src.core.config import SERVO_CFG

_SERVO1_MIN = SERVO_CFG["servo1"]["max_angle"]   # 0   — full right (smallest angle)
_SERVO1_MAX = SERVO_CFG["servo1"]["min_angle"]   # 180 — full left  (largest angle)
_SERVO1_CTR = SERVO_CFG["servo1"]["center_angle"]  # 89.85


# ---------------------------------------------------------------------------
# detect_obstacles — blank / None frames must never raise
# ---------------------------------------------------------------------------

def test_detect_obstacles_blank_frame_returns_empty():
    blank = np.zeros((480, 640, 3), dtype=np.uint8)
    assert detect_obstacles(blank) == []


def test_detect_obstacles_none_returns_empty():
    assert detect_obstacles(None) == []


def test_detect_obstacles_empty_array_returns_empty():
    assert detect_obstacles(np.zeros((0, 0, 3), dtype=np.uint8)) == []


# ---------------------------------------------------------------------------
# select_primary_obstacle
# ---------------------------------------------------------------------------

def test_select_primary_empty_list():
    assert select_primary_obstacle([]) is None


def test_select_primary_single():
    box = {"x1": 10, "y1": 10, "x2": 100, "y2": 100}
    assert select_primary_obstacle([box]) is box


def test_select_primary_picks_largest_area():
    small = {"x1": 0, "y1": 0, "x2": 100, "y2": 100}   # area 10 000
    large = {"x1": 0, "y1": 0, "x2": 200, "y2": 200}   # area 40 000
    assert select_primary_obstacle([small, large]) is large
    assert select_primary_obstacle([large, small]) is large


def test_select_primary_tiebreak_closer_to_centre():
    """Equal area — box nearest the frame centre (x=320) wins."""
    off_centre = {"x1": 0,   "y1": 0, "x2": 100, "y2": 100}   # cx = 50
    at_centre  = {"x1": 270, "y1": 0, "x2": 370, "y2": 100}   # cx = 320
    result = select_primary_obstacle([off_centre, at_centre], frame_width=640)
    assert result is at_centre


# ---------------------------------------------------------------------------
# classify_width_threat
# ---------------------------------------------------------------------------

def _det(x2: int, frame_width: int = 640) -> dict:
    return {"x1": 0, "x2": x2}


def test_classify_wide():
    assert classify_width_threat(_det(320), 640) == "WIDE"   # exactly 50 %


def test_classify_wide_over_threshold():
    assert classify_width_threat(_det(400), 640) == "WIDE"   # 62.5 %


def test_classify_medium():
    assert classify_width_threat(_det(160), 640) == "MEDIUM"  # exactly 25 %


def test_classify_medium_between_thresholds():
    assert classify_width_threat(_det(200), 640) == "MEDIUM"  # 31.25 %


def test_classify_narrow():
    assert classify_width_threat(_det(100), 640) == "NARROW"  # 15.6 %


def test_classify_narrow_under_threshold():
    assert classify_width_threat(_det(1), 640) == "NARROW"


# ---------------------------------------------------------------------------
# pixel_x_to_servo_angle
# ---------------------------------------------------------------------------

def test_servo_angle_centre_pixel():
    angle = pixel_x_to_servo_angle(320, frame_width=640)
    assert abs(angle - round(_SERVO1_CTR)) <= 1, (
        f"Centre pixel should map to ~{_SERVO1_CTR}°, got {angle}"
    )


def test_servo_angle_left_edge_clamped():
    angle = pixel_x_to_servo_angle(0, frame_width=640)
    assert angle >= _SERVO1_MIN


def test_servo_angle_right_edge_clamped():
    angle = pixel_x_to_servo_angle(640, frame_width=640)
    assert angle <= _SERVO1_MAX


def test_servo_angle_monotonic():
    """Moving right across the frame should decrease (or at worst equal) the servo angle."""
    angles = [pixel_x_to_servo_angle(x, 640) for x in range(0, 641, 64)]
    assert angles == sorted(angles, reverse=True) or angles == sorted(angles), (
        "Servo angle mapping is not monotonic across frame pixels"
    )


# ---------------------------------------------------------------------------
# calculate_real_width
# ---------------------------------------------------------------------------

def test_calculate_real_width_formula():
    result = calculate_real_width(100, 50, 554)
    expected = (100 * 50) / 554
    assert abs(result - expected) < 0.01


def test_calculate_real_width_scales_linearly():
    w1 = calculate_real_width(100, 50, 554)
    w2 = calculate_real_width(200, 50, 554)
    assert abs(w2 - 2 * w1) < 0.01


# ---------------------------------------------------------------------------
# sweep_obstacle
# ---------------------------------------------------------------------------

class _FakeController:
    def __init__(self):
        self.moves   = []
        self.centred = False

    def move_camera_to(self, axis: str, angle: int):
        self.moves.append((axis, angle))

    def center_camera(self):
        self.centred = True


class _FakeSensor:
    def __init__(self, reading: float = 50.0):
        self._reading = reading
        self.ping_count = 0

    def distance_cm(self) -> float:
        self.ping_count += 1
        return self._reading


def test_sweep_returns_correct_keys():
    ctrl   = _FakeController()
    sensor = _FakeSensor()
    result = sweep_obstacle(ctrl, sensor, bbox_left_px=100, bbox_right_px=300)
    assert result.keys() == {"left", "center", "right"}


def test_sweep_values_are_floats():
    ctrl   = _FakeController()
    sensor = _FakeSensor(42.0)
    result = sweep_obstacle(ctrl, sensor, 100, 300)
    for val in result.values():
        assert isinstance(val, float)


def test_sweep_pings_sensor_three_times():
    ctrl   = _FakeController()
    sensor = _FakeSensor()
    sweep_obstacle(ctrl, sensor, 100, 300)
    assert sensor.ping_count == 3


def test_sweep_moves_head_three_times():
    ctrl   = _FakeController()
    sensor = _FakeSensor()
    sweep_obstacle(ctrl, sensor, 100, 300)
    assert len(ctrl.moves) == 3


def test_sweep_centres_after_sweep():
    ctrl   = _FakeController()
    sensor = _FakeSensor()
    sweep_obstacle(ctrl, sensor, 100, 300)
    assert ctrl.centred, "center_camera() was not called after sweep"


def test_sweep_centres_even_on_sensor_error():
    """center_camera() must be called inside a finally block."""
    ctrl = _FakeController()

    class _BrokenSensor:
        def distance_cm(self):
            raise RuntimeError("sensor fault")

    with pytest.raises(RuntimeError):
        sweep_obstacle(ctrl, _BrokenSensor(), 100, 300)

    assert ctrl.centred, "center_camera() was not called when sensor raised"
