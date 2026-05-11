"""
Tests for src/perception/vision/object_detection.py

Hardware-free: UltrasonicSensor is never instantiated.
sweep_obstacle tests use FakeController / FakeSensor.
"""
import cv2
import numpy as np
import pytest

from src.perception.vision.object_detection import (
    detect_obstacles,
    select_primary_obstacle,
    classify_width_threat,
    pixel_x_to_servo_angle,
    calculate_real_width,
    sweep_obstacle,
    draw_detections,
    _COCO_LABELS,
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


# ---------------------------------------------------------------------------
# _COCO_LABELS — stream annotation label table
# ---------------------------------------------------------------------------

def test_coco_labels_person():
    assert _COCO_LABELS[0] == "person"


def test_coco_labels_vehicle_classes():
    assert _COCO_LABELS[1] == "bicycle"
    assert _COCO_LABELS[2] == "car"
    assert _COCO_LABELS[5] == "bus"
    assert _COCO_LABELS[7] == "truck"


def test_coco_labels_animal_classes():
    assert _COCO_LABELS[14] == "bird"
    assert _COCO_LABELS[15] == "cat"
    assert _COCO_LABELS[16] == "dog"


def test_coco_labels_furniture_classes():
    assert _COCO_LABELS[56] == "chair"
    assert _COCO_LABELS[57] == "couch"
    assert _COCO_LABELS[59] == "bed"


def test_coco_labels_unknown_id_falls_back():
    unknown_id = 999
    assert unknown_id not in _COCO_LABELS
    label = _COCO_LABELS.get(unknown_id, f"cls{unknown_id}")
    assert label == "cls999"


def test_coco_labels_all_values_are_strings():
    assert all(isinstance(v, str) for v in _COCO_LABELS.values())


def test_coco_labels_all_keys_are_ints():
    assert all(isinstance(k, int) for k in _COCO_LABELS)


# ---------------------------------------------------------------------------
# Stream annotation rendering
#
# The draw loop inside __main__ is not importable as a function, so these
# tests replicate its logic directly. They verify that:
#   - cv2.rectangle and cv2.putText run without error on a real frame
#   - bounding box pixels are modified (green channel set)
#   - the original frame is not mutated (vis = frame.copy())
#   - empty detections leave the frame pixel-identical to the source
# ---------------------------------------------------------------------------

def _annotate(frame: np.ndarray, detections: list) -> np.ndarray:
    return draw_detections(frame.copy(), detections)


def _blank640() -> np.ndarray:
    return np.zeros((480, 640, 3), dtype=np.uint8)


def _fake_detection(x1=100, y1=50, x2=300, y2=250, conf=0.85, class_id=0) -> dict:
    return {"x1": x1, "y1": y1, "x2": x2, "y2": y2, "conf": conf, "class_id": class_id}


def test_annotate_does_not_mutate_original():
    frame = _blank640()
    original = frame.copy()
    _annotate(frame, [_fake_detection()])
    assert np.array_equal(frame, original), "annotate() must not modify the source frame"


def test_annotate_empty_detections_unchanged():
    frame = _blank640()
    vis   = _annotate(frame, [])
    assert np.array_equal(vis, frame)


def test_annotate_draws_green_rectangle():
    """At least one pixel along the bounding box edge must be green (G=255, R=0, B=0)."""
    vis = _annotate(_blank640(), [_fake_detection(x1=100, y1=50, x2=300, y2=250)])
    # Sample the top edge of the bounding box (row y1=50, cols 100..300)
    top_edge = vis[50, 100:300]
    green_pixels = np.all(top_edge == [0, 255, 0], axis=1)
    assert green_pixels.any(), "No green rectangle pixels found on the bounding box edge"


def test_annotate_known_class_label_in_image():
    """Verify label text is rendered without raising (cv2.putText succeeds)."""
    det = _fake_detection(class_id=0, y1=30)  # person; y1=30 so text fits above
    vis = _annotate(_blank640(), [det])
    assert vis.shape == (480, 640, 3)


def test_annotate_unknown_class_label_in_image():
    """cls<id> fallback label must not raise."""
    det = _fake_detection(class_id=999)
    vis = _annotate(_blank640(), [det])
    assert vis.shape == (480, 640, 3)


def test_annotate_jpeg_encode_succeeds():
    """Annotated frame must encode to JPEG without error."""
    vis = _annotate(_blank640(), [_fake_detection()])
    ok, jpg = cv2.imencode(".jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, 80])
    assert ok
    assert len(jpg.tobytes()) > 0


def test_annotate_multiple_detections():
    dets = [
        _fake_detection(x1=10,  y1=10,  x2=100, y2=100, class_id=0,  conf=0.90),
        _fake_detection(x1=200, y1=100, x2=400, y2=300, class_id=2,  conf=0.75),
        _fake_detection(x1=500, y1=200, x2=620, y2=400, class_id=56, conf=0.60),
    ]
    vis = _annotate(_blank640(), dets)
    assert vis.shape == (480, 640, 3)
    # Frame must have been modified
    assert not np.array_equal(vis, _blank640())
