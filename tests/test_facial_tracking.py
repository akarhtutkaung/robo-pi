"""
Tests for src/features/facial_tracking/ (detector.py, targeting.py).

Hardware-free: no RobotController, no camera. Detection/geometry functions
are exercised directly against synthetic inputs.
"""
import numpy as np
import pytest

from src.features.facial_tracking.detector import (
    detect_faces,
    _resolve_cascade_path,
)
from src.features.facial_tracking.targeting import (
    face_center,
    _face_area,
    bbox_points,
    select_target,
    compute_new_angles,
    _FRAME_W,
    _FRAME_H,
    _SERVO1_CENTER,
    _SERVO2_CENTER,
    _DEAD_ZONE_PX,
)
from src.components.core.config import SERVO_CFG

_SERVO1_MIN = SERVO_CFG["servo1"]["max_angle"]   # 0   — full right (smallest angle)
_SERVO1_MAX = SERVO_CFG["servo1"]["min_angle"]   # 180 — full left  (largest angle)
_SERVO2_MIN = SERVO_CFG["servo2"]["max_angle"]
_SERVO2_MAX = SERVO_CFG["servo2"]["min_angle"]


# ---------------------------------------------------------------------------
# detect_faces — blank / None frames must never raise or false-positive
# ---------------------------------------------------------------------------

def test_detect_faces_blank_frame_returns_empty():
    blank = np.zeros((480, 640, 3), dtype=np.uint8)
    assert detect_faces(blank) == []


# ---------------------------------------------------------------------------
# cascade path resolution
# ---------------------------------------------------------------------------

def test_resolve_cascade_path_finds_a_file():
    # No override configured in this repo's hardware.yaml — must fall through
    # to cv2.data.haarcascades or a common apt path and find a real file.
    import pathlib
    path = _resolve_cascade_path()
    assert pathlib.Path(path).is_file()


# ---------------------------------------------------------------------------
# face_center / _face_area
# ---------------------------------------------------------------------------

def test_face_center_and_area():
    box = {"x1": 100, "y1": 50, "x2": 200, "y2": 150}
    assert face_center(box) == (150.0, 100.0)
    assert _face_area(box) == 100 * 100


def test_bbox_points_clockwise_from_top_left():
    box = {"x1": 100, "y1": 50, "x2": 200, "y2": 150}
    assert bbox_points(box) == [
        [100, 50],   # top-left
        [200, 50],   # top-right
        [200, 150],  # bottom-right
        [100, 150],  # bottom-left
    ]


# ---------------------------------------------------------------------------
# select_target — "lock" behavior
# ---------------------------------------------------------------------------

def test_select_target_no_faces_returns_none():
    assert select_target([], None) is None


def test_select_target_first_acquisition_picks_largest():
    small = {"x1": 0, "y1": 0, "x2": 50, "y2": 50}      # area 2500
    large = {"x1": 200, "y1": 200, "x2": 300, "y2": 300}  # area 10000
    assert select_target([small, large], None) is large


def test_select_target_locks_onto_closest_when_previous_exists():
    # Two faces; previous target was near the left one. Even though the
    # right one might be bigger, lock-on should prefer proximity.
    left  = {"x1": 90,  "y1": 90,  "x2": 110, "y2": 110}   # center (100, 100)
    right = {"x1": 490, "y1": 90,  "x2": 600, "y2": 200}   # much bigger, center (545, 145)
    previous_center = (105.0, 105.0)  # close to `left`
    assert select_target([left, right], previous_center) is left


def test_select_target_falls_back_to_largest_when_previous_too_far():
    # Previous target is nowhere near either detected face this tick —
    # both jumps exceed lock_max_jump_px — so fall back to the largest box.
    small = {"x1": 0, "y1": 0, "x2": 20, "y2": 20}
    large = {"x1": 300, "y1": 300, "x2": 400, "y2": 400}
    previous_center = (5000.0, 5000.0)  # far from both — well beyond lock_max_jump_px
    assert select_target([small, large], previous_center) is large


# ---------------------------------------------------------------------------
# compute_new_angles — dead zone, direction sign, clamping
# ---------------------------------------------------------------------------

def test_compute_new_angles_dead_zone_no_op_when_centered():
    center = (_FRAME_W / 2.0, _FRAME_H / 2.0)
    new_pan, new_tilt = compute_new_angles(center, _SERVO1_CENTER, _SERVO2_CENTER)
    assert new_pan == _SERVO1_CENTER
    assert new_tilt == _SERVO2_CENTER


def test_compute_new_angles_face_right_of_center_turns_pan_right():
    # Face well to the right of frame center (offset beyond dead zone).
    face = (_FRAME_W / 2.0 + _DEAD_ZONE_PX + 50, _FRAME_H / 2.0)
    new_pan, _ = compute_new_angles(face, _SERVO1_CENTER, _SERVO2_CENTER)
    # servo1: decreasing angle = right (matches pixel_x_to_servo_angle's convention)
    assert new_pan < _SERVO1_CENTER


def test_compute_new_angles_face_left_of_center_turns_pan_left():
    face = (_FRAME_W / 2.0 - _DEAD_ZONE_PX - 50, _FRAME_H / 2.0)
    new_pan, _ = compute_new_angles(face, _SERVO1_CENTER, _SERVO2_CENTER)
    assert new_pan > _SERVO1_CENTER


def test_compute_new_angles_clamps_to_servo_range():
    # Start already near the boundary so the (damped) correction would overshoot
    # past the servo's physical range without the clamp — isolates the clamp
    # itself rather than relying on a single tick's gain being large enough to
    # reach the boundary from center.
    face_at_right_edge = (_FRAME_W, _FRAME_H / 2.0)
    new_pan, _ = compute_new_angles(face_at_right_edge, _SERVO1_MIN + 2, _SERVO2_CENTER)
    assert new_pan == _SERVO1_MIN

    face_at_top_edge = (_FRAME_W / 2.0, 0.0)
    _, new_tilt = compute_new_angles(face_at_top_edge, _SERVO1_CENTER, _SERVO2_MAX - 2)
    assert new_tilt == _SERVO2_MAX
