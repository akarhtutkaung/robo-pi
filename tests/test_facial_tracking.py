"""
Tests for src/features/facial_tracking/ (detector.py, targeting.py, and
track_step's hardware-write hysteresis in tracker.py).

Hardware-free: no RobotController, no camera. Detection/geometry functions
are exercised directly against synthetic inputs; track_step tests use a
MagicMock controller and a monkeypatched capture step.
"""
import asyncio
import time
from unittest.mock import MagicMock

import numpy as np
import pytest

from src.features.facial_tracking.detector import (
    detect_faces,
    detector_name,
    _nms,
    _resolve_cascade_path,
    _CascadeBackend,
    _FRONTAL_CASCADE_FILE,
    _PROFILE_CASCADE_FILE,
)
from src.features.facial_tracking.targeting import (
    face_center,
    _face_area,
    bbox_points,
    select_target,
    compute_new_angles,
    smooth_center,
    _FRAME_W,
    _FRAME_H,
    _SERVO1_CENTER,
    _SERVO2_CENTER,
    _DEAD_ZONE_PX,
    _MAX_STEP_DEG,
    _SMOOTHING_ALPHA,
)
from src.features.facial_tracking import tracker as tracker_mod
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


def test_detect_faces_accepts_back_camera_resolution():
    # YuNet is stateful about input size — a differently-sized frame must not raise.
    assert detect_faces(np.zeros((240, 320, 3), dtype=np.uint8)) == []


def test_detector_name_is_a_known_backend():
    assert detector_name() in ("yunet", "cascade")


# ---------------------------------------------------------------------------
# cascade path resolution + ensemble fallback
# ---------------------------------------------------------------------------

def test_resolve_cascade_path_finds_frontal_and_profile():
    # No override configured in this repo's hardware.yaml — must fall through
    # to cv2.data.haarcascades or a common apt path and find real files. The
    # profile cascade is what recovers faces turned left/right.
    import pathlib
    for filename in (_FRONTAL_CASCADE_FILE, _PROFILE_CASCADE_FILE):
        assert pathlib.Path(_resolve_cascade_path(filename)).is_file()


def test_resolve_cascade_path_honours_override():
    assert _resolve_cascade_path(_FRONTAL_CASCADE_FILE, "/some/explicit/path.xml") == "/some/explicit/path.xml"


def test_cascade_backend_runs_all_three_passes():
    # frontal + profile + mirrored profile — the mirrored pass is what catches
    # faces turned the direction haarcascade_profileface doesn't fire on.
    backend = _CascadeBackend()
    assert backend._profile is not None, "profile cascade missing — yaw coverage lost"
    assert backend.detect(np.zeros((480, 640, 3), dtype=np.uint8)) == []


# ---------------------------------------------------------------------------
# _nms — merging the cascade ensemble's overlapping passes
# ---------------------------------------------------------------------------

def test_nms_empty_input():
    assert _nms([], [], 0.3) == []


def test_nms_merges_overlapping_boxes_keeping_higher_score():
    # Same face found by both the frontal and the profile pass — one box out,
    # and it should be the frontal (higher score) one.
    frontal = [100, 100, 100, 100]
    profile = [105, 103, 100, 100]
    merged = _nms([profile, frontal], [0.9, 1.0], 0.3)
    assert len(merged) == 1
    assert merged[0]["x1"] == 100 and merged[0]["score"] == 1.0


def test_nms_keeps_distinct_faces():
    a = [0, 0, 80, 80]
    b = [400, 300, 80, 80]
    merged = _nms([a, b], [1.0, 1.0], 0.3)
    assert len(merged) == 2


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


def test_compute_new_angles_limits_large_correction_to_max_step_per_tick():
    # A far off-center offset produces a correction many times max_step_deg in
    # one tick (gain * atan(offset/focal) can be 15-30deg) — the rate limit is
    # what caps it to exactly max_step_deg. The offset is many frame-widths out
    # so the raw atan2-based correction comfortably exceeds max_step_deg across
    # any focal length / gain / max_step_deg combination. Starting well away
    # from the servo's own range limit isolates the step clamp from the
    # separate range clamp covered above.
    extreme_offset = 50 * _FRAME_W
    face_far_right = (_FRAME_W / 2.0 + extreme_offset, _FRAME_H / 2.0)
    new_pan, _ = compute_new_angles(face_far_right, _SERVO1_CENTER, _SERVO2_CENTER)
    assert abs(new_pan - _SERVO1_CENTER) == pytest.approx(_MAX_STEP_DEG)

    face_far_bottom = (_FRAME_W / 2.0, _FRAME_H / 2.0 + extreme_offset)
    _, new_tilt = compute_new_angles(face_far_bottom, _SERVO1_CENTER, _SERVO2_CENTER)
    assert abs(new_tilt - _SERVO2_CENTER) == pytest.approx(_MAX_STEP_DEG)


# ---------------------------------------------------------------------------
# track_step — min_step_deg hysteresis on the actual hardware write
# ---------------------------------------------------------------------------

def _face_box(cx: float, cy: float, size: float = 80.0) -> dict:
    half = size / 2
    return {"x1": cx - half, "y1": cy - half, "x2": cx + half, "y2": cy + half, "score": 0.9}


def _run_track_step(monkeypatch, faces, state=None):
    if state is None:
        state = tracker_mod._TrackerState(tracker_mod._SERVO1_CENTER, tracker_mod._SERVO2_CENTER)
    monkeypatch.setattr(tracker_mod, "_capture_and_detect", lambda camera: faces)
    controller = MagicMock()
    asyncio.run(tracker_mod.track_step(controller, camera=None, state=state))
    return controller, state


def test_track_step_skips_write_for_sub_min_step_correction(monkeypatch):
    # Offset just past dead_zone_px (21px) produces ~2.78deg of correction —
    # outside the dead zone, but below min_step_deg (3) — see the comment on
    # min_step_deg in modes.yaml for why it must exceed the dead-zone-boundary
    # correction to have any effect at all.
    face = _face_box(_FRAME_W / 2.0 + _DEAD_ZONE_PX + 1, _FRAME_H / 2.0)
    controller, state = _run_track_step(monkeypatch, [face])
    controller.move_camera_to.assert_not_called()
    # Software state still holds at the un-commanded angle — not left stale
    # relative to what's actually on the servo.
    assert state.pan == _SERVO1_CENTER


def test_track_step_writes_for_correction_past_min_step(monkeypatch):
    # A clearly larger offset (30px) produces ~3.96deg — past min_step_deg.
    face = _face_box(_FRAME_W / 2.0 + 30, _FRAME_H / 2.0)
    controller, state = _run_track_step(monkeypatch, [face])
    controller.move_camera_to.assert_called_once()
    axis, angle = controller.move_camera_to.call_args[0]
    assert axis == "x"
    assert state.pan != _SERVO1_CENTER
    assert angle == int(round(state.pan))  # what's written matches what's tracked


# ---------------------------------------------------------------------------
# smooth_center — EMA filter that damps the raw detection box's frame-to-frame
# instability (worse while a face is moving/turning) before it reaches
# compute_new_angles
# ---------------------------------------------------------------------------

def test_smooth_center_first_reading_passes_through_unfiltered():
    # previous_smoothed=None means this is a fresh acquisition — start exactly
    # at the raw center rather than easing in from some assumed prior point.
    raw = (123.0, 45.0)
    assert smooth_center(raw, None) == raw


def test_smooth_center_moves_toward_raw_by_alpha():
    previous = (100.0, 100.0)
    raw = (200.0, 100.0)
    smoothed_x, smoothed_y = smooth_center(raw, previous)
    assert smoothed_x == pytest.approx(100.0 + _SMOOTHING_ALPHA * 100.0)
    assert smoothed_y == pytest.approx(100.0)


def test_smooth_center_damps_single_frame_noise_spike():
    # A steady previous value plus one noisy outlier reading should land much
    # closer to the steady value than the raw jump does — this is the whole
    # point: one bad detection shouldn't swing the servo target by its full size.
    previous = (160.0, 120.0)
    noisy_outlier = (260.0, 120.0)
    smoothed_x, _ = smooth_center(noisy_outlier, previous)
    raw_jump = abs(noisy_outlier[0] - previous[0])
    smoothed_jump = abs(smoothed_x - previous[0])
    assert smoothed_jump < raw_jump


def test_smooth_center_converges_to_steady_raw_value_over_repeated_ticks():
    previous = None
    raw = (300.0, 200.0)
    for _ in range(50):
        previous = smooth_center(raw, previous)
    assert previous == pytest.approx(raw)


def test_track_step_resets_smoothing_after_target_lost_and_recentered(monkeypatch):
    # A stale smoothed_center from a previous target must not leak into a
    # fresh acquisition after the lost-face recenter — this drives one state
    # object through both phases rather than using the fresh-state helper.
    state = tracker_mod._TrackerState(tracker_mod._SERVO1_CENTER, tracker_mod._SERVO2_CENTER)
    face = _face_box(_FRAME_W / 2.0 + 30, _FRAME_H / 2.0)
    controller, state = _run_track_step(monkeypatch, [face], state=state)
    assert state.smoothed_center is not None

    # Face vanishes and the recenter timeout has already elapsed.
    monkeypatch.setattr(tracker_mod, "_capture_and_detect", lambda camera: [])
    state.last_seen = time.monotonic() - tracker_mod._LOST_RECENTER_S - 1
    asyncio.run(tracker_mod.track_step(controller, camera=None, state=state))
    assert state.smoothed_center is None
