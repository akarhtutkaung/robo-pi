"""
Tracking policy: which detected face to follow, and how much to correct
the head servos to bring it toward frame center.

"Lock": once a face is acquired, select_target prefers the detection
closest to the previously-tracked face's center over always re-picking the
largest box, so tracking doesn't flicker between multiple faces in view.
Falls back to the largest box on first acquisition or after losing the
target.

"Smoothing": smooth_center EMA-filters the detected center before it drives
compute_new_angles. The raw box position is noticeably less stable while a
face is actively moving/turning (motion blur, and a pose-tolerant detector's
box shifting asymmetrically as yaw changes) than while it's static — fed
straight into the control loop, that extra noise reads as a rapid stutter
while tracking movement, even though a still face converges and holds
cleanly (see tracker.py, which keeps this smoothed value separate from the
raw target_center used for locking and status reporting).

Angle correction (compute_new_angles) is applied relative to the servo's
*current* angle, not an absolute frame-centered angle — each frame is
captured from wherever the head currently points, unlike
object_detection.py's pixel_x_to_servo_angle (used for the one-shot
obstacle sweep, which starts centered).
"""
import math

from src.components.core.config import (
    CAMERA_CFG, FACE_TRACKING_CFG, FACIAL_TRACKING_CFG, OBSTACLE_AVOIDANCE_CFG, SERVO_CFG,
)

_PAN_GAIN         = FACIAL_TRACKING_CFG["pan_gain"]
_TILT_GAIN        = FACIAL_TRACKING_CFG["tilt_gain"]
_DEAD_ZONE_PX     = FACIAL_TRACKING_CFG["dead_zone_px"]
_MAX_STEP_DEG     = FACIAL_TRACKING_CFG["max_step_deg"]
_SMOOTHING_ALPHA  = FACIAL_TRACKING_CFG["smoothing_alpha"]
_LOCK_MAX_JUMP_PX = FACIAL_TRACKING_CFG["lock_max_jump_px"]
_INVERT_TILT      = FACIAL_TRACKING_CFG["invert_tilt"]

_SERVO1_CFG = SERVO_CFG["servo1"]  # head left/right (pan)
_SERVO2_CFG = SERVO_CFG["servo2"]  # head up/down (tilt)
_SERVO1_CENTER = _SERVO1_CFG["center_angle"]
_SERVO2_CENTER = _SERVO2_CFG["center_angle"]

# obstacle_avoidance.focal_length_px is calibrated for the 640x480 lores frame.
# Detection here runs at frame_width x frame_height instead (tracker.py's
# _capture_and_detect), so each axis's focal length is scaled by that axis's own
# ratio to 640/480 — separately, because frame_width/frame_height can be a
# different aspect ratio than 640x480, and each axis's pixel-per-degree density
# only tracks that axis's own scale factor. atan2(offset_px, focal) then measures
# the same real-world angle per pixel at whatever resolution facial tracking is
# configured to.
_FRAME_W = FACE_TRACKING_CFG["frame_width"]
_FRAME_H = FACE_TRACKING_CFG["frame_height"]
_FOCAL_X_PX = OBSTACLE_AVOIDANCE_CFG["focal_length_px"] * (_FRAME_W / CAMERA_CFG["front"]["lores_width"])
_FOCAL_Y_PX = OBSTACLE_AVOIDANCE_CFG["focal_length_px"] * (_FRAME_H / CAMERA_CFG["front"]["lores_height"])


def face_center(box: dict) -> tuple[float, float]:
    return ((box["x1"] + box["x2"]) / 2.0, (box["y1"] + box["y2"]) / 2.0)


def _face_area(box: dict) -> int:
    return (box["x2"] - box["x1"]) * (box["y2"] - box["y1"])


def bbox_points(box: dict) -> list[list[int]]:
    """Corner points of a face box, clockwise from top-left — for a frontend
    to draw the detection outline: [top-left, top-right, bottom-right, bottom-left].

    detect_faces() already converts OpenCV's (x, y, w, h) rect into this
    {x1,y1,x2,y2} box, so no separate OpenCV call is needed here.
    """
    x1, y1, x2, y2 = box["x1"], box["y1"], box["x2"], box["y2"]
    return [[x1, y1], [x2, y1], [x2, y2], [x1, y2]]


def select_target(faces: list[dict], previous_center: tuple[float, float] | None) -> dict | None:
    """Pick which detected face to track this tick.

    If a previous target exists, prefer the closest match within
    lock_max_jump_px — this is the "lock": keep tracking the same person
    instead of flickering between faces. Otherwise (first acquisition, or
    the previous target moved too far / vanished) fall back to the largest
    box — the closest / most prominent face.
    """
    if not faces:
        return None

    if previous_center is not None:
        px, py = previous_center

        def dist(box: dict) -> float:
            cx, cy = face_center(box)
            return math.hypot(cx - px, cy - py)

        closest = min(faces, key=dist)
        if dist(closest) <= _LOCK_MAX_JUMP_PX:
            return closest

    return max(faces, key=_face_area)


def smooth_center(
    raw_center: tuple[float, float],
    previous_smoothed: tuple[float, float] | None,
) -> tuple[float, float]:
    """Exponential moving average over the detected face center, applied
    before compute_new_angles — see the module docstring's "Smoothing" note.

    previous_smoothed is None on first acquisition (right after
    select_target picks a fresh target) — start from the raw center rather
    than easing in from nothing.
    """
    if previous_smoothed is None:
        return raw_center
    px, py = previous_smoothed
    rx, ry = raw_center
    return (
        px + _SMOOTHING_ALPHA * (rx - px),
        py + _SMOOTHING_ALPHA * (ry - py),
    )


def _clamp_step(current: float, target: float, max_step: float) -> float:
    """Rate-limit current -> target to at most max_step per call.

    Without this, a large offset (e.g. right after acquisition, or a fast
    head turn) produces a correction of gain * atan(offset/focal) in a
    single tick — easily 15-30 deg — snapping the servo instantly rather
    than gliding. The servo overshoots/vibrates settling from the snap, and
    by the time it does the next tick has already re-targeted it again,
    which reads as jiggering rather than smooth tracking. Capping the step
    spreads a big correction over several ticks instead, mirroring how
    motors.py ramps speed rather than snapping it.
    """
    delta = target - current
    if delta > max_step:
        delta = max_step
    elif delta < -max_step:
        delta = -max_step
    return current + delta


def compute_new_angles(
    face_center_px: tuple[float, float],
    current_pan: float,
    current_tilt: float,
) -> tuple[float, float]:
    """Relative pan/tilt correction to bring face_center_px toward frame center.

    Applied relative to the servo's *current* angle — each frame is captured
    from wherever the head currently points, not from a centered reference.
    Per-tick damping (pan_gain/tilt_gain) keeps cascade detection jitter from
    translating into servo twitch; the dead zone stops correcting once
    already close to centered; max_step_deg (_clamp_step) rate-limits how
    far a single tick may move the servo so a large offset is glided toward
    over several ticks instead of snapped to in one.
    """
    cx, cy = face_center_px
    offset_x = cx - _FRAME_W / 2.0
    offset_y = cy - _FRAME_H / 2.0

    new_pan, new_tilt = current_pan, current_tilt

    if abs(offset_x) > _DEAD_ZONE_PX:
        pan_delta_deg = math.degrees(math.atan2(offset_x, _FOCAL_X_PX))
        # servo1: decreasing angle = right (matches pixel_x_to_servo_angle's sign convention)
        target_pan = current_pan - _PAN_GAIN * pan_delta_deg
        new_pan = _clamp_step(current_pan, target_pan, _MAX_STEP_DEG)

    if abs(offset_y) > _DEAD_ZONE_PX:
        tilt_delta_deg = math.degrees(math.atan2(offset_y, _FOCAL_Y_PX))
        sign = 1.0 if _INVERT_TILT else -1.0
        target_tilt = current_tilt + sign * _TILT_GAIN * tilt_delta_deg
        new_tilt = _clamp_step(current_tilt, target_tilt, _MAX_STEP_DEG)

    new_pan = max(_SERVO1_CFG["max_angle"], min(_SERVO1_CFG["min_angle"], new_pan))
    new_tilt = max(_SERVO2_CFG["max_angle"], min(_SERVO2_CFG["min_angle"], new_tilt))
    return new_pan, new_tilt
