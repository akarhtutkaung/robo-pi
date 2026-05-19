"""
Rotating 3-position sensor cache for the autonomous clear phase.

Accumulates ultrasonic distances and YOLO detections from left/center/right
head positions and exposes boolean queries used by navigate_step to decide
whether to slow down, avoid, or hard-stop.
"""

import math

from src.core.config import AUTONOMOUS_CFG, SERVO_CFG, OBSTACLE_AVOIDANCE_CFG, ULTRASONIC_CFG

_SERVO1_CENTER   = SERVO_CFG["servo1"]["center_angle"]
_SERVO1_MIN      = SERVO_CFG["servo1"]["max_angle"]     # 0   — full right
_SERVO1_MAX      = SERVO_CFG["servo1"]["min_angle"]     # 180 — full left
_SWEEP_ANGLE_DEG = AUTONOMOUS_CFG["sweep_angle_deg"]
_WARN_CM         = AUTONOMOUS_CFG["warn_cm"]
_STOP_CM         = ULTRASONIC_CFG["stop_cm"]
_ULTRASONIC_HEIGHT_CM = ULTRASONIC_CFG["height_cm"]
_FOCAL_LENGTH_PX = OBSTACLE_AVOIDANCE_CFG["focal_length_px"]
_ROBOT_WIDTH_CM  = OBSTACLE_AVOIDANCE_CFG["robot_width_cm"]
_FRAME_W         = 640

_YOLO_BLOCK_RATIO = max(
    AUTONOMOUS_CFG["yolo_block_ratio"],
    (_ULTRASONIC_HEIGHT_CM * _FOCAL_LENGTH_PX) / (_STOP_CM * _FRAME_W),
)

# (name, servo1 angle) triples traversed left → centre → right each tick cycle
_SWEEP_POSITIONS: list[tuple[str, int]] = [
    ("left",   int(min(_SERVO1_MAX, round(_SERVO1_CENTER + _SWEEP_ANGLE_DEG)))),
    ("center", int(round(_SERVO1_CENTER))),
    ("right",  int(max(_SERVO1_MIN, round(_SERVO1_CENTER - _SWEEP_ANGLE_DEG)))),
]

# At sweep angle θ, a side reading d has lateral offset d·sin(θ). Below this distance
# that offset is less than the robot's half-width → obstacle is inside the body corridor.
_SWEEP_SIDE_CORRIDOR_CM = (_ROBOT_WIDTH_CM / 2.0) / math.sin(math.radians(_SWEEP_ANGLE_DEG))


class _SweepCache:
    """Rotating 3-position sensor cache for the clear phase.

    Each navigate_step call in the clear phase advances to the next position
    (left → center → right → left …), stores ultrasonic distance and YOLO
    detections, and uses the cached data to adapt speed before the forward
    ultrasonic would trigger is_blocked().
    """

    def __init__(self):
        self._idx       = 0
        self.distances  = {"left": None, "center": None, "right": None}
        self.detections: dict[str, list] = {"left": [],   "center": [],   "right": []}
        self.last_error = 0.0
        self.last_conf  = 0.0

    def advance(self) -> tuple[str, int]:
        """Return (name, servo_angle) for the current position and advance the index."""
        name, angle = _SWEEP_POSITIONS[self._idx]
        self._idx = (self._idx + 1) % len(_SWEEP_POSITIONS)
        return name, angle

    def any_side_blocked(self) -> bool:
        """True if any SIDE-direction cached distance is in the stop zone (center excluded).

        Center readings belong to the forward-ultrasonic path (obstacle.is_blocked), which
        applies physics-based smooth deceleration. Including center here would bypass that
        and call force_stop() instead — a hard jerk stop when the obstacle is directly ahead.
        """
        return any(
            d is not None and d <= _STOP_CM
            for name, d in self.distances.items()
            if name != "center"
        )

    def should_slow(self) -> bool:
        """True if sweep data suggests slowing is warranted.

        Conditions (any one triggers slow):
        - YOLO + ultrasonic both agree on obstacle inside warn_cm (normal case)
        - Ultrasonic alone < stop_cm × 1.2 — catches dark/novel objects YOLO misses
        - in_corridor()     — YOLO sees obstacle in robot's body corridor, ultrasonic missed
        - side_in_corridor() — side geometry: obstacle in body path, YOLO also missed
        """
        _ultrasonic_slow_cm = _STOP_CM * 1.2
        for name, dist in self.distances.items():
            if dist is None:
                continue
            if dist < _WARN_CM and self.detections[name]:
                return True
            if dist < _ultrasonic_slow_cm:
                return True
        if self.in_corridor():
            return True
        if self.side_in_corridor():
            return True
        return False

    def in_corridor(self, frame_width: int = _FRAME_W) -> bool:
        """True if any center-frame YOLO detection overlaps the robot's projected body width.

        Uses the pinhole model: at center distance d, the robot's half-width in pixels is
        (robot_half_width_cm × focal_length_px) / d. Any detection whose x-range overlaps
        [cx − half_w_px, cx + half_w_px] is within the robot's physical path.
        Only meaningful after the center tick has been processed.
        """
        dets = self.detections["center"]
        dist = self.distances["center"]
        if not dets or not dist or dist <= 0:
            return False
        half_w_px = (_ROBOT_WIDTH_CM / 2.0 * _FOCAL_LENGTH_PX) / max(dist, 10.0)
        cx = frame_width / 2.0
        return any(d["x1"] < cx + half_w_px and d["x2"] > cx - half_w_px for d in dets)

    def side_in_corridor(self) -> bool:
        """True if any side-sweep ultrasonic reading places an obstacle inside the robot's body.

        At sweep angle θ, a distance d gives lateral offset d·sin(θ). When that is less than
        the robot's half-width the obstacle is geometrically within the collision corridor —
        catches dark/novel objects that neither YOLO nor the center ultrasonic detects.
        """
        for name in ("left", "right"):
            d = self.distances[name]
            if d is not None and 0 < d < _SWEEP_SIDE_CORRIDOR_CM:
                return True
        return False

    def should_avoid_side(self) -> bool:
        """True if a side-sweep reading is inside warn_cm AND YOLO confirms an obstacle there.

        Fills the gap between should_slow() (reduces speed) and any_side_blocked() (hard stop):
        an obstacle 30–warn_cm on the side that YOLO also sees warrants full avoidance, not
        just a speed reduction.
        """
        for name in ("left", "right"):
            d = self.distances[name]
            if d is not None and d < _WARN_CM and self.detections[name]:
                return True
        return False

    def yolo_blocking(self) -> bool:
        """True when a large in-corridor YOLO detection indicates a low obstacle the ultrasonic missed.

        The sensor at _ULTRASONIC_HEIGHT_CM passes over obstacles shorter than itself, returning
        a clear ultrasonic reading while YOLO sees a large bounding box. Guard: only fires when
        ultrasonic reads >= warn_cm so the normal is_blocked()/should_turn() paths are not bypassed.

        Corridor width is computed at stop_cm rather than the (unreliable) ultrasonic distance to
        give a conservative check appropriate for an obstacle assumed to be close.
        """
        dets = self.detections["center"]
        dist = self.distances["center"]
        if not dets or dist is None:
            return False
        if dist < _WARN_CM:
            return False   # normal ultrasonic path owns this distance range
        half_w_px = (_ROBOT_WIDTH_CM / 2.0 * _FOCAL_LENGTH_PX) / _STOP_CM
        cx = _FRAME_W / 2.0
        for det in dets:
            if (det["x2"] - det["x1"]) / _FRAME_W >= _YOLO_BLOCK_RATIO:
                if det["x1"] < cx + half_w_px and det["x2"] > cx - half_w_px:
                    return True
        return False

    def invalidate(self):
        """Reset all cached sensor readings after avoidance completes.

        Prevents stale left/right readings from a previously-passed obstacle from
        re-triggering should_avoid_side() or any_side_blocked() on subsequent ticks.
        """
        self.distances  = {"left": None, "center": None, "right": None}
        self.detections = {"left": [],   "center": [],   "right": []}