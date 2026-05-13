"""
Tests for src/core/modes/autonomous.py

Covers pure logic (decide_avoidance, _direction_label) and the full
navigate_step() pipeline with all hardware mocked at the module level.
No Pi hardware is touched.
"""
import asyncio
import contextlib
import json
import numpy as np
import pytest

import src.core.modes.autonomous as auto_mod
from src.core.modes.autonomous import (
    decide_avoidance,
    _direction_label,
    AUTONOMOUS_SPEED,
    REVERSE_SPEED,
    APPROACH_SPEED,
    _STEER_LEFT,
    _STEER_RIGHT,
    _SweepCache,
    _STOP_CM,
    _WARN_CM,
    _SWEEP_POSITIONS,
    _FOCAL_LENGTH_PX,
    _ROBOT_WIDTH_CM,
    _SWEEP_SIDE_CORRIDOR_CM,
    _FRAME_W,
)


# ---------------------------------------------------------------------------
# Helpers shared by navigate_step tests
# ---------------------------------------------------------------------------

def _blank_frame() -> np.ndarray:
    return np.zeros((480, 640, 3), dtype=np.uint8)


class _Sensor:
    """Fake ultrasonic sensor — distance_cm() is now called directly in the clear phase."""
    def __init__(self, reading: float = 200.0):
        self._reading = reading
    def distance_cm(self) -> float:
        return self._reading


class _Obstacle:
    def __init__(self, blocked=False, sudden=False, turn=False, dist=50.0,
                 sensor_dist: float = 200.0):
        self._blocked = blocked
        self._sudden  = sudden
        self._turn    = turn
        self._dist    = dist
        self.sensor   = _Sensor(sensor_dist)

    def is_blocked(self)     : return self._blocked
    def is_sudden_stop(self) : return self._sudden
    def should_turn(self)    : return self._turn
    def distance_cm(self)    : return self._dist


class _Controller:
    def __init__(self, speed=0.0):
        self.calls         = []
        self.current_speed = speed

    def forward(self, spd):              self.calls.append(("forward", spd))
    def backward(self, spd):             self.calls.append(("backward", spd))
    def steer(self, angle):              self.calls.append(("steer", angle))
    def steer_center(self):              self.calls.append(("steer_center",))
    def force_stop(self):                self.calls.append(("force_stop",))
    def move_camera_to(self, axis, ang): self.calls.append(("move_camera_to", axis, ang))
    async def smooth_stop(self, rate=None): self.calls.append(("smooth_stop",))


class _Camera:
    def use_back(self):  pass
    def use_front(self): pass

    @contextlib.asynccontextmanager
    async def reverse_cam(self):
        self.use_back()
        try:
            yield
        finally:
            self.use_front()


class _WS:
    def __init__(self):
        self.msgs = []

    async def send(self, payload: str):
        self.msgs.append(json.loads(payload))

    def phases(self):
        return [m["phase"] for m in self.msgs]


def _run(coro):
    return asyncio.run(coro)


# ---------------------------------------------------------------------------
# decide_avoidance — pure function, no hardware
# ---------------------------------------------------------------------------

class TestDecideAvoidance:
    def test_wide_always_reverses(self):
        for sweep in [
            {"left": 100, "center": 50, "right": 100},
            {"left": 0,   "center": 0,  "right": 0},
        ]:
            assert decide_avoidance("WIDE", sweep) == "REVERSE_AND_TURN"

    def test_narrow_left_side_larger(self):
        sweep = {"left": 80, "center": 40, "right": 20}
        assert decide_avoidance("NARROW", sweep) == "TURN_LEFT"

    def test_narrow_right_side_larger(self):
        sweep = {"left": 20, "center": 40, "right": 80}
        assert decide_avoidance("NARROW", sweep) == "TURN_RIGHT"

    def test_narrow_equal_sides_picks_left(self):
        # left >= right tie-break → TURN_LEFT
        sweep = {"left": 50, "center": 40, "right": 50}
        assert decide_avoidance("NARROW", sweep) == "TURN_LEFT"

    def test_medium_both_sides_below_clearance(self):
        # default: robot_width=20, clearance=10 → threshold=30 cm
        sweep = {"left": 15, "center": 10, "right": 15}
        assert decide_avoidance("MEDIUM", sweep) == "REVERSE_AND_TURN"

    def test_medium_left_side_clear(self):
        sweep = {"left": 50, "center": 30, "right": 10}
        assert decide_avoidance("MEDIUM", sweep) == "TURN_LEFT"

    def test_medium_right_side_clear(self):
        sweep = {"left": 10, "center": 30, "right": 50}
        assert decide_avoidance("MEDIUM", sweep) == "TURN_RIGHT"

    def test_medium_right_beats_threshold_left_does_not(self):
        sweep = {"left": 5, "center": 40, "right": 40}
        assert decide_avoidance("MEDIUM", sweep) == "TURN_RIGHT"


# ---------------------------------------------------------------------------
# _direction_label — pure function
# ---------------------------------------------------------------------------

class TestDirectionLabel:
    def test_centre_zero(self):
        assert _direction_label(0.0) == "CENTER"

    def test_centre_near_zero(self):
        assert _direction_label(0.05)  == "CENTER"
        assert _direction_label(-0.05) == "CENTER"

    def test_slight_right(self):
        assert _direction_label(0.15) == "SLIGHT RIGHT"

    def test_slight_left(self):
        assert _direction_label(-0.15) == "SLIGHT LEFT"

    def test_plain_right(self):
        assert _direction_label(0.4) == "RIGHT"

    def test_plain_left(self):
        assert _direction_label(-0.4) == "LEFT"

    def test_hard_right(self):
        assert _direction_label(0.9) == "HARD RIGHT"

    def test_hard_left(self):
        assert _direction_label(-0.9) == "HARD LEFT"

    def test_exact_boundary_slight(self):
        # 0.08 < 0.25 → SLIGHT
        assert _direction_label(0.08) == "SLIGHT RIGHT"


# ---------------------------------------------------------------------------
# _SweepCache — pure logic, no hardware
# ---------------------------------------------------------------------------

class TestSweepCache:
    def test_advance_cycles_left_center_right(self):
        cache = _SweepCache()
        names = [cache.advance()[0] for _ in range(6)]
        assert names == ["left", "center", "right", "left", "center", "right"]

    def test_advance_returns_servo_angle(self):
        cache = _SweepCache()
        _, angle = cache.advance()  # first position: left
        assert isinstance(angle, int)
        assert 0 <= angle <= 180

    def test_any_side_blocked_false_initially(self):
        assert not _SweepCache().any_side_blocked()

    def test_any_side_blocked_when_distance_at_threshold(self):
        cache = _SweepCache()
        cache.distances["left"] = float(_STOP_CM)  # exactly at threshold (<=)
        assert cache.any_side_blocked()

    def test_any_side_blocked_when_distance_below_threshold(self):
        cache = _SweepCache()
        cache.distances["right"] = _STOP_CM - 5
        assert cache.any_side_blocked()

    def test_any_side_blocked_false_when_all_clear(self):
        cache = _SweepCache()
        cache.distances = {"left": 100.0, "center": 150.0, "right": 200.0}
        assert not cache.any_side_blocked()

    def test_any_side_blocked_center_distance_ignored(self):
        """Center distance below stop_cm must not trigger — forward ultrasonic owns that axis."""
        cache = _SweepCache()
        cache.distances["center"] = _STOP_CM - 5
        assert not cache.any_side_blocked()

    def test_should_slow_requires_yolo_and_distance(self):
        cache = _SweepCache()
        cache.distances["left"]  = _WARN_CM - 10   # inside warn zone
        cache.detections["left"] = [{"x1": 0, "y1": 0, "x2": 100, "y2": 100}]
        assert cache.should_slow()

    def test_should_slow_no_yolo_but_above_ultrasonic_threshold(self):
        # _WARN_CM - 10 = 50 cm; ultrasonic-only threshold = _STOP_CM * 1.2 = 36 cm
        # 50 > 36 → YOLO required at this distance; no YOLO → no slow
        cache = _SweepCache()
        cache.distances["right"] = _WARN_CM - 10
        cache.detections["right"] = []
        assert not cache.should_slow()

    def test_should_slow_ultrasonic_alone_triggers_below_threshold(self):
        # Any distance < stop_cm * 1.2 slows even without YOLO (dark/novel objects)
        cache = _SweepCache()
        cache.distances["left"] = _STOP_CM * 1.2 - 1   # just inside threshold
        cache.detections["left"] = []
        assert cache.should_slow()

    def test_should_slow_false_outside_both_thresholds(self):
        cache = _SweepCache()
        cache.distances["center"] = _WARN_CM + 10  # outside warn zone
        cache.detections["center"] = [{"x1": 0, "y1": 0, "x2": 100, "y2": 100}]
        assert not cache.should_slow()

    def test_sweep_positions_span_all_three_names(self):
        names = {name for name, _ in _SWEEP_POSITIONS}
        assert names == {"left", "center", "right"}

    def test_left_angle_greater_than_right_angle(self):
        angles = {name: angle for name, angle in _SWEEP_POSITIONS}
        # For servo1: left = larger angle, right = smaller angle
        assert angles["left"] > angles["right"]


# ---------------------------------------------------------------------------
# navigate_step — full pipeline mocked at module level
# ---------------------------------------------------------------------------

class TestNavigateStep:
    """
    Module-level attributes (detect_obstacles, detect, capture_bgr, etc.) are
    patched directly on auto_mod so LOAD_GLOBAL inside navigate_step picks up
    the stubs. Each test saves and restores the originals via a fixture.
    """

    @pytest.fixture(autouse=True)
    def restore_module_attrs(self):
        originals = {
            "capture_bgr":      auto_mod.capture_bgr,
            "detect":           auto_mod.detect,
            "detect_obstacles": auto_mod.detect_obstacles,
            "sweep_obstacle":   auto_mod.sweep_obstacle,
            "execute_avoidance": auto_mod.execute_avoidance,
        }
        yield
        for name, val in originals.items():
            setattr(auto_mod, name, val)

    # -----------------------------------------------------------------------

    def _cache(self):
        return _SweepCache()

    def test_approaching_sends_correct_phase(self):
        obs  = _Obstacle(turn=True)
        ctrl = _Controller()
        ws   = _WS()
        _run(auto_mod.navigate_step(ctrl, obs, _Camera(), ws, self._cache()))
        assert ws.phases()[-1] == "approaching"

    def test_approaching_drives_at_approach_speed(self):
        obs  = _Obstacle(turn=True)
        ctrl = _Controller()
        ws   = _WS()
        _run(auto_mod.navigate_step(ctrl, obs, _Camera(), ws, self._cache()))
        assert ("forward", APPROACH_SPEED) in ctrl.calls

    def test_clear_phase_steers_and_drives(self):
        obs  = _Obstacle()
        ctrl = _Controller()
        ws   = _WS()
        auto_mod.capture_bgr      = lambda cam: _blank_frame()
        auto_mod.detect           = lambda f: (0.0, 0.6)
        auto_mod.detect_obstacles = lambda f: []

        _run(auto_mod.navigate_step(ctrl, obs, _Camera(), ws, self._cache()))

        assert ws.phases()[-1] == "clear"
        assert any(c[0] == "forward" for c in ctrl.calls)

    def test_clear_phase_low_confidence_centres_steering(self):
        obs  = _Obstacle()
        ctrl = _Controller()
        ws   = _WS()
        auto_mod.capture_bgr      = lambda cam: _blank_frame()
        auto_mod.detect           = lambda f: (0.0, 0.1)  # below MIN_CONFIDENCE
        auto_mod.detect_obstacles = lambda f: []
        cache = self._cache()
        cache._idx = 1  # force center tick so free-space/steer path runs

        _run(auto_mod.navigate_step(ctrl, obs, _Camera(), ws, cache))

        assert ("steer_center",) in ctrl.calls

    def test_clear_moves_head_to_sweep_position(self):
        obs  = _Obstacle()
        ctrl = _Controller()
        ws   = _WS()
        auto_mod.capture_bgr      = lambda cam: _blank_frame()
        auto_mod.detect           = lambda f: (0.0, 0.6)
        auto_mod.detect_obstacles = lambda f: []

        _run(auto_mod.navigate_step(ctrl, obs, _Camera(), ws, self._cache()))

        assert any(c[0] == "move_camera_to" for c in ctrl.calls)

    def test_clear_caches_sensor_reading(self):
        obs  = _Obstacle(sensor_dist=120.0)
        ctrl = _Controller()
        ws   = _WS()
        auto_mod.capture_bgr      = lambda cam: _blank_frame()
        auto_mod.detect           = lambda f: (0.0, 0.6)
        auto_mod.detect_obstacles = lambda f: []
        cache = self._cache()

        _run(auto_mod.navigate_step(ctrl, obs, _Camera(), ws, cache))

        filled = [d for d in cache.distances.values() if d is not None]
        assert len(filled) == 1
        assert filled[0] == 120.0

    def test_clear_slows_when_yolo_and_distance_warn(self):
        """YOLO detection + distance inside warn zone → APPROACH_SPEED."""
        obs  = _Obstacle(sensor_dist=40.0)  # < _WARN_CM (60)
        ctrl = _Controller()
        ws   = _WS()
        auto_mod.capture_bgr      = lambda cam: _blank_frame()
        auto_mod.detect           = lambda f: (0.0, 0.6)
        auto_mod.detect_obstacles = lambda f: [
            {"x1": 100, "y1": 50, "x2": 300, "y2": 200, "conf": 0.85, "class_id": 0}
        ]

        _run(auto_mod.navigate_step(ctrl, obs, _Camera(), ws, self._cache()))

        assert ("forward", APPROACH_SPEED) in ctrl.calls

    def test_clear_full_speed_when_no_yolo(self):
        """YOLO returns nothing → AUTONOMOUS_SPEED when distance is above all slow thresholds."""
        # 50 cm: inside warn_cm (60) but above ultrasonic-alone threshold (stop_cm*1.2 = 36)
        # and above side corridor threshold (~29 cm) → no slow condition fires
        obs  = _Obstacle(sensor_dist=50.0)
        ctrl = _Controller()
        ws   = _WS()
        auto_mod.capture_bgr      = lambda cam: _blank_frame()
        auto_mod.detect           = lambda f: (0.0, 0.6)
        auto_mod.detect_obstacles = lambda f: []

        _run(auto_mod.navigate_step(ctrl, obs, _Camera(), ws, self._cache()))

        assert ("forward", AUTONOMOUS_SPEED) in ctrl.calls

    def test_side_blocked_triggers_force_stop_and_recentre(self):
        """A cached side distance <= STOP_CM should force_stop + re-centre head + steer away."""
        obs  = _Obstacle()  # forward sensor clear
        ctrl = _Controller()
        ws   = _WS()
        auto_mod.capture_bgr       = lambda cam: _blank_frame()
        auto_mod.detect_obstacles  = lambda f: []
        auto_mod.detect            = lambda f: (0.0, 0.0)
        auto_mod.execute_avoidance = lambda c, cam, dec: asyncio.sleep(0)
        cache = self._cache()
        # Pre-load a stop-zone reading (simulates a previous tick's sweep result)
        cache.distances["right"] = _STOP_CM - 1

        _run(auto_mod.navigate_step(ctrl, obs, _Camera(), ws, cache))

        assert ("force_stop",) in ctrl.calls
        head_moves = [c for c in ctrl.calls if c[0] == "move_camera_to"]
        assert head_moves, "move_camera_to not called to re-centre head"

    def test_sudden_stop_calls_force_stop(self):
        obs  = _Obstacle(blocked=True, sudden=True, dist=5.0)
        ctrl = _Controller()
        ws   = _WS()
        auto_mod.capture_bgr      = lambda cam: _blank_frame()
        auto_mod.detect_obstacles = lambda f: []
        auto_mod.detect           = lambda f: (0.0, 0.0)

        _run(auto_mod.navigate_step(ctrl, obs, _Camera(), ws, self._cache()))

        assert ("force_stop",) in ctrl.calls

    def test_blocked_yolo_path_calls_sweep(self):
        """When YOLO returns a detection, sweep_obstacle must be called."""
        obs   = _Obstacle(blocked=True, dist=25.0)
        ctrl  = _Controller()
        ws    = _WS()
        swept = []

        auto_mod.capture_bgr      = lambda cam: _blank_frame()
        auto_mod.detect_obstacles = lambda f: [
            {"x1": 100, "y1": 50, "x2": 300, "y2": 200, "conf": 0.85, "class_id": 0}
        ]
        auto_mod.sweep_obstacle   = lambda c, s, l, r: (swept.append(True) or
                                                         {"left": 60.0, "center": 45.0, "right": 20.0})
        auto_mod.execute_avoidance = lambda ctrl, cam, dec: asyncio.sleep(0)  # fast no-op

        _run(auto_mod.navigate_step(ctrl, obs, _Camera(), ws, self._cache()))

        assert swept, "sweep_obstacle was not called on the YOLO path"

    def test_blocked_yolo_path_sends_avoiding(self):
        obs  = _Obstacle(blocked=True, dist=25.0)
        ctrl = _Controller()
        ws   = _WS()

        auto_mod.capture_bgr      = lambda cam: _blank_frame()
        auto_mod.detect_obstacles = lambda f: [
            {"x1": 100, "y1": 50, "x2": 300, "y2": 200, "conf": 0.85, "class_id": 0}
        ]
        auto_mod.sweep_obstacle   = lambda c, s, l, r: {"left": 60.0, "center": 45.0, "right": 20.0}
        auto_mod.execute_avoidance = lambda ctrl, cam, dec: asyncio.sleep(0)

        _run(auto_mod.navigate_step(ctrl, obs, _Camera(), ws, self._cache()))

        assert "avoiding" in ws.phases()

    def test_blocked_no_yolo_free_space_fallback(self):
        """No YOLO detections + confident free-space → avoiding via free_space path."""
        obs  = _Obstacle(blocked=True, dist=25.0)
        ctrl = _Controller()
        ws   = _WS()

        auto_mod.capture_bgr      = lambda cam: _blank_frame()
        auto_mod.detect_obstacles = lambda f: []
        auto_mod.detect           = lambda f: (0.5, 0.7)  # confident, steer right
        auto_mod.execute_avoidance = lambda ctrl, cam, dec: asyncio.sleep(0)

        _run(auto_mod.navigate_step(ctrl, obs, _Camera(), ws, self._cache()))

        assert "avoiding" in ws.phases()

    def test_blocked_no_yolo_all_blocked_reverses(self):
        """No YOLO + low free-space confidence → reverse straight."""
        obs  = _Obstacle(blocked=True, dist=25.0)
        ctrl = _Controller()
        ws   = _WS()

        auto_mod.capture_bgr      = lambda cam: _blank_frame()
        auto_mod.detect_obstacles = lambda f: []
        auto_mod.detect           = lambda f: (0.0, 0.1)  # below MIN_CONFIDENCE

        _run(auto_mod.navigate_step(ctrl, obs, _Camera(), ws, self._cache()))

        assert "blocked" in ws.phases()
        assert any(c[0] == "backward" for c in ctrl.calls)

    def test_ultrasonic_thread_not_blocked_branch_unchanged(self):
        """is_sudden_stop() branch remains at the top and is independent of YOLO."""
        obs  = _Obstacle(blocked=True, sudden=True, dist=4.0)
        ctrl = _Controller()
        ws   = _WS()
        yolo_called = []

        auto_mod.capture_bgr      = lambda cam: _blank_frame()
        auto_mod.detect_obstacles = lambda f: (yolo_called.append(True) or [])
        auto_mod.detect           = lambda f: (0.0, 0.0)

        _run(auto_mod.navigate_step(ctrl, obs, _Camera(), ws, self._cache()))

        # force_stop must have been called (sudden stop path)
        assert ("force_stop",) in ctrl.calls


# ---------------------------------------------------------------------------
# _SweepCache corridor checks — in_corridor() and side_in_corridor()
# ---------------------------------------------------------------------------

class TestSweepCacheCorridor:

    def _det_in(self, dist: float = 100.0) -> dict:
        """Detection centred on the frame, guaranteed inside the robot corridor at dist."""
        half_w = _ROBOT_WIDTH_CM / 2.0 * _FOCAL_LENGTH_PX / max(dist, 10.0)
        cx = _FRAME_W / 2.0
        return {"x1": int(cx - half_w + 1), "y1": 0,
                "x2": int(cx + half_w - 1), "y2": 100,
                "conf": 0.8, "class_id": 0}

    def _det_out(self, dist: float = 100.0) -> dict:
        """Detection entirely to the left of the robot corridor at dist."""
        half_w = _ROBOT_WIDTH_CM / 2.0 * _FOCAL_LENGTH_PX / max(dist, 10.0)
        cx = _FRAME_W / 2.0
        return {"x1": 0, "y1": 0,
                "x2": int(cx - half_w - 5), "y2": 100,
                "conf": 0.8, "class_id": 0}

    def test_in_corridor_true_when_detection_overlaps(self):
        cache = _SweepCache()
        cache.distances["center"]  = 100.0
        cache.detections["center"] = [self._det_in(100.0)]
        assert cache.in_corridor()

    def test_in_corridor_false_detection_outside_corridor(self):
        cache = _SweepCache()
        cache.distances["center"]  = 100.0
        cache.detections["center"] = [self._det_out(100.0)]
        assert not cache.in_corridor()

    def test_in_corridor_false_no_detections(self):
        cache = _SweepCache()
        cache.distances["center"]  = 100.0
        cache.detections["center"] = []
        assert not cache.in_corridor()

    def test_in_corridor_false_center_dist_none(self):
        cache = _SweepCache()
        cache.distances["center"]  = None
        cache.detections["center"] = [self._det_in(100.0)]
        assert not cache.in_corridor()

    def test_side_in_corridor_true_left_inside_threshold(self):
        cache = _SweepCache()
        cache.distances["left"] = _SWEEP_SIDE_CORRIDOR_CM - 1.0
        assert cache.side_in_corridor()

    def test_side_in_corridor_true_right_inside_threshold(self):
        cache = _SweepCache()
        cache.distances["right"] = _SWEEP_SIDE_CORRIDOR_CM - 1.0
        assert cache.side_in_corridor()

    def test_side_in_corridor_false_outside_threshold(self):
        cache = _SweepCache()
        cache.distances["left"]  = _SWEEP_SIDE_CORRIDOR_CM + 10.0
        cache.distances["right"] = _SWEEP_SIDE_CORRIDOR_CM + 10.0
        assert not cache.side_in_corridor()

    def test_side_in_corridor_false_all_none(self):
        assert not _SweepCache().side_in_corridor()

    def test_should_slow_true_via_in_corridor(self):
        """YOLO detection in robot corridor triggers slow even when ultrasonic sees nothing close."""
        dist = _WARN_CM + 20.0   # well outside both warn_cm and stop_cm*1.5 thresholds
        cache = _SweepCache()
        cache.distances["center"]  = dist
        cache.detections["center"] = [self._det_in(dist)]
        assert cache.should_slow()

    def test_should_slow_false_yolo_outside_corridor_far_dist(self):
        """Detection outside the physical corridor at a far distance must not trigger slow."""
        dist = _WARN_CM + 20.0
        cache = _SweepCache()
        cache.distances["center"]  = dist
        cache.detections["center"] = [self._det_out(dist)]
        assert not cache.should_slow()


# ---------------------------------------------------------------------------
# _SweepCache.should_avoid_side — side obstacle escalation
# ---------------------------------------------------------------------------

class TestSweepCacheSideAvoid:

    def test_false_initially(self):
        assert not _SweepCache().should_avoid_side()

    def test_true_left_with_yolo(self):
        cache = _SweepCache()
        cache.distances["left"]   = _WARN_CM - 10
        cache.detections["left"]  = [{"x1": 0, "y1": 0, "x2": 100, "y2": 100}]
        assert cache.should_avoid_side()

    def test_true_right_with_yolo(self):
        cache = _SweepCache()
        cache.distances["right"]  = _WARN_CM - 10
        cache.detections["right"] = [{"x1": 0, "y1": 0, "x2": 100, "y2": 100}]
        assert cache.should_avoid_side()

    def test_false_distance_only_no_yolo(self):
        """Distance alone without YOLO confirmation must not trigger side-avoid."""
        cache = _SweepCache()
        cache.distances["left"]  = _WARN_CM - 10
        cache.detections["left"] = []
        assert not cache.should_avoid_side()

    def test_false_yolo_only_beyond_warn(self):
        """YOLO detection beyond warn_cm must not trigger side-avoid."""
        cache = _SweepCache()
        cache.distances["left"]  = _WARN_CM + 10
        cache.detections["left"] = [{"x1": 0, "y1": 0, "x2": 100, "y2": 100}]
        assert not cache.should_avoid_side()

    def test_false_center_detection_only(self):
        """Center-direction readings must not trigger should_avoid_side."""
        cache = _SweepCache()
        cache.distances["center"]  = _WARN_CM - 10
        cache.detections["center"] = [{"x1": 0, "y1": 0, "x2": 100, "y2": 100}]
        assert not cache.should_avoid_side()

    def test_false_at_exact_warn_boundary(self):
        """Distance exactly equal to _WARN_CM must not trigger (condition is strictly <)."""
        cache = _SweepCache()
        cache.distances["left"]  = float(_WARN_CM)
        cache.detections["left"] = [{"x1": 0, "y1": 0, "x2": 100, "y2": 100}]
        assert not cache.should_avoid_side()

    def test_invalidate_clears_all_readings(self):
        cache = _SweepCache()
        cache.distances  = {"left": 10.0, "center": 20.0, "right": 30.0}
        cache.detections = {"left": [{"x1": 0}], "center": [], "right": []}
        cache.invalidate()
        assert all(v is None for v in cache.distances.values())
        assert all(v == []   for v in cache.detections.values())

    def test_invalidate_prevents_should_avoid_side_re_trigger(self):
        cache = _SweepCache()
        cache.distances["left"]  = _WARN_CM - 10
        cache.detections["left"] = [{"x1": 0, "y1": 0, "x2": 100, "y2": 100}]
        assert cache.should_avoid_side()
        cache.invalidate()
        assert not cache.should_avoid_side()


# ---------------------------------------------------------------------------
# navigate_step — side-avoid path
# ---------------------------------------------------------------------------

class TestNavigateStepSideAvoid:
    """navigate_step must route confirmed side threats through _handle_side_threat."""

    @pytest.fixture(autouse=True)
    def restore_module_attrs(self):
        originals = {
            "capture_bgr":       auto_mod.capture_bgr,
            "detect":            auto_mod.detect,
            "detect_obstacles":  auto_mod.detect_obstacles,
            "execute_avoidance": auto_mod.execute_avoidance,
        }
        yield
        for name, val in originals.items():
            setattr(auto_mod, name, val)

    def _armed_cache(self, side="left"):
        cache = _SweepCache()
        cache.distances[side]  = _WARN_CM - 10
        cache.detections[side] = [{"x1": 0, "y1": 0, "x2": 100, "y2": 100}]
        return cache

    def test_side_avoid_sends_avoiding_phase(self):
        """Side distance < warn_cm + YOLO → avoiding phase sent."""
        obs  = _Obstacle()
        ctrl = _Controller()
        ws   = _WS()
        auto_mod.execute_avoidance = lambda c, cam, dec: asyncio.sleep(0)

        _run(auto_mod.navigate_step(ctrl, obs, _Camera(), ws, self._armed_cache()))

        assert "avoiding" in ws.phases()

    def test_side_avoid_does_not_force_stop(self):
        """Side avoid is a steering correction — no hard stop."""
        obs  = _Obstacle()
        ctrl = _Controller()
        ws   = _WS()
        auto_mod.execute_avoidance = lambda c, cam, dec: asyncio.sleep(0)

        _run(auto_mod.navigate_step(ctrl, obs, _Camera(), ws, self._armed_cache()))

        assert ("force_stop",) not in ctrl.calls

    def test_side_avoid_turns_right_when_left_threatened(self):
        """Obstacle on the left → TURN_RIGHT (steer away from threat)."""
        obs       = _Obstacle()
        decisions = []
        auto_mod.execute_avoidance = lambda c, cam, dec: (decisions.append(dec), asyncio.sleep(0))[1]

        _run(auto_mod.navigate_step(_Controller(), obs, _Camera(), _WS(), self._armed_cache("left")))

        assert decisions == ["TURN_RIGHT"]

    def test_side_avoid_turns_left_when_right_threatened(self):
        """Obstacle on the right → TURN_LEFT (steer away from threat)."""
        obs       = _Obstacle()
        decisions = []
        auto_mod.execute_avoidance = lambda c, cam, dec: (decisions.append(dec), asyncio.sleep(0))[1]

        _run(auto_mod.navigate_step(_Controller(), obs, _Camera(), _WS(), self._armed_cache("right")))

        assert decisions == ["TURN_LEFT"]

    def test_side_avoid_invalidates_cache(self):
        """Cache must be cleared after avoidance so stale readings cannot re-trigger."""
        obs   = _Obstacle()
        cache = self._armed_cache()
        auto_mod.execute_avoidance = lambda c, cam, dec: asyncio.sleep(0)

        _run(auto_mod.navigate_step(_Controller(), obs, _Camera(), _WS(), cache))

        assert all(v is None for v in cache.distances.values())

    def test_side_avoid_does_not_fire_when_clear(self):
        """No side obstacle → side-avoid path not entered."""
        obs  = _Obstacle()
        ws   = _WS()
        auto_mod.execute_avoidance = lambda c, cam, dec: asyncio.sleep(0)
        auto_mod.detect            = lambda f: (0.0, 0.6)
        auto_mod.capture_bgr       = lambda cam: np.zeros((480, 640, 3), dtype=np.uint8)
        auto_mod.detect_obstacles  = lambda f: []

        _run(auto_mod.navigate_step(_Controller(), obs, _Camera(), ws, _SweepCache()))

        assert "avoiding" not in ws.phases()
