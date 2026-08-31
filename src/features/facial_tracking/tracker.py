"""
Facial tracking loop — pan/tilt the head servos to keep a detected face
centered in frame. Triggered by a {"type": "mode", "action": "facial_tracking"}
WebSocket message, same mechanism as autonomous mode.

Detection lives in detector.py, tracking/angle policy in targeting.py; this
module owns the async tick loop, websocket status reporting, and the
standalone CLI harness.

After lost_face_recenter_after_s with no detection, the head recenters and
the locked target is dropped (see track_step).

Debug (SSH, no app running):
    cd ~/robo-pi
    python3 -m src.features.facial_tracking.tracker
    Ctrl+C to stop.

face_state message format (sent to the client each tick, mirrors
autonomous.py's drive_state):
    {
        "type":        "face_state",
        "tracking":    bool,
        "pan_angle":   float,
        "tilt_angle":  float,
        "error_x":     float,           # px offset of face center from frame center, signed
        "error_y":     float,
        "bbox_points": [[x,y], ...] | null,  # 4 corners of the face box, clockwise from
                                              # top-left — null when tracking is false
    }
"""

import asyncio
import json
import logging
import time

from websockets.exceptions import ConnectionClosed

from src.components.camera.camera import capture_bgr
from src.components.core.config import CAMERA_CFG, FACIAL_TRACKING_CFG
from src.features.facial_tracking.detector import detect_faces, detector_name, load_detector
from src.features.facial_tracking.targeting import (
    bbox_points,
    compute_new_angles,
    face_center,
    select_target,
    _DEAD_ZONE_PX,
    _FRAME_H,
    _FRAME_W,
    _LOCK_MAX_JUMP_PX,
    _MAX_STEP_DEG,
    _PAN_GAIN,
    _SERVO1_CENTER,
    _SERVO2_CENTER,
    _TILT_GAIN,
)

log = logging.getLogger(__name__)

_LOOP_PERIOD     = FACIAL_TRACKING_CFG["loop_period_s"]
_LOST_RECENTER_S = FACIAL_TRACKING_CFG["lost_face_recenter_after_s"]
# Below this, skip the hardware write entirely — cheap hobby servos visibly
# buzz/jiggle when re-commanded every tick for a fraction of a degree, rather
# than gliding. Gates the write only; compute_new_angles' own state (below)
# still reflects the actual last-commanded angle either way.
_MIN_STEP_DEG    = FACIAL_TRACKING_CFG["min_step_deg"]

_MAX_CONSECUTIVE_ERRORS = 5


class _TrackerState:
    """Mutable per-session tracking state threaded through the tick loop."""
    __slots__ = ("target_center", "target_box", "last_seen", "pan", "tilt")

    def __init__(self, pan: float, tilt: float):
        self.target_center: tuple[float, float] | None = None
        self.target_box: dict | None = None  # last detected {x1,y1,x2,y2}, for bbox_points
        self.last_seen: float = time.monotonic()
        self.pan = pan
        self.tilt = tilt


def _capture_and_detect(camera) -> list[dict]:
    """Blocking work for one tick — always run via run_in_executor."""
    frame = capture_bgr(camera)
    return detect_faces(frame)


async def track_step(controller, camera, state: _TrackerState) -> bool:
    """Run one tracking tick. Returns True if a face is currently tracked."""
    loop = asyncio.get_running_loop()
    faces = await loop.run_in_executor(None, _capture_and_detect, camera)

    target = select_target(faces, state.target_center)

    if target is None:
        if state.target_center is not None and time.monotonic() - state.last_seen > _LOST_RECENTER_S:
            state.target_center = None
            state.target_box = None
            state.pan, state.tilt = _SERVO1_CENTER, _SERVO2_CENTER
            controller.center_camera()
        return False

    state.last_seen = time.monotonic()
    state.target_center = face_center(target)
    state.target_box = target
    new_pan, new_tilt = compute_new_angles(state.target_center, state.pan, state.tilt)
    if abs(new_pan - state.pan) >= _MIN_STEP_DEG:
        controller.move_camera_to("x", int(round(new_pan)))
        state.pan = new_pan
    if abs(new_tilt - state.tilt) >= _MIN_STEP_DEG:
        controller.move_camera_to("y", int(round(new_tilt)))
        state.tilt = new_tilt
    return True


async def _send(websocket, tracking: bool, state: _TrackerState, error_x: float, error_y: float):
    if websocket is None:
        return
    # Only report a box while actually tracking this tick — target_box can be a few
    # ticks stale (retained for lock continuity across brief misses, see track_step).
    points = bbox_points(state.target_box) if tracking and state.target_box else None
    try:
        await websocket.send(json.dumps({
            "type":        "face_state",
            "tracking":    tracking,
            "pan_angle":   round(state.pan, 1),
            "tilt_angle":  round(state.tilt, 1),
            "error_x":     round(error_x, 1),
            "error_y":     round(error_y, 1),
            "bbox_points": points,
        }))
    except (ConnectionClosed, OSError):
        pass  # client disconnected — not an error
    except Exception:
        log.exception("Unexpected error sending face_state")


async def setup(controller):
    controller.center_camera()
    load_detector()  # fail fast, before the loop starts, if no detector can be built
    log.info(
        "Facial tracking: detector=%s pan_gain=%.2f tilt_gain=%.2f dead_zone=%dpx "
        "max_step=%.1fdeg min_step=%.1fdeg lock_max_jump=%dpx",
        detector_name(), _PAN_GAIN, _TILT_GAIN, _DEAD_ZONE_PX, _MAX_STEP_DEG, _MIN_STEP_DEG,
        _LOCK_MAX_JUMP_PX,
    )


async def run_facial_tracking(controller, camera, websocket=None):
    await setup(controller)
    state = _TrackerState(_SERVO1_CENTER, _SERVO2_CENTER)
    loop = asyncio.get_running_loop()
    consecutive_errors = 0

    try:
        while True:
            deadline = loop.time() + _LOOP_PERIOD
            try:
                tracking = await track_step(controller, camera, state)
                consecutive_errors = 0
                if state.target_center is not None:
                    error_x = state.target_center[0] - _FRAME_W / 2.0
                    error_y = state.target_center[1] - _FRAME_H / 2.0
                else:
                    error_x = error_y = 0.0
                await _send(websocket, tracking, state, error_x, error_y)
            except asyncio.CancelledError:
                raise
            except Exception:
                log.exception("track_step failed.")
                consecutive_errors += 1
                if consecutive_errors >= _MAX_CONSECUTIVE_ERRORS:
                    log.critical(
                        "Too many consecutive tracking failures (%d) — recentering and halting.",
                        consecutive_errors,
                    )
                    controller.center_camera()
                    raise
            remaining = deadline - loop.time()
            if remaining > 0:
                await asyncio.sleep(remaining)
    except asyncio.CancelledError:
        controller.center_camera()


# ---------------------------------------------------------------------------
# standalone CLI harness — python3 -m src.features.facial_tracking.tracker
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    from src.components.camera.camera import make_camera
    from src.components.navigation.controller import RobotController

    print("Facial tracking — standalone test. Ctrl+C to stop.\n")

    controller = RobotController()
    fc = CAMERA_CFG["front"]
    camera = make_camera(
        fc["index"], fc["main_width"], fc["main_height"],
        fc["lores_width"], fc["lores_height"], fc["framerate"],
        fc.get("rotate_180", False),
    )

    async def _run():
        state = _TrackerState(_SERVO1_CENTER, _SERVO2_CENTER)
        await setup(controller)
        while True:
            tracking = await track_step(controller, camera, state)
            status = "tracking" if tracking else "searching..."
            print(f"pan={state.pan:6.1f}  tilt={state.tilt:6.1f}  {status}", end="\r")
            await asyncio.sleep(_LOOP_PERIOD)

    try:
        asyncio.run(_run())
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        controller.center_camera()
        controller.cleanup()
        camera.stop()
