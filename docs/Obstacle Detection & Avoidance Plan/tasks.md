# Obstacle Detection & Avoidance — Task List

---

## Task 1: Add obstacle_avoidance config block to hardware.yaml
**File(s)**: `config/hardware.yaml`, `src/core/config.py`
**Description**: Add the four required constants under a new `obstacle_avoidance:` key in `hardware.yaml`. Expose the block via `config.py` as `OBSTACLE_AVOIDANCE_CFG`.
```yaml
obstacle_avoidance:
  robot_width_cm: 20
  clearance_buffer_cm: 10
  focal_length_px: 554        # calibrate: (lores_width / 2) / tan(HFOV_rad / 2)
  camera_hfov_deg: 102
```
**Acceptance Criteria**:
- `hardware.yaml` contains all four keys under `obstacle_avoidance:`
- `src/core/config.py` exports `OBSTACLE_AVOIDANCE_CFG = _hw["obstacle_avoidance"]`
- `python3 -c "from src.core.config import OBSTACLE_AVOIDANCE_CFG; print(OBSTACLE_AVOIDANCE_CFG)"` prints the dict without error on the Pi
**Dependencies**: none

---

## Task 2: Download and place YOLOv8n ONNX model
**File(s)**: `src/ai/models/yolov8n_320.onnx`
**Description**: Export YOLOv8n at `imgsz=320` to ONNX format and copy the file to `src/ai/models/`. Run on a machine with enough RAM/GPU, then `scp` to the Pi.
```bash
pip install ultralytics
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt').export(format='onnx', imgsz=320)"
scp yolov8n.onnx pi@<pi-ip>:~/robo-pi/src/ai/models/yolov8n_320.onnx
```
**Acceptance Criteria**:
- File exists at `src/ai/models/yolov8n_320.onnx` on the Pi
- `python3 -c "import cv2; net = cv2.dnn.readNetFromONNX('src/ai/models/yolov8n_320.onnx'); print('ok')"` succeeds without error
**Dependencies**: none

---

## Task 3: Implement YOLOv8n inference in object_detection.py
**File(s)**: `src/perception/vision/object_detection.py`
**Description**: Replace the stub with a working implementation that loads the ONNX model once at module level and exposes a single `detect_obstacles(frame_bgr)` function.

The function must:
1. Resize the input BGR frame to 320×320, normalise to [0,1], run `cv2.dnn` forward pass
2. Parse the raw output tensor (shape `[1, 84, 6300]`): transpose → `[6300, 84]`, apply confidence threshold ≥ 0.4, NMS (IOU threshold 0.45)
3. Return a list of dicts: `{"x1","y1","x2","y2","conf","class_id"}` in pixel coords of the *original* frame (scale back from 320→original size)
4. Return empty list `[]` on empty frame or any exception — never raise

Coordinates must be clamped to `[0, frame_width]` / `[0, frame_height]`.

The model path is read from `OBSTACLE_AVOIDANCE_CFG` or falls back to `src/ai/models/yolov8n_320.onnx`.

**Acceptance Criteria**:
- `detect_obstacles(frame)` returns `[]` on a blank (black) 640×480 frame without error
- Given a test image containing a person, at least one dict with keys `x1,y1,x2,y2,conf,class_id` is returned and `class_id == 0`
- Inference time on Pi ≤ 2 s per frame at imgsz=320 (measure with `time.perf_counter`)
**Dependencies**: Task 2

---

## Task 4: Implement obstacle priority selection and threat classification
**File(s)**: `src/perception/vision/object_detection.py`
**Description**: Add two functions to `object_detection.py` (below `detect_obstacles`):

`select_primary_obstacle(detections, frame_width)` — picks the single highest-priority box from a detection list. Priority = largest bounding-box area; tie-break by smallest horizontal distance from frame centre. Returns the chosen dict, or `None` if list is empty.

`classify_width_threat(detection, frame_width)` — returns one of `"WIDE"`, `"MEDIUM"`, `"NARROW"`:
- `WIDE`   — bbox width ≥ 50 % of frame_width
- `MEDIUM` — bbox width ≥ 25 % of frame_width
- `NARROW` — bbox width < 25 % of frame_width

**Acceptance Criteria**:
- `select_primary_obstacle([])` returns `None`
- Given two boxes of known sizes, `select_primary_obstacle` returns the larger one
- `classify_width_threat` returns correct label for a manually constructed dict at each boundary
**Dependencies**: Task 3

---

## Task 5: Implement pixel_x_to_servo_angle()
**File(s)**: `src/perception/vision/object_detection.py`
**Description**: Add `pixel_x_to_servo_angle(pixel_x, frame_width=640)` to `object_detection.py`.

Formula:
```
offset_frac = (pixel_x - frame_width / 2) / (frame_width / 2)   # [-1, 1]
raw_angle   = SERVO1_CENTER - offset_frac * (CAMERA_HFOV / 2)
angle       = clamp(raw_angle, SERVO1_MIN, SERVO1_MAX)
```

Where constants come from `OBSTACLE_AVOIDANCE_CFG` (`camera_hfov_deg`) and `SERVO_CFG["servo1"]` (`center_angle=89.85`, `min_angle=0`, `max_angle=180`).

**Acceptance Criteria**:
- `pixel_x_to_servo_angle(320)` returns `89.85` (centre pixel → centre servo)
- `pixel_x_to_servo_angle(0)` returns a value ≥ `SERVO1_MIN` (clamped, not out of range)
- `pixel_x_to_servo_angle(640)` returns a value ≤ `SERVO1_MAX` (clamped)
**Dependencies**: Task 1

---

## Task 6: Implement sweep_obstacle() and calculate_real_width()
**File(s)**: `src/perception/vision/object_detection.py`
**Description**: Add two functions that require hardware access — pass `controller` and `ultrasonic` as arguments (do not import hardware directly).

`sweep_obstacle(controller, ultrasonic, bbox_left_px, bbox_right_px, frame_width=640) -> dict`:
- Compute left, center, right pixel X positions of the bounding box
- For each of the three positions:
  1. Move head servo (servo1) to `pixel_x_to_servo_angle(px)` via `controller.move_camera_to("x", angle)`
  2. Sleep 100 ms for servo settle
  3. Read `ultrasonic.distance_cm()`
- After all three readings, call `controller.center_camera()` to restore head to forward position
- Return `{"left": float, "center": float, "right": float}` in cm

`calculate_real_width(bbox_pixel_width, distance_cm, focal_length_px) -> float`:
- Returns `(bbox_pixel_width * distance_cm) / focal_length_px`

**Acceptance Criteria**:
- `calculate_real_width(100, 50, 554)` ≈ 9.03 cm (verify with manual calculation)
- `sweep_obstacle` calls `controller.move_camera_to` exactly 3 times then `controller.center_camera` once (verify with a mock controller)
- Sweep returns a dict with keys `left`, `center`, `right` all as floats
- After sweep, servo is back at center (verified by checking `controller.center_camera()` was called)
**Dependencies**: Task 5

---

## Task 7: Implement decide_avoidance()
**File(s)**: `src/core/modes/autonomous.py`
**Description**: Add a pure function `decide_avoidance(width_threat: str, sweep: dict) -> str` that returns one of `"TURN_LEFT"`, `"TURN_RIGHT"`, `"REVERSE_AND_TURN"`.

Decision logic:
- If `width_threat == "NARROW"`: turn toward whichever side of the sweep has the greater distance (`sweep["left"]` vs `sweep["right"]`)
- If `width_threat == "MEDIUM"`: same as NARROW but only if the winning side exceeds `robot_width_cm + clearance_buffer_cm`; otherwise `"REVERSE_AND_TURN"`
- If `width_threat == "WIDE"`: always `"REVERSE_AND_TURN"`

`robot_width_cm` and `clearance_buffer_cm` are read from `OBSTACLE_AVOIDANCE_CFG`.

**Acceptance Criteria**:
- `decide_avoidance("WIDE", any_sweep)` always returns `"REVERSE_AND_TURN"`
- `decide_avoidance("NARROW", {"left": 80, "center": 40, "right": 20})` returns `"TURN_LEFT"`
- `decide_avoidance("MEDIUM", {"left": 15, "center": 10, "right": 15})` returns `"REVERSE_AND_TURN"` (both sides below clearance threshold of 30 cm with default config)
**Dependencies**: Task 1

---

## Task 8: Implement execute_avoidance() maneuvers
**File(s)**: `src/core/modes/autonomous.py`
**Description**: Add `async def execute_avoidance(controller, camera, decision: str)` that maps the three decision strings to concrete motor+servo sequences using existing `controller` methods. Use the same speed constants already imported at the top of `autonomous.py` (`AUTONOMOUS_SPEED`, `REVERSE_SPEED`).

Maneuvers:
- `"TURN_LEFT"`: steer left (`_STEER_LEFT`) → forward 0.8 s → steer center
- `"TURN_RIGHT"`: steer right (`_STEER_RIGHT`) → forward 0.8 s → steer center
- `"REVERSE_AND_TURN"`: mirror the existing K-turn block already in `navigate_step` (steer → back → opposite → forward → center) — extract that block into this function and call it from both places to avoid duplication

All maneuvers end with `await controller.smooth_stop()` and `controller.steer_center()`.

**Acceptance Criteria**:
- `"TURN_LEFT"` calls `controller.steer(_STEER_LEFT)` then `controller.forward(AUTONOMOUS_SPEED)` (verify with mock)
- `"REVERSE_AND_TURN"` calls `controller.backward(REVERSE_SPEED)` at some point
- The K-turn block in `navigate_step` is replaced by a call to `execute_avoidance` — no duplicated maneuver code remains
**Dependencies**: Task 7

---

## Task 9: Wire YOLO detections into navigate_step()
**File(s)**: `src/core/modes/autonomous.py`
**Description**: Update `navigate_step` to run YOLO when the robot is in `is_blocked()` state, before deciding to K-turn. Replace the current `if conf >= MIN_CONFIDENCE` branch with the full YOLO-informed decision pipeline.

New flow inside `is_blocked()`:
1. Stop motors (already done in existing code)
2. Call `detect_obstacles(capture_bgr(camera))` → detections
3. If detections non-empty:
   a. `primary = select_primary_obstacle(detections, frame_width=640)`
   b. `threat   = classify_width_threat(primary, frame_width=640)`
   c. `sweep    = await asyncio.get_event_loop().run_in_executor(None, sweep_obstacle, controller, obstacle._sensor, primary["x1"], primary["x2"])` — runs blocking sweep in thread pool so the event loop is not blocked
   d. `width_cm = calculate_real_width(primary["x2"]-primary["x1"], sweep["center"], OBSTACLE_AVOIDANCE_CFG["focal_length_px"])`
   e. `decision = decide_avoidance(threat, sweep)`
   f. `await execute_avoidance(controller, camera, decision)`
   g. Send `drive_state` with `phase="avoiding"`
4. If detections empty: fall back to existing `free_space.detect()` K-turn logic (no change)

Constraint: YOLO must not run while servo is moving — the sweep in step (c) moves the servo; YOLO in step (2) runs before the sweep begins. This ordering satisfies the constraint.

**Acceptance Criteria**:
- When YOLO returns a detection, `sweep_obstacle` is called (verify the head servo moves to 3 positions)
- When YOLO returns empty, the existing `free_space` path executes unchanged
- Thread A (ultrasonic hard-stop) is not modified — `is_sudden_stop()` branch remains at the top of `navigate_step`
- No import of hardware modules directly in `autonomous.py` — hardware is accessed only through `controller` and the `obstacle` object already passed in
**Dependencies**: Tasks 3, 4, 6, 8

---

## Task 10: Add obstacle argument to navigate_step and pass ultrasonic sensor to sweep
**File(s)**: `src/core/modes/autonomous.py`
**Description**: `sweep_obstacle` needs a reference to the `UltrasonicSensor` instance, which currently lives inside `obstacle._sensor`. Verify that `obstacle._sensor` is accessible from `navigate_step` (it is — `obstacle` is already a parameter). Update the call in Task 9 to pass `obstacle._sensor` directly. No new constructor changes needed if `ObstacleDetector._sensor` is accessible; if it is private and inaccessible, add a `sensor` property to `ObstacleDetector`.

**Acceptance Criteria**:
- `obstacle._sensor.distance_cm()` is reachable from `navigate_step` without AttributeError
- Alternatively, `obstacle.sensor` property returns the `UltrasonicSensor` instance if a property was added
- No changes to `hardware/sensors/ultrasonic.py` required
**Dependencies**: Task 6

---

## Summary of dependency order

```
Task 1 (config)
Task 2 (model download)
    └─ Task 3 (YOLO inference)
           └─ Task 4 (priority + threat)
Task 1 ──► Task 5 (pixel→servo angle)
               └─ Task 6 (sweep + real_width)
Task 1 ──► Task 7 (decide_avoidance)
               └─ Task 8 (execute_avoidance)
Tasks 3,4,6,8,10 ──► Task 9 (wire into navigate_step)
Task 6 ──► Task 10 (sensor access check)
```
