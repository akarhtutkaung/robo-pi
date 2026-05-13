# Obstacle Detection & Avoidance Plan
## Robotic Car — Raspberry Pi 5 + Pi Camera Module 3 Wide + Ultrasonic (Shared Servo)

---

## 1. Hardware Configuration Summary

| Component | Spec | Role |
|---|---|---|
| Raspberry Pi 5 | 4GB RAM, quad-core Cortex-A76 | Main compute |
| Pi Camera Module 3 Wide | 102° horizontal FOV, 12MP | Obstacle detection + free-space steering |
| Ultrasonic sensor (HC-SR04) | ~15° beam angle, 2–400cm range, mounted 14 cm above ground | Metric distance measurement |
| Servo motor (head, ch1) | Shared mount for camera + ultrasonic | Directional targeting + lateral sweep |

**Critical constraint:** Camera and ultrasonic share the same servo. When the servo rotates, both sensors move together. YOLO inference run while the head is off-centre sees a shifted viewpoint — free-space steering is therefore only applied on centre-facing frames.

**Ultrasonic blind-spot:** The sensor is mounted 14 cm (5.5 in) above ground pointing straight ahead (0° tilt). Any obstacle shorter than 14 cm is invisible to the beam — it passes over and returns a clear reading. The `yolo_blocking()` check compensates for this (see Section 6).

---

## 2. Core Strategy

Four-layer detection pipeline:

```
Layer 1 (Always On):   Ultrasonic phase classification — clear / approaching / blocked / sudden-stop
Layer 2 (Clear Phase): Continuous lateral sweep (±20°) — YOLO + ultrasonic at each angle each tick
                       Provides early warning, slows speed, and triggers side avoidance
                       before Layer 1 triggers
Layer 3 (Blocked):     YOLOv8n detects obstacle, servo sweep measures per-angle clearance
                       decide_avoidance() selects TURN_LEFT / TURN_RIGHT / REVERSE_AND_TURN
Layer 4 (Steering):    Free-space floor-colour detector drives steering on centre-facing frames
```

---

## 3. Processing Loop (asyncio tick model)

The navigation loop is a single asyncio coroutine running at 10 Hz (`_LOOP_PERIOD = 0.1 s`).
No OS threads are used. Blocking I/O (camera capture, YOLO, ultrasonic) runs in the thread pool via `run_in_executor`.

```
run_autonomous():
  sweep_cache = _SweepCache()
  LOOP every 0.1 s:
    navigate_step(controller, obstacle, camera, websocket, sweep_cache)
    sleep remaining budget

navigate_step():
  ├── obstacle.is_blocked() OR sweep_cache.any_side_blocked()
  │     ├── if side-blocked only (forward sensor clear):
  │     │     force_stop + move_camera_to(center) + 150ms settle
  │     │     → _handle_side_threat()   # steer away using cached sweep data
  │     └── else (forward blocked):
  │           → _handle_blocked()       # smooth stop → YOLO → sweep → avoidance
  │           → sweep_cache.invalidate()
  │
  ├── sweep_cache.should_avoid_side()
  │     → _handle_side_threat()         # YOLO-confirmed side threat within warn_cm
  │                                     # no force_stop needed — speed already reduced
  │
  ├── sweep_cache.yolo_blocking()
  │     force_stop + move_camera_to(center) + 150ms settle
  │     → _handle_blocked()             # low obstacle: beam passed over it, YOLO confirmed
  │     → sweep_cache.invalidate()
  │
  ├── obstacle.should_turn()
  │     → _handle_approaching()         # drive at approach_speed
  │
  └── else (clear)
        → _handle_clear()               # advance sweep, steer, drive
```

**Phase thresholds** (from `config/hardware.yaml`):

| Phase | Condition | Action |
|---|---|---|
| Clear | `distance > turn_cm` (90 cm) | Sweep + free-space steer, full speed |
| Approaching | `stop_cm < distance ≤ turn_cm` | Drive at `approach_speed` |
| Blocked | `distance ≤ stop_cm` (30 cm) | Physics-based smooth stop → avoidance pipeline |
| Sudden stop | `distance < sudden_stop_cm` (20 cm) | `force_stop()` immediately |

---

## 4. Clear-Phase Lateral Sweep

During normal forward driving, the head sweeps continuously across three positions each tick:

```
Tick 1: head → left  (+20°) → capture frame + read ultrasonic → YOLO
Tick 2: head → center (0°)  → capture frame + read ultrasonic → YOLO + free-space
Tick 3: head → right (-20°) → capture frame + read ultrasonic → YOLO
Tick 4: repeat
```

**`_SweepCache`** stores the most recent `{distance, detections}` per position across ticks. Methods:

| Method | Description |
|---|---|
| `any_side_blocked()` | Any **side** (left/right) cached distance ≤ `stop_cm`. Center excluded — forward axis belongs to `obstacle.is_blocked()` which applies physics-based smooth decel; including it here would cause a hard jerk stop. |
| `should_avoid_side()` | Any side distance < `warn_cm` AND YOLO detection at that angle. Fills the gap between slowing and hard-stop: triggers full `_handle_side_threat` avoidance. |
| `should_slow()` | Any of: (1) YOLO + ultrasonic both inside `warn_cm`; (2) ultrasonic alone < `stop_cm × 1.2` (catches dark/novel objects YOLO misses); (3) `in_corridor()` — YOLO center detection overlaps robot body; (4) `side_in_corridor()` — side distance geometrically inside body width. |
| `in_corridor()` | YOLO center detection overlaps projected robot body `[cx ± (robot_half_width × focal_length / dist)]`. Only valid after the center tick. |
| `side_in_corridor()` | Side distance < `robot_half_width / sin(sweep_angle_deg)`. At ±20°, this threshold is ~29 cm. Catches dark or novel objects the beam hits but YOLO misses. |
| `yolo_blocking()` | Large in-corridor YOLO center detection with clear ultrasonic (see Section 6). |
| `invalidate()` | Clears all cached distances and detections after avoidance. Prevents stale readings from re-triggering on the next tick. |

**Speed adaptation each tick:**

1. `any_side_blocked()` → hard stop path
2. `should_avoid_side()` → `_handle_side_threat` (steering correction, no hard stop)
3. `yolo_blocking()` → stop + `_handle_blocked` (low obstacle)
4. `should_slow()` → `approach_speed`
5. Otherwise → `autonomous_speed`

**Key implementation detail:** Free-space steering is only updated on centre-frame ticks. On left/right ticks the camera FOV is angled, so the passability signal is invalid for forward path planning. The last computed steer angle is held between non-centre ticks.

Configuration (`config/modes.yaml`):

```yaml
sweep_angle_deg: 20      # degrees either side of centre
warn_cm: 60              # YOLO+ultrasonic early-warning threshold
yolo_block_ratio: 0.25   # min bounding-box width fraction for YOLO-only avoidance
```

---

## 5. Side-Obstacle Avoidance

**Two triggers, one handler:**

| Trigger | Condition | Entry path |
|---|---|---|
| Hard side-stop | `any_side_blocked()`: side dist ≤ `stop_cm` | `force_stop` → `move_camera_to(center)` → 150ms settle → `_handle_side_threat` |
| Soft side-avoid | `should_avoid_side()`: side dist < `warn_cm` AND YOLO confirms | Directly → `_handle_side_threat` (robot already at `approach_speed`) |

**`_handle_side_threat`** uses the cached `{distances, detections}` to pick a direction:

```python
# left threatened, not right (or left closer) → steer right (away from threat)
# right threatened, not left                  → steer left
# both threatened equally                     → TURN_RIGHT (fallback)
decision = "TURN_RIGHT" | "TURN_LEFT"
execute_avoidance(controller, camera, decision)
sweep_cache.invalidate()
```

The handler reads cached data rather than routing through `_handle_blocked` because the forward ultrasonic may read clear — `_handle_blocked`'s deceleration logic and forward-facing YOLO sweep would target the wrong axis.

---

## 6. YOLO-Only Low-Obstacle Avoidance

**Hardware constraint:** The ultrasonic sensor is mounted 14 cm above ground pointing dead ahead. Any obstacle shorter than 14 cm produces a clear reading regardless of its actual distance.

**Configuration:**

```yaml
# config/hardware.yaml
ultrasonic:
  height_cm: 14    # sensor mount height — used to compute _YOLO_BLOCK_RATIO floor

# config/modes.yaml
autonomous:
  yolo_block_ratio: 0.25   # minimum qualifying bounding-box width fraction
```

**`_YOLO_BLOCK_RATIO` geometric floor:** The configured ratio is floored by a physics-derived minimum so it can never be set dangerously low:

```
floor = (height_cm × focal_length_px) / (stop_cm × frame_width)
      = (14 × 259) / (30 × 640) ≈ 0.189 (19%)
```

At `stop_cm` (30 cm), an obstacle as wide as the sensor height projects to ~19% of the frame. Setting the ratio below this would trigger avoidance for genuinely detectable distant obstacles (false positives). The configured 0.25 already exceeds the floor; the `max()` in code catches future accidental config edits.

**`_SweepCache.yolo_blocking()` fires when all conditions hold:**

1. Center ultrasonic ≥ `warn_cm` — beam is reading clear, so normal paths won't trigger
2. Center YOLO detection width / frame_width ≥ `yolo_block_ratio`
3. Detection overlaps the robot's projected corridor: `det["x1"] < cx + half_w_px AND det["x2"] > cx − half_w_px`, where `half_w_px = (robot_width/2 × focal_length) / stop_cm`

The corridor width is computed at `stop_cm` (not at the actual ultrasonic reading) — conservative, since the obstacle is assumed to be close even though the beam passed over it.

**navigate_step entry path:**

```python
elif sweep_cache.yolo_blocking():
    controller.force_stop()
    controller.move_camera_to("x", int(round(_SERVO1_CENTER)))  # forward frame for _handle_blocked
    await asyncio.sleep(_HEAD_SETTLE_S)
    await _handle_blocked(controller, obstacle, camera, websocket)
    sweep_cache.invalidate()
```

The head re-centre is required because `yolo_blocking()` checks cached center data regardless of the current head position — if the triggering tick was a left/right sweep tick, the live frame would be angled, and bounding box coordinates fed into `sweep_obstacle` via `pixel_x_to_servo_angle` would produce wrong sweep angles.

---

## 7. Servo Angle Mapping

Pi Camera Module 3 Wide has **~102° horizontal FOV**. The wide-angle lens requires an `atan2` mapping — linear interpolation diverges by ~10° at frame edges.

```python
# config/hardware.yaml → obstacle_avoidance.focal_length_px
# Geometric estimate: (lores_width/2) / tan(hfov/2) = 320 / tan(51°) ≈ 259
# Calibrate empirically: place a 30 cm object at 50 cm, measure pixel width W,
# then set focal_length_px = (W * 50) / 30

def pixel_x_to_servo_angle(pixel_x: int, frame_width: int = 640) -> int:
    offset_px = pixel_x - frame_width / 2.0
    angle_deg = math.degrees(math.atan2(offset_px, FOCAL_LENGTH_PX))
    raw_angle = SERVO1_CENTER - angle_deg        # servo1: left = larger angle
    return int(round(max(SERVO1_MIN, min(SERVO1_MAX, raw_angle))))
```

Servo1 config (`config/hardware.yaml`):

| Key | Value | Meaning |
|---|---|---|
| `center_angle` | 89.85 | Forward |
| `min_angle` | 180 | Full left |
| `max_angle` | 0 | Full right |

---

## 8. Quick Threat Screen (Bounding Box Width %)

Before spending time on servo movement, classify the obstacle's apparent width from the bounding box alone. Fast — no servo required.

```python
def classify_width_threat(detection, frame_width=640):
    ratio = (detection["x2"] - detection["x1"]) / frame_width
    center_x = (detection["x1"] + detection["x2"]) / 2.0
    offset_ratio = abs(center_x - frame_width / 2.0) / (frame_width / 2.0)
    if ratio >= 0.50 and offset_ratio < 0.30:
        return "WIDE"
    elif ratio >= 0.25:
        return "MEDIUM"
    else:
        return "NARROW"
```

| Class | Width ratio | Centred? | Decision pre-filter |
|---|---|---|---|
| WIDE | ≥ 50% | Yes (`offset_ratio < 0.30`) | Skip sweep. `REVERSE_AND_TURN` immediately. |
| MEDIUM | 25–50%, or ≥ 50% off-centre | — | Full 3-point sweep. Pass only if winning side ≥ `robot_width + clearance`. |
| NARROW | < 25% | — | Full 3-point sweep. Pass on whichever side has more clearance. |

**Why the centred check on WIDE:** A large obstacle at the frame edge (e.g. a wall the robot is already turning away from) would otherwise trigger `REVERSE_AND_TURN` even when a simple steering correction is the right move. Requiring `offset_ratio < 0.30` limits WIDE to genuinely centred blockers.

---

## 9. Multi-Point Ultrasonic Sweep (Blocked Phase)

When blocked, a servo sweep measures clearance at three angles spanning the obstacle bounding box. Always runs for MEDIUM and NARROW (WIDE skips directly to `REVERSE_AND_TURN`).

```python
def sweep_obstacle(controller, sensor, bbox_left_px, bbox_right_px):
    positions = {
        "left":   bbox_left_px,
        "center": (bbox_left_px + bbox_right_px) / 2.0,
        "right":  bbox_right_px,
    }
    readings = {}
    try:
        for label, px in positions.items():
            angle = pixel_x_to_servo_angle(px)
            controller.move_camera_to("x", angle)
            time.sleep(0.10)                    # servo settle
            readings[label] = sensor.distance_cm()
    finally:
        controller.center_camera()              # always restore head
    return readings   # e.g. {"left": 48, "center": 50, "right": 200}
```

Total time: ~300–450 ms. Runs in the thread pool to avoid blocking the event loop. `center_camera()` is in a `finally` block so the head is restored even if the sensor raises.

**Interpretation:**
- All readings similar → flat-fronted obstacle spanning full box
- Right reading jumps far (e.g. 200 cm) → right edge of obstacle is near right box edge; right side clear
- Left reading jumps far → left side clear

---

## 10. Physical Width Calculation

```
real_width_cm = (W_px × D_cm) / focal_length_px
```

Calibrate `focal_length_px` once by placing a 30 cm object at 50 cm and measuring its pixel width `W`:

```
focal_length_px = (W × 50) / 30
```

Current estimate: **259 px** (geometric: `320 / tan(51°)`). Set in `config/hardware.yaml` under `obstacle_avoidance.focal_length_px`.

---

## 11. Avoidance Decision Logic

```python
def decide_avoidance(width_threat: str, sweep: dict) -> str:
    """
    width_threat — "WIDE" | "MEDIUM" | "NARROW"
    sweep        — {"left": cm, "center": cm, "right": cm}
    Returns      — "TURN_LEFT" | "TURN_RIGHT" | "REVERSE_AND_TURN"
    """
    if width_threat == "WIDE":
        return "REVERSE_AND_TURN"

    left_cm  = sweep.get("left",  0.0)
    right_cm = sweep.get("right", 0.0)
    if left_cm == 0.0 and right_cm == 0.0:
        return "REVERSE_AND_TURN"           # sweep failed — conservative fallback

    best_side = "TURN_LEFT" if left_cm >= right_cm else "TURN_RIGHT"
    best_cm   = max(left_cm, right_cm)

    if width_threat == "NARROW":
        return best_side                    # pass on the better side unconditionally

    # MEDIUM — only pass if the winning side has enough physical clearance
    min_pass_gap = ROBOT_WIDTH_CM + CLEARANCE_BUFFER_CM   # default: 20 + 10 = 30 cm
    return best_side if best_cm >= min_pass_gap else "REVERSE_AND_TURN"
```

Distance zone filtering (slow-down, stop) is handled upstream by `ObstacleDetector` — `decide_avoidance` only runs once the robot is already stopped.

---

## 12. Avoidance Maneuvers

```python
async def execute_avoidance(controller, camera, decision):
    if decision == "TURN_LEFT":
        controller.steer(STEER_LEFT)
        controller.forward(AUTONOMOUS_SPEED)
        await asyncio.sleep(turn_drive_s)           # 0.8 s
        await controller.smooth_stop()
        controller.steer_center()

    elif decision == "TURN_RIGHT":
        controller.steer(STEER_RIGHT)
        controller.forward(AUTONOMOUS_SPEED)
        await asyncio.sleep(turn_drive_s)
        await controller.smooth_stop()
        controller.steer_center()

    else:  # REVERSE_AND_TURN — K-turn, direction chosen by free-space signal
        # steer → settle → reverse → stop → opposite steer → settle → forward → centre → stop
        ...
```

All timing constants live in `config/modes.yaml` (`turn_drive_s`, `kturn_*`) so they stay in sync with speed values.

---

## 13. Free-Space Steering (Clear Phase, Centre Frames Only)

`src/perception/vision/free_space.py` — `detect(frame) → (error, confidence)`

Floor-colour passability + edge-density penalty, entirely classical CV:

1. Resize to 640×480 if needed.
2. Crop ROI: rows 300–420, cols 80–560 (removes ceiling, chassis, wide-angle vignetting).
3. HSV floor mask: saturation ≤ 60, brightness ≥ 100. Sum column-wise → floor coverage.
4. Canny edges (after 9×9 Gaussian blur). Sum column-wise → obstacle density.
5. `passability = floor_norm − 0.5 × edge_norm` per column (41-wide moving average).
6. `free_col = argmax(passability)`.
7. `error = (free_col − ROI_centre) / ROI_half_width` → [-1, 1].
8. `confidence` = normalised spread between best and worst column.

Steering is applied only when `confidence ≥ MIN_CONFIDENCE (0.25)`. Below threshold: `steer_center()`.

```python
steer_angle = round(CENTER_ANGLE - error * STEER_HALF_RANGE)
controller.steer(int(steer_angle))
```

**Suppressed when YOLO confirms an in-corridor obstacle:** If `in_corridor()` is True on a centre tick, `steer_center()` is applied instead of the free-space steer angle — driving toward a confirmed blocker because the floor looks clear in front of it is wrong.

Tune `FLOOR_S_MAX` / `FLOOR_V_MIN` in `free_space.py` for your floor colour and lighting.

---

## 14. Software Architecture

```
robo-pi/
├── config/
│   ├── hardware.yaml              # GPIO pins, servo angles, ultrasonic thresholds,
│   │                              # focal_length_px, robot_width_cm, clearance_buffer_cm,
│   │                              # ultrasonic.height_cm
│   └── modes.yaml                 # Speed values, avoidance timing, sweep_angle_deg,
│                                  # warn_cm, yolo_block_ratio
└── src/
    ├── core/
    │   ├── config.py              # Loads both yaml files, exposes named dicts
    │   └── modes/
    │       └── autonomous.py      # navigate_step, _handle_clear/approaching/blocked,
    │                              # _handle_side_threat, _SweepCache (with yolo_blocking),
    │                              # decide_avoidance, execute_avoidance
    ├── hardware/
    │   ├── motors.py              # RearMotor — smooth accel/decel ramp loop (50 Hz)
    │   ├── servos.py              # Steering servo (ch0), head pan (ch1), head tilt (ch2)
    │   └── sensors/
    │       └── ultrasonic.py      # HC-SR04: ObstacleDetector.is_blocked/should_turn/is_sudden_stop
    ├── perception/
    │   ├── camera.py              # CameraSwitch, reverse_cam() context manager, capture_bgr()
    │   └── vision/
    │       ├── free_space.py      # detect(), draw_debug()
    │       └── object_detection.py# detect_obstacles(), select_primary_obstacle(),
    │                              # classify_width_threat(), sweep_obstacle(),
    │                              # pixel_x_to_servo_angle(), calculate_real_width()
    ├── comms/
    │   └── debug_stream_server.py # Port 8080: combined free-space + YOLO MJPEG stream
    └── navigation/
        └── controller.py          # forward(), backward(), steer(), smooth_stop(), move_camera_to()
```

**Implementation status:**

| Component | Status |
|---|---|
| Ultrasonic zone classification | ✅ Done |
| YOLO obstacle detection (YOLOv8n ONNX 320px) | ✅ Done |
| Servo sweep (3-point, pixel→angle via atan2) | ✅ Done |
| Physical width calculation | ✅ Done |
| `classify_width_threat` with centre-offset guard | ✅ Done |
| `decide_avoidance` + `execute_avoidance` | ✅ Done |
| Free-space floor-colour steering | ✅ Done |
| Clear-phase lateral sweep (`_SweepCache`) | ✅ Done |
| Side-obstacle avoidance (`should_avoid_side` + `_handle_side_threat`) | ✅ Done |
| YOLO-only low-obstacle avoidance (`yolo_blocking`) | ✅ Done |
| Camera switch (front ↔ back during reverse) | ✅ Done |
| Physics-based smooth stop | ✅ Done |
| SLAM | ❌ Not started |
| Speech recognition | ❌ Not started |

---

## 15. Performance Expectations on Pi 5

| Step | Estimated Time |
|---|---|
| Camera capture (640×480 lores) | ~5 ms |
| YOLOv8n ONNX inference (320px input) | ~50–80 ms |
| Single ultrasonic ping | ~10–30 ms |
| Servo move (20°) + settle | ~80–120 ms |
| Head re-centre settle (`_HEAD_SETTLE_S`) | 150 ms |
| 3-point obstacle sweep | ~300–450 ms |
| Clear phase tick (capture + YOLO + ultrasonic concurrent) | ~60–90 ms |
| Centre tick (YOLO + free-space concurrent) | ~80–100 ms |

**Normal driving (clear phase):** Each 100 ms tick runs one sweep position. A full left/center/right cycle completes in ~300 ms. YOLO and free-space run concurrently on centre frames — no serial wait.

**Avoidance path:** Smooth stop + YOLO + 3-point sweep + maneuver = ~1–2 s total. Safe operating speed is ~0.3–0.5 m/s with `stop_cm = 30 cm`.

---

## 16. Known Limitations

1. **Single ultrasonic beam** — discrete samples, not a continuous profile. Irregularly shaped obstacles (chair legs, narrow poles) can be missed between sample angles. Mitigated by the ±20° clear-phase sweep catching off-axis obstacles early.

2. **Servo movement shifts camera frame** — free-space steering is only valid on centre-facing frames. Left/right sweep ticks use YOLO only. The `yolo_blocking()` branch always re-centres the head before capturing the blocked frame to ensure bounding-box pixel coordinates are valid for `sweep_obstacle`.

3. **Ultrasonic fails on soft/angled surfaces** — foam, carpet edges, and angled walls absorb or deflect pulses. `sudden_stop_cm = 20 cm` is intentionally conservative.

4. **Free-space tuning is floor-specific** — `FLOOR_S_MAX` and `FLOOR_V_MIN` are calibrated for light-coloured, low-saturation floors. Recalibrate for carpet or coloured tile.

5. **`focal_length_px` needs empirical calibration** — the default 259 px is a geometric estimate for the Pi Camera Module 3 Wide. Width estimates will be inaccurate until calibrated: place a 30 cm object at 50 cm, measure pixel width `W`, set `focal_length_px = (W × 50) / 30`.

6. **YOLO has no depth awareness** — two objects at different distances with similar pixel widths are classified identically. Ultrasonic is the ground truth for distance; YOLO provides type, position, and bounding box geometry only.

7. **`yolo_blocking()` fires on the cached center tick only** — if the robot approaches a low obstacle entirely during left/right sweep ticks, the center cache may not yet be populated with the detection. In practice the sweep cycles every 300 ms so the center tick will arrive within one full cycle. A future improvement could check all three positions, not just center.

---

## 17. Optional Upgrades

| Upgrade | Cost | What It Solves |
|---|---|---|
| IMU (MPU6050) | ~$3 | Detects if robot is stuck, spinning, or tipping |
| Second ultrasonic (fixed forward-left/right at 30–45°) | ~$2 | Covers blind spots between the 20° sweep positions during blocking phase |
| Downward-facing ultrasonic or ToF | ~$5 | Directly detects low obstacles the current beam passes over, removing reliance on `yolo_blocking()` |
| VL53L5CX ToF array (8×8 grid) | ~$15 | Full-width distance profile, no sweep needed |
| Coral USB Accelerator | ~$60 | Real-time YOLO + MiDaS depth at full 30 FPS on Pi 5 |

The fixed-angle second ultrasonic remains the highest value-per-dollar upgrade. The current servo sweep covers ±20° during clear driving, but the blocking-phase sweep points at the obstacle — a fixed side-facing sensor would cover flanks independently.
