# Obstacle Detection & Avoidance Plan
## Robotic Car — Raspberry Pi 5 + Pi Camera Module 3 Wide + Ultrasonic (Shared Servo)

---

## 1. Hardware Configuration Summary

| Component | Spec | Role |
|---|---|---|
| Raspberry Pi 5 | 4GB RAM, quad-core Cortex-A76 | Main compute |
| Pi Camera Module 3 Wide | 102° horizontal FOV, 12MP | Obstacle detection + free-space steering |
| Ultrasonic sensor (HC-SR04) | ~15° beam angle, 2–400cm range | Metric distance measurement |
| Servo motor (head, ch1) | Shared mount for camera + ultrasonic | Directional targeting + lateral sweep |

**Critical constraint:** Camera and ultrasonic share the same servo. When the servo rotates, both sensors move together. YOLO inference run while the head is off-centre sees a shifted viewpoint — free-space steering is therefore only applied on centre-facing frames.

---

## 2. Core Strategy

Four-layer detection pipeline:

```
Layer 1 (Always On):   Ultrasonic phase classification — clear / approaching / blocked / sudden-stop
Layer 2 (Clear Phase): Continuous lateral sweep (±20°) — YOLO + ultrasonic at each angle each tick
                       Provides early warning and slows speed before Layer 1 triggers
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
  │     → if side-blocked: force_stop + re-centre head + 80ms settle
  │     → _handle_blocked()
  │
  ├── obstacle.should_turn()
  │     → _handle_approaching()   # drive at approach_speed
  │
  └── else (clear)
        → _handle_clear()         # advance sweep, steer, drive
```

**Phase thresholds** (from `config/hardware.yaml` under `obstacle_avoidance`):

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

**`_SweepCache`** stores the most recent `{distance, detections}` per position across ticks.

Speed adaptation logic each tick:
- `any_side_blocked()`: any cached distance ≤ `stop_cm` → `force_stop` + head re-centre + enter blocked pipeline
- `should_slow()`: any cached distance < `warn_cm` (60 cm) AND YOLO detection at that angle → `approach_speed`
- Otherwise: `autonomous_speed`

**Why YOLO required for slowing:** Prevents false positives from ultrasonic noise on open space (ground reflections, narrow objects). YOLO + distance together give high confidence.

**Key implementation detail:** Free-space steering is only updated on centre-frame ticks. On left/right ticks the camera FOV is angled, so the passability signal is invalid for forward path planning. The last computed steer angle is held between non-centre ticks.

Configuration (`config/modes.yaml`):

```yaml
sweep_angle_deg: 20   # degrees either side of centre
warn_cm: 60           # YOLO+ultrasonic early-warning threshold
```

---

## 5. Servo Angle Mapping

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

## 6. Quick Threat Screen (Bounding Box Width %)

Before spending time on servo movement, classify the obstacle's apparent width from the bounding box alone. Fast — no servo required.

```python
def classify_width_threat(bbox, frame_width=640):
    width_ratio = (bbox["x2"] - bbox["x1"]) / frame_width
    if width_ratio >= 0.50:  return "WIDE"
    elif width_ratio >= 0.25: return "MEDIUM"
    else:                     return "NARROW"
```

| Class | Width ratio | Decision pre-filter |
|---|---|---|
| WIDE | ≥ 50% | Skip sweep. `REVERSE_AND_TURN` immediately. |
| MEDIUM | 25–50% | Full 3-point sweep. Pass only if winning side ≥ `robot_width + clearance`. |
| NARROW | < 25% | Full 3-point sweep. Pass on whichever side has more clearance. |

---

## 7. Multi-Point Ultrasonic Sweep (Blocked Phase)

When blocked, a servo sweep measures clearance at three angles spanning the obstacle bounding box. Always runs for MEDIUM and NARROW (WIDE skips directly to `REVERSE_AND_TURN`).

```python
def sweep_obstacle(controller, sensor, bbox_left_px, bbox_right_px):
    angles = {
        "left":   pixel_x_to_servo_angle(bbox_left_px  - margin),
        "center": pixel_x_to_servo_angle((bbox_left_px + bbox_right_px) // 2),
        "right":  pixel_x_to_servo_angle(bbox_right_px + margin),
    }
    readings = {}
    for label, angle in angles.items():
        controller.move_camera_to("x", angle)
        time.sleep(0.10)                    # servo settle
        readings[label] = sensor.distance_cm()
    controller.center_camera()
    return readings   # e.g. {"left": 48, "center": 50, "right": 200}
```

Total time: ~300–450 ms. Runs in the thread pool to avoid blocking the event loop.

**Interpretation:**
- All readings similar → flat-fronted obstacle spanning full box
- Right reading jumps far (e.g. 200 cm) → right edge of obstacle is near right box edge; right side clear
- Left reading jumps far → left side clear

---

## 8. Physical Width Calculation

```
real_width_cm = (W_px × D_cm) / focal_length_px
```

Calibrate `focal_length_px` once by placing a 30 cm object at 50 cm and measuring its pixel width `W`:

```
focal_length_px = (W × 50) / 30
```

Current estimate: **259 px** (geometric: `320 / tan(51°)`). Set in `config/hardware.yaml` under `obstacle_avoidance.focal_length_px`.

---

## 9. Avoidance Decision Logic

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

## 10. Avoidance Maneuvers

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

## 11. Free-Space Steering (Clear Phase, Centre Frames Only)

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

Tune `FLOOR_S_MAX` / `FLOOR_V_MIN` in `free_space.py` for your floor colour and lighting.

---

## 12. Software Architecture

```
robo-pi/
├── config/
│   ├── hardware.yaml              # GPIO pins, servo angles, ultrasonic thresholds,
│   │                              # focal_length_px, robot_width_cm, clearance_buffer_cm
│   └── modes.yaml                 # Speed values, avoidance timing, sweep_angle_deg, warn_cm
└── src/
    ├── core/
    │   ├── config.py              # Loads both yaml files, exposes named dicts
    │   └── modes/
    │       └── autonomous.py      # navigate_step, _handle_clear/approaching/blocked,
    │                              # _SweepCache, decide_avoidance, execute_avoidance
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
| decide_avoidance + execute_avoidance | ✅ Done |
| Free-space floor-colour steering | ✅ Done |
| Clear-phase lateral sweep (`_SweepCache`) | ✅ Done |
| Camera switch (front ↔ back during reverse) | ✅ Done |
| Physics-based smooth stop | ✅ Done |
| SLAM | ❌ Not started |
| Speech recognition | ❌ Not started |

---

## 13. Performance Expectations on Pi 5

| Step | Estimated Time |
|---|---|
| Camera capture (640×480 lores) | ~5 ms |
| YOLOv8n ONNX inference (320px input) | ~50–80 ms |
| Single ultrasonic ping | ~10–30 ms |
| Servo move (20°) + settle | ~80–120 ms |
| 3-point obstacle sweep | ~300–450 ms |
| Clear phase tick (capture + YOLO + ultrasonic concurrent) | ~60–90 ms |
| Centre tick (YOLO + free-space concurrent) | ~80–100 ms |

**Normal driving (clear phase):** Each 100 ms tick runs one sweep position. A full left/center/right cycle completes in ~300 ms. YOLO and free-space run concurrently on centre frames — no serial wait.

**Avoidance path:** Smooth stop + YOLO + 3-point sweep + maneuver = ~1–2 s total. Safe operating speed is ~0.3–0.5 m/s with `stop_cm = 30 cm`.

---

## 14. Known Limitations

1. **Single ultrasonic beam** — discrete samples, not a continuous profile. Irregularly shaped obstacles (chair legs, narrow poles) can be missed between sample angles. Mitigated by the ±20° clear-phase sweep catching off-axis obstacles early.

2. **Servo movement shifts camera frame** — free-space steering is only valid on centre-facing frames. Left/right sweep ticks use YOLO only.

3. **Ultrasonic fails on soft/angled surfaces** — foam, carpet edges, and angled walls absorb or deflect pulses. `sudden_stop_cm = 20 cm` is intentionally conservative.

4. **Free-space tuning is floor-specific** — `FLOOR_S_MAX` and `FLOOR_V_MIN` are calibrated for light-coloured, low-saturation floors. Recalibrate for carpet or coloured tile.

5. **`focal_length_px` needs empirical calibration** — the default 259 px is a geometric estimate for the Pi Camera Module 3 Wide. Width estimates will be inaccurate until calibrated: place a 30 cm object at 50 cm, measure pixel width `W`, set `focal_length_px = (W × 50) / 30`.

6. **YOLO has no depth awareness** — two objects at different distances with similar pixel widths are classified identically. Ultrasonic is the ground truth for distance; YOLO provides type, position, and bounding box geometry only.

---

## 15. Optional Upgrades

| Upgrade | Cost | What It Solves |
|---|---|---|
| IMU (MPU6050) | ~$3 | Detects if robot is stuck, spinning, or tipping |
| Second ultrasonic (fixed forward-left/right at 30–45°) | ~$2 | Covers blind spots between the 20° sweep positions during blocking phase |
| VL53L5CX ToF array (8×8 grid) | ~$15 | Full-width distance profile, no sweep needed |
| Coral USB Accelerator | ~$60 | Real-time YOLO + MiDaS depth at full 30 FPS on Pi 5 |

The fixed-angle second ultrasonic remains the highest value-per-dollar upgrade. The current servo sweep covers ±20° during clear driving, but the blocking-phase sweep points at the obstacle — a fixed side-facing sensor would cover flanks independently.
