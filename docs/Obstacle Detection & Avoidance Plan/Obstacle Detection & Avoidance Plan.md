# Obstacle Detection & Avoidance Plan
## Robotic Car — Raspberry Pi 5 + Pi Camera Module 3 Wide + Ultrasonic (Shared Servo)

---

## 1. Hardware Configuration Summary

| Component | Spec | Role |
|---|---|---|
| Raspberry Pi 5 | 4GB RAM, quad-core Cortex-A76 | Main compute |
| Pi Camera Module 3 Wide | 102° horizontal FOV, 12MP | Obstacle detection + width estimation |
| Ultrasonic sensor (HC-SR04) | ~15° beam angle, 2–400cm range | Metric distance measurement |
| Servo motor | Shared mount for camera + ultrasonic | Directional targeting |

**Critical constraint:** Camera and ultrasonic share the same servo. When the servo rotates, both sensors move together. YOLO inference during servo movement will produce detections from a shifted viewpoint — this must be handled explicitly in the software loop.

---

## 2. Core Strategy

Use a **3-layer detection pipeline**:

```
Layer 1 (Always On):   Ultrasonic hard-stop safety check — independent of everything else
Layer 2 (Detection):   YOLOv8n detects obstacles, calculates bounding box geometry
Layer 3 (Measurement): Servo targets obstacle, ultrasonic measures metric distance
                        + Real physical width computed from pixel width + distance
```

Important: These layers are not sequential — Layer 1 runs in a separate thread. Layers 2 and 3 share the main loop.

---

## 3. Processing Loop (Main Logic)

```
LOOP:
  ├── [Thread A — always running]
  │     Fire ultrasonic at current angle
  │     If distance < HARD_STOP_CM (e.g. 20cm) → STOP motors immediately
  │     No YOLO dependency, no servo dependency
  │
  └── [Thread B — main detection loop]
        1. Servo at forward position (servo center)
        2. Capture camera frame
        3. Run YOLOv8n inference → get bounding boxes
        4. If no detections → continue driving
        5. If detections exist:
             a. Select highest-priority obstacle (largest box / most centered)
             b. Quick threat screen using bounding box width % (see Section 5)
             c. Calculate bounding box center X → map to servo angle
             d. Pause motor if needed (see latency note)
             e. Rotate servo to obstacle center angle
             f. Wait for servo settle (100ms)
             g. Fire ultrasonic → record distance D_center
             h. [Optional] Sweep left/right edges of bounding box (see Section 6)
             i. Calculate real physical width (see Section 7)
             j. Execute avoidance decision (see Section 8)
             k. Return servo to forward position
```

---

## 4. Servo Angle Mapping

Pi Camera Module 3 Wide has **~102° horizontal FOV**.

Map camera frame pixel X to servo angle:

```python
FRAME_WIDTH = 640        # pixels
CAMERA_HFOV = 102.0      # degrees, Pi Camera Module 3 Wide
SERVO_CENTER = 90        # degrees (forward)
SERVO_MIN = 39           # degrees (full left: 90 - 51)
SERVO_MAX = 141          # degrees (full right: 90 + 51)

def pixel_x_to_servo_angle(pixel_x):
    # Normalize pixel_x to [-0.5, 0.5]
    normalized = (pixel_x / FRAME_WIDTH) - 0.5
    # Scale to FOV
    angle_offset = normalized * CAMERA_HFOV
    # Apply to servo center
    servo_angle = SERVO_CENTER + angle_offset
    return max(SERVO_MIN, min(SERVO_MAX, servo_angle))
```

**Example:**
- Obstacle center at pixel x=160 (left quarter) → servo angle ≈ 64°
- Obstacle center at pixel x=320 (frame center) → servo angle = 90°
- Obstacle center at pixel x=480 (right quarter) → servo angle ≈ 116°

---

## 5. Quick Threat Screen (Bounding Box Width %)

Before spending time on servo movement and ultrasonic pings, classify the obstacle's apparent width using the bounding box alone. This is fast — no servo movement required.

```python
def classify_width_threat(bbox_pixel_width, frame_width=640):
    width_ratio = bbox_pixel_width / frame_width

    if width_ratio > 0.50:
        return "WIDE"     # occupies >50% of frame — treat as wall/large barrier
    elif width_ratio > 0.20:
        return "MEDIUM"   # may be passable depending on robot width
    else:
        return "NARROW"   # likely passable on either side
```

**Decision pre-filter:**
- `WIDE` → Do not attempt to pass. Reverse or wide turn immediately. Skip ultrasonic sweep.
- `MEDIUM` → Proceed to full ultrasonic measurement + width calculation.
- `NARROW` → Single center ultrasonic ping sufficient. Check which side has clearance.

---

## 6. Multi-Point Ultrasonic Sweep (For MEDIUM Obstacles)

Fire the ultrasonic at 3 angles across the bounding box: left edge, center, right edge.

```python
def sweep_obstacle(bbox_left_px, bbox_right_px):
    angles = {
        "left":   pixel_x_to_servo_angle(bbox_left_px),
        "center": pixel_x_to_servo_angle((bbox_left_px + bbox_right_px) / 2),
        "right":  pixel_x_to_servo_angle(bbox_right_px)
    }

    readings = {}
    for label, angle in angles.items():
        set_servo(angle)
        time.sleep(0.10)           # settle time
        readings[label] = ping_ultrasonic()
        time.sleep(0.05)

    return readings  # e.g. {"left": 48, "center": 50, "right": 52}
```

**Interpretation:**
- Readings similar across all 3 → flat-fronted obstacle, width spans full bounding box
- Right reading jumps far (e.g. 200cm) → right edge of obstacle is within bounding box, right side is clear
- Left reading jumps far → left side is clear

Total time for 3-point sweep: ~450ms. Account for this in motor speed tuning.

---

## 7. Physical Width Calculation

Once you have center distance `D` from ultrasonic and bounding box pixel width `W_px` from YOLO, calculate the real physical width of the obstacle.

**Formula:**
```
real_width_cm = (W_px × D_cm) / focal_length_px
```

**Calibrate focal length once** by placing a known-width object (e.g. 30cm box) at a known distance (e.g. 50cm) and measuring its pixel width:

```python
# One-time calibration
FOCAL_LENGTH_PX = (measured_pixel_width × known_distance_cm) / known_real_width_cm

# Example: object is 30cm wide, placed 50cm away, measures 200px wide
# FOCAL_LENGTH_PX = (200 × 50) / 30 = 333.3

def calculate_real_width(bbox_pixel_width, distance_cm, focal_length_px=333.3):
    return (bbox_pixel_width * distance_cm) / focal_length_px
```

**Combined decision input:**

```python
real_width = calculate_real_width(W_px, D_center)
# Now you know: obstacle is Xcm wide and Ycm away
# Compare to your robot's physical width to decide if passage is possible
```

---

## 8. Avoidance Decision Logic

```python
ROBOT_WIDTH_CM = 20        # measure your actual robot chassis width
CLEARANCE_BUFFER_CM = 10   # minimum extra clearance on each side
HARD_STOP_CM = 20          # Layer 1 threshold (Thread A)
SLOW_DOWN_CM = 50          # start slowing
AVOIDANCE_CM = 35          # trigger avoidance decision

def decide_avoidance(width_threat, D_center, real_width, sweep_readings):

    # Layer 1 handled by Thread A — already handled

    # Too far away — no action
    if D_center > SLOW_DOWN_CM:
        return "CONTINUE"

    # Slow down zone
    if D_center > AVOIDANCE_CM:
        return "SLOW"

    # Within avoidance range
    if width_threat == "WIDE":
        return "REVERSE_AND_TURN"

    # Check if robot can physically fit
    min_gap_needed = ROBOT_WIDTH_CM + (2 * CLEARANCE_BUFFER_CM)
    # Determine which side has more clearance from sweep readings
    left_clear = sweep_readings["left"] > AVOIDANCE_CM
    right_clear = sweep_readings["right"] > AVOIDANCE_CM

    if left_clear and sweep_readings["left"] > sweep_readings["right"]:
        return "TURN_LEFT"
    elif right_clear:
        return "TURN_RIGHT"
    else:
        return "REVERSE_AND_TURN"
```

---

## 9. Software Architecture

```
robo-pi/
├── main.py                              # Entry point — selects operating mode
├── config/
│   ├── hardware.yaml                    # All constants: thresholds, servo angles, pins, FOV, etc.
│   └── modes.yaml                       # Mode-specific settings (autonomous speed, etc.)
└── src/
    ├── core/
    │   ├── config.py                    # Loads hardware.yaml + modes.yaml, exposes constants
    │   └── modes/
    │       ├── autonomous.py            # Thread A (ultrasonic hard-stop) +
    │       │                            # Thread B (detect → decide → act loop)
    │       └── remote.py                # Remote-controlled mode (WebSocket + WebRTC)
    ├── hardware/
    │   ├── motors.py                    # Rear DC motor via PCA9685
    │   ├── servos.py                    # Steering servo + head pan/tilt servos
    │   └── sensors/
    │       └── ultrasonic.py            # HC-SR04 trigger/echo + distance calculation
    ├── perception/
    │   ├── camera.py                    # Picamera2 capture, CameraSwitch (front/back cameras)
    │   └── vision/
    │       ├── free_space.py            # Floor-colour + edge-density free-path detector
    │       │                            # pixel_x_to_servo_angle() and width estimation go here
    │       └── object_detection.py      # YOLOv8n inference — stub, to be implemented
    └── navigation/
        └── controller.py                # High-level: forward(), turn(), steer(), smooth_stop()
                                         # TURN_LEFT / TURN_RIGHT / REVERSE_AND_TURN routines go here
```

**Mapping from the plan's proposed layout to the real structure:**

| Plan module | Actual location |
|---|---|
| `config.py` | `config/hardware.yaml` + `src/core/config.py` |
| `hardware/servo.py` | `src/hardware/servos.py` |
| `hardware/ultrasonic.py` | `src/hardware/sensors/ultrasonic.py` |
| `hardware/motors.py` | `src/hardware/motors.py` |
| `hardware/camera.py` | `src/perception/camera.py` |
| `detection/yolo_detector.py` | `src/perception/vision/object_detection.py` (stub) |
| `detection/angle_mapper.py` | Add to `src/perception/vision/free_space.py` |
| `detection/width_estimator.py` | Add to `src/perception/vision/free_space.py` |
| `avoidance/sweep.py` | Add to `src/core/modes/autonomous.py` |
| `avoidance/decision.py` | `src/core/modes/autonomous.py` (already partially implemented) |
| `avoidance/maneuver.py` | `src/navigation/controller.py` |
| `threads/safety_thread.py` | `src/core/modes/autonomous.py` (Thread A) |
| `threads/detection_thread.py` | `src/core/modes/autonomous.py` (Thread B) |

The avoidance logic (sweep, decision, maneuver) lives inside `autonomous.py` rather than split into separate files. Split those out only once YOLO + servo sweep is implemented and the file becomes too large to manage.

---

## 10. Recommended Libraries & Models

| Purpose | Library / Model | Notes |
|---|---|---|
| Camera capture | `picamera2` | Official Pi 5 library |
| YOLO inference | `ultralytics` YOLOv8n | Export to ONNX for faster Pi 5 inference |
| ONNX runtime | `onnxruntime` | Faster than PyTorch on Pi 5 |
| Servo / GPIO | `pigpio` | More precise PWM than RPi.GPIO |
| Ultrasonic | `RPi.GPIO` or `pigpio` | pigpio preferred for accurate timing |
| Optional depth | `MiDaS Small` via ONNX | Only if you need full-frame depth — 3–5 FPS |

**YOLOv8n export for Pi 5:**
```bash
pip install ultralytics
yolo export model=yolov8n.pt format=onnx imgsz=320
# imgsz=320 gives faster inference than 640 with acceptable detection quality
```

---

## 11. Performance Expectations on Pi 5

| Step | Estimated Time |
|---|---|
| Camera capture (320×240) | ~5ms |
| YOLOv8n ONNX inference (320px) | ~50–80ms (~15 FPS) |
| Servo move + settle | ~100–150ms |
| Single ultrasonic ping | ~10–30ms |
| 3-point sweep (servo × 3 + pings) | ~450–600ms |
| Full loop (single obstacle, center only) | ~200–300ms |
| Full loop (medium obstacle, 3-point sweep) | ~600–800ms |

**Implication:** At 3-point sweep speed (~600ms per decision cycle), safe operating speed for your robot is roughly **0.3–0.5 m/s maximum** with 35cm trigger distance. Faster than that and the robot cannot react in time. Tune your motor speed accordingly.

---

## 12. Known Limitations (Do Not Ignore)

1. **Single ultrasonic beam** — even with sweeping, you are sampling discrete points, not a continuous profile. Irregularly shaped obstacles (e.g. chair legs) can be missed between sample angles.

2. **Servo movement shifts camera frame** — YOLO detections during or after servo rotation are from a different viewpoint. Always return servo to forward before next YOLO inference.

3. **MiDaS relative depth is not metric** — if you add MiDaS, its output must be calibrated against ultrasonic readings. Do not use raw MiDaS values as distances.

4. **Ultrasonic fails on soft/angled surfaces** — foam, carpet edges, fabric, and angled walls absorb or deflect ultrasonic pulses. Hard-stop threshold should be conservative (20cm, not 10cm).

5. **Wide-angle camera introduces barrel distortion** — pixel-to-angle mapping is only linear at frame center. For edge detections, apply lens distortion correction before mapping to servo angle. Pi Camera Module 3 supports calibration via OpenCV's `calibrateCamera()`.

6. **YOLO has no depth awareness** — two objects at different distances with similar pixel widths will be classified similarly. The ultrasonic measurement is the ground truth for distance; YOLO only provides type and position.

---

## 13. Optional Upgrades (If Current Setup Is Insufficient)

| Upgrade | Cost | What It Solves |
|---|---|---|
| Second ultrasonic (side-facing, fixed) | ~$2 | Detects obstacles outside servo sweep zone |
| VL53L5CX ToF array (8×8 grid) | ~$15 | Full-width distance profile, no sweep needed |
| IMU (MPU6050) | ~$3 | Detects if robot is stuck or tipping |
| Coral USB Accelerator | ~$60 | Enables MiDaS + YOLO at real-time FPS on Pi 5 |

The second ultrasonic is the highest value-per-dollar upgrade. Mount it facing forward-left or forward-right at a fixed 30–45° angle to catch obstacles that the servo-mounted sensor misses while pointing at a detected object.