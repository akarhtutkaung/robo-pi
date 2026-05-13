# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**robo-pi** is the software system for an **Adeept PiCar-B Mars Rover** running on a Raspberry Pi. The goal is to build an AI-integrated autonomous rover with camera vision, SLAM, and speech recognition, controlled via a local WebSocket server.

The `examples/` directory contains Adeept reference scripts — they are not part of the system, just hardware reference material.

## Project Structure

```
robo-pi/
├── main.py                        # Entry point — selects operating mode
├── requirements.txt
├── config/
│   ├── hardware.yaml              # GPIO pins, I2C addresses, PWM settings, sensor thresholds,
│   │                              # focal_length_px, robot_width_cm, clearance_buffer_cm
│   └── modes.yaml                 # Mode-specific tuning (speeds, avoidance timing,
│                                  # sweep_angle_deg, warn_cm)
├── src/
│   ├── hardware/                  # Low-level hardware drivers (Pi-only)
│   │   ├── motors.py              # Rear DC motor via PCA9685
│   │   ├── servos.py              # Servo 0 (steering), 1 (head L/R), 2 (head U/D)
│   │   ├── leds.py                # RGB LEDs + WS2812 strip
│   │   ├── buzzer.py              # Tonal buzzer output
│   │   └── sensors/
│   │       ├── ultrasonic.py      # Distance (GPIO 23/24) — ObstacleDetector class
│   │       ├── line_tracking.py   # Line sensors (GPIO 22/27/17)
│   │       ├── light_tracking.py  # ADC light sensors (ADS7830)
│   │       └── battery.py         # Battery voltage monitoring
│   ├── perception/                # Sensor data → interpreted signals
│   │   ├── camera.py              # make_camera(), CameraSwitch (+ reverse_cam() context manager),
│   │   │                          # CameraVideoTrack, capture_bgr()
│   │   └── vision/
│   │       ├── stream.py          # configure_h264(pc) — forces H.264 codec on RTCPeerConnection
│   │       ├── free_space.py      # detect(), draw_debug() — floor-colour passability steering
│   │       ├── gesture.py         # Hand gesture → movement command (planned)
│   │       └── object_detection.py# detect_obstacles(), classify_width_threat(),
│   │                              # sweep_obstacle(), pixel_x_to_servo_angle(),
│   │                              # calculate_real_width(), draw_detections()
│   ├── navigation/                # Movement logic and mapping
│   │   ├── controller.py          # High-level drive commands (forward, steer, smooth_stop,
│   │   │                          # move_camera_to, center_camera)
│   │   └── slam/
│   │       ├── mapper.py          # Build and update map (planned)
│   │       └── localizer.py       # Estimate position within map (planned)
│   ├── ai/
│   │   ├── inference.py           # Run on-device AI models (planned)
│   │   └── models/                # Model weight files (gitignored if large)
│   ├── comms/                     # WebSocket communication layer
│   │   ├── websocket_server.py    # Control WebSocket server (port 8765)
│   │   ├── webrtc_server.py       # WebRTC signaling WS (port 8766) + camera stream
│   │   ├── debug_stream_server.py # Combined free-space + YOLO MJPEG stream (port 8080, dev only)
│   │   ├── protocols/             # Per-domain message schemas and parsing
│   │   │   ├── base.py            # build_response() — shared by all handlers
│   │   │   ├── movement.py        # throttle, steer, stop
│   │   │   ├── vision.py          # camera-x, camera-y
│   │   │   └── voice.py           # command/text — placeholder for sherpa-ncnn
│   │   └── handlers/
│   │       ├── dispatch.py        # Central router — routes by "type" field to domain handler
│   │       ├── movement.py        # Handle throttle/steer/stop (type: "movement")
│   │       ├── vision.py          # Handle camera pan/tilt (type: "vision")
│   │       └── query.py           # Handle state/sensor queries from remote
│   └── core/
│       ├── robot.py               # Top-level Robot class — wires all modules together
│       ├── config.py              # Loads hardware.yaml + modes.yaml, exposes named dicts
│       └── modes/
│           ├── autonomous.py      # Full obstacle-avoidance loop (see below)
│           └── remote.py          # Runs control WS + WebRTC signaling WS concurrently
├── tests/
│   ├── conftest.py                # Hardware stubs (gpiozero, picamera2, aiortc…) for non-Pi hosts
│   ├── test_autonomous_logic.py
│   ├── test_free_space.py
│   └── test_object_detection.py
└── examples/                      # Adeept reference scripts — read-only hardware reference
```

### Two Operating Modes

| Mode | Flow | When to use |
|------|------|-------------|
| **autonomous** | Ultrasonic + camera (YOLO + free-space) → `autonomous.py` → `controller.py` → hardware | Pi runs standalone |
| **remote** | WebSocket message → `comms/handlers/dispatch.py` → domain handler → `controller.py` → hardware | Controlled over local WiFi |

Both modes share `hardware/` drivers and `navigation/controller.py`.

---

## Autonomous Navigation Architecture

`src/core/modes/autonomous.py` runs a 10 Hz asyncio tick loop. All blocking I/O (camera capture, YOLO inference, ultrasonic reads) must go through `loop.run_in_executor(None, fn, *args)` — never called directly in a coroutine.

### Tick structure

```
run_autonomous()
  sweep_cache = _SweepCache()
  while True:
    deadline = loop.time() + _LOOP_PERIOD    # 0.1 s
    await navigate_step(controller, obstacle, camera, websocket, sweep_cache)
    await asyncio.sleep(deadline - loop.time())   # sleep only remaining budget
```

### Phase dispatch (`navigate_step`)

```
obstacle.is_blocked() OR sweep_cache.any_side_blocked()
  → if side-blocked only: force_stop + move_camera_to(centre) + 80ms settle
  → _handle_blocked()

obstacle.should_turn()
  → _handle_approaching()      # drive at approach_speed

else
  → _handle_clear()            # lateral sweep + free-space steering
```

### Clear-phase lateral sweep (`_SweepCache`)

Each clear-phase tick rotates the head servo through `left → center → right` (±`sweep_angle_deg`, default 20°). At each position:

- **All ticks**: `capture_bgr` + `obstacle.sensor.distance_cm()` run **concurrently** via `asyncio.gather`. YOLO runs on the captured frame.
- **Centre tick only**: free-space `detect()` also runs concurrently with YOLO. Steering is updated from the free-space result. On left/right ticks the camera is angled — free-space is invalid, so the last steer angle is held.

`_SweepCache` exposes:
- `any_side_blocked()` — any cached distance ≤ `stop_cm` → triggers blocked pipeline
- `should_slow()` — any cached distance < `warn_cm` (60 cm) **and** YOLO detection at that angle → reduces to `approach_speed`

### Blocked-phase avoidance pipeline

1. Physics-based smooth stop (`v² × cm_per_speed_unit / (2 × d_target)`)
2. YOLO → `select_primary_obstacle` → `classify_width_threat` (WIDE/MEDIUM/NARROW)
3. Servo sweep (`sweep_obstacle`) — 3 pings across bounding box, runs in executor
4. `decide_avoidance(width_threat, sweep)` → `TURN_LEFT` | `TURN_RIGHT` | `REVERSE_AND_TURN`
5. `execute_avoidance(controller, camera, decision)`

Free-space fallback (`_free_space_avoid`) runs if YOLO finds no detection or the sweep fails.

### Key constants (all sourced from config, never hardcoded)

| Constant | Source |
|---|---|
| `AUTONOMOUS_SPEED`, `REVERSE_SPEED`, `APPROACH_SPEED` | `modes.yaml` |
| `_TURN_DRIVE_S`, `_KTURN_*` | `modes.yaml` |
| `_SWEEP_ANGLE_DEG`, `_WARN_CM` | `modes.yaml` |
| `_STEER_LEFT/RIGHT`, `_CENTER_ANGLE`, `_STEER_HALF_RANGE` | `hardware.yaml` servo0 |
| `_SERVO1_CENTER/MIN/MAX`, `_SWEEP_POSITIONS` | `hardware.yaml` servo1 |
| `_FOCAL_LENGTH_PX`, `_ROBOT_WIDTH_CM`, `_CLEARANCE_CM`, `_STOP_CM` | `hardware.yaml` obstacle_avoidance |
| `_CM_PER_SPEED_UNIT` | `hardware.yaml` motors.rear |

---

## Implemented System Components

### Motor/movement control — `src/hardware/motors.py`, `src/hardware/servos.py`, `src/navigation/controller.py`
- `RearMotor.set_speed(speed)`: ramp step per call — `step = accelerate_rate × dt`.
- `RearMotor.smooth_stop()`: async coroutine — decelerates at 50 Hz using `decelerate_rate × 0.02s` until speed < 0.1.
- `RearMotor.stop()`: immediate hard cut — reserved for disconnect/emergency only.
- `RearMotor` takes an already-initialised `pca` object — does not import I2C itself.

### WebSocket server — `src/comms/websocket_server.py`
- Idle timeout (`IDLE_TIMEOUT = 0.3s`): smooth-stops if no message within 0.3 s; connection stays alive.
- Single in-flight task: each message cancels the previous `current_task`.
- All routing delegated to `dispatch.py` — `websocket_server.py` does no action-level parsing.
- Adding a handler: create `protocols/<domain>.py` + `handlers/<domain>.py`, add entry to `HANDLERS` in `dispatch.py`.

### Camera + WebRTC streaming — `src/perception/camera.py`, `src/comms/webrtc_server.py`
- Two cameras: front = Pi Camera V3 Wide (CSI 0, 1920×1080 main / 640×480 lores), back = rear (CSI 1, 640×480 main / 320×240 lores). Resolutions in `hardware.yaml` under `cameras.front/back`.
- `CameraSwitch` exposes the active camera via `capture_array()`. Call `use_back()` before reversing, `use_front()` when going forward. The `reverse_cam()` **async context manager** wraps both calls and guarantees `use_front()` is restored even if the body raises — always use this instead of calling `use_back/front` directly in async code.
- `capture_bgr(camera)` reads lores stream and converts YUV420 → BGR. Accepts `Picamera2` or `CameraSwitch`.
- `configure_h264(pc)` forces H.264 on `RTCPeerConnection` — call after `addTrack()`, before `setRemoteDescription()`.
- Ports: 8765 = control WS, 8766 = WebRTC signaling, 8080 = MJPEG debug stream.

### Obstacle detection — `src/perception/vision/object_detection.py`
- `detect_obstacles(frame)` → list of `{x1,y1,x2,y2,conf,class_id}` — runs YOLOv8n ONNX (320px).
- `pixel_x_to_servo_angle(pixel_x)` — uses `atan2(offset_px, focal_length_px)` for wide-angle accuracy. The linear formula diverges by ~10° at frame edges for the 102° Pi Camera V3 Wide.
- `sweep_obstacle(controller, sensor, bbox_left_px, bbox_right_px)` — synchronous, must be called via `run_in_executor`. Returns `{left, center, right}` cm dict. Always calls `controller.center_camera()` in `finally`.
- `focal_length_px` default is 259 (geometric estimate for Pi Camera V3 Wide). **Calibrate empirically**: place 30 cm object at 50 cm, measure pixel width `W`, set `focal_length_px = (W × 50) / 30` in `hardware.yaml`.

### Free-space steering — `src/perception/vision/free_space.py`
- `detect(frame)` → `(error, confidence)`. Floor-colour passability (HSV primary) minus edge-density penalty (Canny secondary). Reference resolution 640×480; frames from back camera (320×240) auto-resized.
- ROI: rows 300–420, cols 80–560. Tune `FLOOR_S_MAX` / `FLOOR_V_MIN` for your floor colour.
- Steer only when `confidence ≥ MIN_CONFIDENCE (0.25)`. Below threshold: `steer_center()`.
- Steering formula: `steer_angle = round(CENTER_ANGLE - error * STEER_HALF_RANGE)`.
- Only valid on centre-facing frames. Do not apply free-space steering when the head servo is angled.

---

## Planned (Not Yet Implemented)

- **Gesture control** — `src/perception/vision/gesture.py`
- **SLAM** — `src/navigation/slam/`
- **Speech recognition** — `src/perception/speech/` (sherpa-ncnn binary at `/home/pi/sherpa-ncnn/…`)
- **AI inference** — `src/ai/inference.py`

---

## Hardware Reference (Adeept PiCar-B)

All hardware communicates through I2C, GPIO, or SPI on the Raspberry Pi.

| Bus | Device | Address | Usage |
|-----|--------|---------|-------|
| I2C | PCA9685 PWM | `0x5f` | Motors (4x DC), servos — motor config keys: `max_speed`, `accelerate_rate`, `decelerate_rate` |
| I2C | ADS7830 ADC | `0x48` | Light tracking, battery voltage |
| GPIO | Various | — | LEDs, buzzer, ultrasonic, line tracking |
| PWM GPIO 12 | WS2812 | — | NeoPixel LED strip (not supported on RPi 5) |

### Key GPIO Pins

| GPIO | Function |
|------|----------|
| 9, 25, 11 | LEDs |
| 18 | Buzzer |
| 19, 0, 13 | Left RGB LED (R/G/B) |
| 1, 5, 6 | Right RGB LED (R/G/B) |
| 23, 24 | Ultrasonic trigger/echo |
| 22, 27, 17 | Line tracking (left/middle/right) |

### Battery Monitoring

ADS7830 ADC → voltage divider (R15=3000Ω, R17=1000Ω, Vref=8.4V) → battery percentage. Warning threshold: 6.75V (<20%).

### Speech Recognition

Runs external `sherpa-ncnn` binary at `/home/pi/sherpa-ncnn/build/bin/sherpa-ncnn-alsa`, ALSA input `plughw:3,0`. Outputs to `output.txt` which is tail-polled for recognized text.

## Running on Device

All system code must run on the Raspberry Pi. Scripts importing `gpiozero`, `adafruit_pca9685`, or `rpi_ws281x` will fail on non-Pi hardware. Tests stub these out via `tests/conftest.py`.

```bash
python3 examples/01_LED.py   # reference only — test individual hardware
```
