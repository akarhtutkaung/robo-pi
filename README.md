# robo-pi

Software system for an **Adeept PiCar-B Mars Rover** running on a Raspberry Pi. Built to support AI-integrated autonomous operation with camera vision, SLAM, and speech recognition — controllable over a local WebSocket connection with live WebRTC camera streaming and a full autonomous obstacle-avoidance mode.

## Hardware

| Component | Details |
|-----------|---------|
| Platform | Adeept PiCar-B (Raspberry Pi) |
| Motor | Rear DC motor via PCA9685 PWM controller (`0x5f`) |
| Servos | Steering (ch0), head L/R (ch1), head U/D (ch2) |
| Front camera | Pi Camera V3 Wide Angle — CSI port 0 (1920×1080 WebRTC, 640×480 OpenCV) |
| Back camera | Rear camera — CSI port 1 (640×480 WebRTC, 320×240 OpenCV) |
| Sensors | Ultrasonic HC-SR04 (GPIO 23/24), line tracking (GPIO 22/27/17), light tracking (ADS7830 ADC `0x48`) |
| LEDs | RGB LEDs (GPIO 9/25/11, 19/0/13, 1/5/6) + WS2812 strip |
| Buzzer | GPIO 18 |

## Project Structure

```
robo-pi/
├── main.py                        # Entry point (--mode remote|autonomous)
├── setup.sh                       # One-time Pi setup (apt deps + venv)
├── config/
│   ├── hardware.yaml              # GPIO pins, I2C addresses, PWM settings, sensor thresholds
│   └── modes.yaml                 # Mode-specific tuning (autonomous speeds)
├── src/
│   ├── hardware/                  # Low-level hardware drivers
│   │   ├── motors.py              # Rear DC motor — smooth accel/decel, dynamic braking
│   │   ├── servos.py              # Steering + head servos
│   │   ├── leds.py                # RGB LEDs + WS2812 strip
│   │   ├── buzzer.py
│   │   └── sensors/               # Ultrasonic, line, light, battery
│   ├── perception/                # Sensor data → interpreted signals
│   │   ├── camera.py              # make_camera(), CameraSwitch, CameraVideoTrack, capture_bgr()
│   │   └── vision/                # stream.py (H.264 config), object_detection.py
│   ├── navigation/
│   │   ├── controller.py          # High-level drive commands (forward, steer, smooth_stop)
│   │   ├── planner.py
│   │   └── slam/                  # Mapping + localization (planned)
│   ├── ai/                        # On-device model inference (planned)
│   ├── comms/
│   │   ├── websocket_server.py    # Control WS server — port 8765
│   │   ├── webrtc_server.py       # WebRTC signaling WS — port 8766
│   │   ├── protocols/             # Per-domain message schemas
│   │   │   ├── base.py            # Shared build_response()
│   │   │   ├── movement.py        # throttle, steer, stop
│   │   │   ├── vision.py          # camera-x, camera-y
│   │   │   └── voice.py           # command/text (planned)
│   │   └── handlers/
│   │       ├── dispatch.py        # Routes messages by "type" field
│   │       ├── movement.py        # type: "movement"
│   │       ├── vision.py          # type: "vision"
│   │       └── query.py
│   └── core/
│       ├── robot.py
│       ├── config.py              # Loads hardware.yaml + modes.yaml, exposes named constants
│       └── modes/
│           ├── remote.py          # Runs control WS + WebRTC signaling concurrently
│           └── autonomous.py      # Obstacle avoidance loop — perception → decision → action
├── tests/
└── examples/                      # Adeept reference scripts (read-only)
```

## Operating Modes

| Mode | Flow |
|------|------|
| **remote** | WebSocket message → `dispatch.py` → handler → `controller.py` → hardware |
| **autonomous** | Ultrasonic → `ObstacleDetector` → avoidance algorithm → `controller.py` → hardware |

Both modes share the same `hardware/` drivers and `navigation/controller.py`. You can switch between them at runtime via WebSocket without restarting.

## Autonomous Obstacle Avoidance

The autonomous loop runs a three-tier speed approach and a K-turn avoidance maneuver:

**Speed zones** (tunable in `config/modes.yaml`):

| Zone | Condition | Behaviour |
|------|-----------|-----------|
| Cruise | `distance > turn_cm` | Full speed forward |
| Approach | `turn_cm > distance > stop_cm` | Half speed — braking zone |
| Stop | `distance < stop_cm` | Physics-based smooth stop targeting 5 cm from obstacle |
| Sudden stop | `distance < sudden_stop_cm` | Immediate hard cut (`force_stop`) |

**Avoidance maneuver** (K-turn):
1. Stop → scan right, then left with camera
2. Choose direction with greater clearance
3. Steer toward clear side → reverse → opposite steer → forward → center steering

## Algorithms

### Obstacle Detection — Ultrasonic threshold zones
`src/hardware/sensors/ultrasonic.py` + `autonomous.py`

Distance is read from an HC-SR04 sensor and classified into four zones each loop tick. No state machine or prediction — pure threshold comparison on the raw reading.

| Zone | Trigger | Action |
|------|---------|--------|
| Cruise | `distance > turn_cm` (90 cm) | Full speed forward |
| Approach | `stop_cm < distance ≤ turn_cm` | Reduce to `approach_speed` |
| Blocked | `distance ≤ stop_cm` (30 cm) | Physics-based smooth stop (see below) |
| Sudden stop | `distance < sudden_stop_cm` (20 cm) | Immediate hard cut |

### Stopping — Physics-based deceleration rate
`src/core/modes/autonomous.py` — `navigate_step()`

When blocked, the required deceleration rate is derived from kinematics rather than a fixed value:

```
required_rate = v² × cm_per_speed_unit / (2 × d_target)
```

where `v` is current speed, `d_target = distance - 5 cm` safety margin. This targets a stop exactly 5 cm from the obstacle regardless of approach speed.

### Motor speed ramping — Fixed-rate ramp loop
`src/hardware/motors.py` — `RearMotor._ramp_loop()`

Speed changes are applied at 50 Hz. The step size per tick depends on direction:

- **Accelerating forward** → `accelerate_rate × 0.02 s`
- **Into reverse from stop** → `reverse_accelerate_rate × 0.02 s` (slow creep)
- **Decelerating toward zero** → `decelerate_rate × 0.02 s`

`smooth_stop()` runs the same loop as an `async` coroutine and blocks until the motor reaches zero.

### Free-space detection — Column-wise Canny edge density
`src/perception/vision/free_space.py` — `detect(frame)`

Camera steering uses a single-pass OpenCV pipeline. The reference resolution is 640×480 (front camera lores). Frames from the back camera (320×240) are upscaled to 640×480 before processing so the same tuning constants apply to both.

1. Resize to 640×480 if needed.
2. Crop to a horizontal ROI (rows 200–400) — cuts out ceiling and rover chassis.
3. Grayscale → Gaussian blur (9×9) → Canny edge detection (lo=30, hi=80).
4. Sum edges column-wise → 1-D density array (640 values).
5. Smooth with a 41-wide moving average to prevent noise spikes from winning.
6. **Free column** = `argmin(smoothed_density)` — column with fewest edges.
7. **Error** = `(free_col - 320) / 320` → [-1, 1]; negative = free space is left.
8. **Confidence** = `1 - d_min / d_max` → 0 if the scene is uniformly cluttered or uniformly open.

No learning, no model weights — entirely classical CV. Requires `MIN_CONFIDENCE ≥ 0.25` before the signal is acted on.

### Avoidance maneuver — K-turn
`src/core/modes/autonomous.py` — `navigate_step()` blocked branch

When the rover is blocked and the camera reports a confident free-space direction, it executes a fixed-timing K-turn:

1. Steer toward free side (full lock).
2. Reverse for 1.5 s.
3. Smooth stop → steer opposite lock.
4. Drive forward 1.0 s → center steering → smooth stop.

If confidence is too low (all directions obstructed), reverse straight for 2 s and reassess.

> **Planned replacement:** PID loop for steering (see `Todo.md`) — replaces the fixed-angle steer in the "clear" phase and the K-turn with proportional corrections.

## Getting Started

### Setup (first time on Pi)

```bash
bash setup.sh
```

This installs system dependencies (including `python3-lgpio` for accurate ultrasonic timing), creates the virtual environment, and starts the `pigpiod`-equivalent `lgpio` daemon.

### Running

```bash
source .venv/bin/activate

# Remote mode (WebSocket + WebRTC)
python3 main.py --mode remote

# Autonomous mode (standalone obstacle avoidance)
python3 main.py --mode autonomous
```

### Run as a systemd service (auto-start on boot)

Create `/etc/systemd/system/robo-pi.service`:

```ini
[Unit]
Description=Robo-Pi Robot System
After=network.target

[Service]
ExecStart=/home/akar/robo-pi/.venv/bin/python3 /home/akar/robo-pi/main.py
WorkingDirectory=/home/akar/robo-pi
Restart=on-failure
User=akar

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable robo-pi
sudo systemctl start robo-pi
```

## Ports

| Port | Protocol | Purpose |
|------|----------|---------|
| 8765 | WebSocket | Robot control messages |
| 8766 | WebSocket | WebRTC signaling (SDP offer/answer) |

## WebSocket Protocol

The control server listens on `ws://<pi-ip>:8765`. All messages are JSON. The `"type"` field is required — unknown or missing type returns an error response.

### Movement (`"type": "movement"`)

```json
{ "type": "movement", "action": "throttle", "direction": "forward", "speed": 50 }
{ "type": "movement", "action": "throttle", "direction": "backward", "speed": 30 }
{ "type": "movement", "action": "steer", "angle": 30 }
{ "type": "movement", "action": "stop" }
```

### Vision / camera head (`"type": "vision"`)

```json
{ "type": "vision", "action": "move", "axis": "x", "angle":  5 }
{ "type": "vision", "action": "move", "axis": "x", "angle": -5 }
{ "type": "vision", "action": "move", "axis": "y", "angle":  5 }
{ "type": "vision", "action": "move", "axis": "y", "angle": -5 }
{ "type": "vision", "action": "center" }
```

`axis` — `"x"` for left/right, `"y"` for up/down. `angle` is a relative step in degrees.

### Mode switching (`"type": "mode"`)

```json
{ "type": "mode", "action": "autonomous" }
{ "type": "mode", "action": "manual" }
```

Switches the robot between autonomous obstacle-avoidance and manual remote-control at runtime. The robot smooth-stops when leaving autonomous mode.

### Behaviour

- **Idle timeout** — if no message is received for 300 ms, the robot smooth-stops (connection stays alive).
- **Disconnect** — hard stop triggered automatically on client disconnect.

### Responses

```json
{ "status": "ok",    "message": "" }
{ "status": "error", "message": "Unknown type: ..." }
{ "status": "error", "message": "Invalid JSON" }
```

## Camera Streaming

The Pi runs two cameras managed by `CameraSwitch` (`src/perception/camera.py`):

| Camera | CSI port | Main stream | Lores (OpenCV) | Used for |
|--------|----------|-------------|----------------|----------|
| Front — Pi Camera V3 Wide | 0 | 1920×1080 | 640×480 | Forward driving, free-space detection, WebRTC when moving forward or stopped |
| Back | 1 | 640×480 | 320×240 | Reversing visibility, free-space detection during reverse, WebRTC during reverse |

The WebRTC stream and OpenCV vision always read from the same active camera. `CameraSwitch` switches the active camera automatically: `use_back()` is called before any reverse move, `use_front()` is called once the rover is stopped and ready to go forward again.

Connect to the signaling server at `ws://<pi-ip>:8766`, send an SDP offer, receive an H.264 answer. ICE gathering completes on the Pi before the answer is sent (vanilla ICE — no trickle).

## Configuration

All pin assignments and tuning values are in config files — never hardcoded in source.

### `config/hardware.yaml` — hardware config

Key motor parameters:

| Key | Default | Description |
|-----|---------|-------------|
| `max_speed` | 14 | Throttle ceiling (unitless) |
| `accelerate_rate` | 10 | Units/sec ramp-up for forward motion |
| `reverse_accelerate_rate` | 3 | Units/sec ramp-up into reverse — slow creep |
| `decelerate_rate` | 200 | Units/sec ramp-down in `smooth_stop()` |
| `cm_per_speed_unit` | 1.2 | **Calibrate this** — cm/s per throttle unit (see below) |

Camera configuration (`cameras.front` / `cameras.back`):

| Key | Description |
|-----|-------------|
| `index` | Picamera2 CSI port index (0 = front, 1 = back) |
| `main_width` / `main_height` | Resolution of the main stream fed to WebRTC |
| `lores_width` / `lores_height` | Resolution of the lores stream used by OpenCV |

Key ultrasonic thresholds:

| Key | Default | Description |
|-----|---------|-------------|
| `turn_cm` | 90 | Distance to start slowing down |
| `stop_cm` | 30 | Distance to trigger stop maneuver |
| `sudden_stop_cm` | 20 | Distance for immediate hard stop |

### `config/modes.yaml` — mode-specific tuning

| Key | Default | Description |
|-----|---------|-------------|
| `speed` | 6 | Autonomous cruising speed |
| `reverse_speed` | 4 | Speed during avoidance reverse |
| `approach_speed` | 3 | Speed in braking zone |

### Calibrating `cm_per_speed_unit`

This value determines stopping precision. To calibrate:

1. Place the robot on the floor you'll operate on.
2. Run `forward(6)` for exactly 1 second.
3. Measure distance travelled `d_cm`.
4. Set `cm_per_speed_unit: <d_cm / 6>` in `hardware.yaml`.

## Planned Features

- Hand gesture control
- SLAM (simultaneous localization and mapping)
- Speech recognition via `sherpa-ncnn`
- AI inference (`src/ai/inference.py`)

## Dependencies

| Package | Purpose |
|---------|---------|
| `adafruit-circuitpython-pca9685` | PCA9685 PWM controller (motors + servos) |
| `adafruit-circuitpython-motor` | Motor abstractions |
| `rpi-lgpio` | GPIO compatibility on Raspberry Pi |
| `lgpio` | Hardware-accurate timing for ultrasonic sensor |
| `websockets` | WebSocket server |
| `aiortc` | WebRTC peer connection + media |
| `aiohttp` | HTTP server used by aiortc |
| `pyyaml` | Config file parsing |

> All hardware code requires a Raspberry Pi. Scripts importing `gpiozero`, `adafruit_pca9685`, or `rpi_ws281x` will not run on other platforms.
