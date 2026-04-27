# robo-pi

Software system for an **Adeept PiCar-B Mars Rover** running on a Raspberry Pi. Built to support AI-integrated autonomous operation with camera vision, SLAM, and speech recognition — currently controllable over a local WebSocket connection with live WebRTC camera streaming.

## Hardware

| Component | Details |
|-----------|---------|
| Platform | Adeept PiCar-B (Raspberry Pi) |
| Motor | Rear DC motor via PCA9685 PWM controller (`0x5f`) |
| Servos | Steering (ch0), head L/R (ch1), head U/D (ch2) |
| Camera | Pi Camera (picamera2 → WebRTC H.264 stream) |
| Sensors | Ultrasonic (GPIO 23/24), line tracking (GPIO 22/27/17), light tracking (ADS7830 ADC `0x48`) |
| LEDs | RGB LEDs (GPIO 9/25/11, 19/0/13, 1/5/6) + WS2812 strip |
| Buzzer | GPIO 18 |

## Project Structure

```
robo-pi/
├── main.py                        # Entry point (--mode remote|autonomous)
├── config/
│   ├── hardware.yaml              # GPIO pins, I2C addresses, PWM settings
│   └── modes.yaml                 # Mode-specific settings
├── src/
│   ├── hardware/                  # Low-level hardware drivers
│   │   ├── motors.py              # Rear DC motor (smooth accel/decel via set_speed)
│   │   ├── servos.py              # Steering + head servos
│   │   ├── leds.py                # RGB LEDs + WS2812 strip
│   │   ├── buzzer.py
│   │   └── sensors/               # Ultrasonic, line, light, battery
│   ├── perception/                # Sensor data → interpreted signals
│   │   ├── camera.py              # CameraVideoTrack (picamera2 → aiortc)
│   │   └── vision/                # stream.py (H.264 config), gesture, object detection
│   ├── navigation/
│   │   ├── controller.py          # High-level drive commands
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
│       ├── config.py
│       └── modes/
│           ├── remote.py          # Runs control WS + WebRTC signaling concurrently
│           └── autonomous.py      # Standalone perception → action (planned)
├── tests/
└── examples/                      # Adeept reference scripts (read-only)
```

## Operating Modes

| Mode | Flow |
|------|------|
| **remote** | WebSocket message → `dispatch.py` → handler → `controller.py` → hardware |
| **autonomous** | Camera → perception → `controller.py` → hardware (planned) |

Both modes share the same `hardware/` drivers and `navigation/controller.py`.

## Getting Started

### Prerequisites

Install dependencies on the Raspberry Pi:

```bash
pip install -r requirements.txt
```

### Running

```bash
# Remote mode (default)
python3 main.py

# Explicit mode selection
python3 main.py --mode remote
```

### Run as a systemd service (auto-start on boot)

Create `/etc/systemd/system/robo-pi.service`:

```ini
[Unit]
Description=Robo-Pi Robot System
After=network.target

[Service]
ExecStart=/usr/bin/python3 /home/pi/robo-pi/main.py
WorkingDirectory=/home/pi/robo-pi
Restart=on-failure
User=pi

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

The control server listens on `ws://<pi-ip>:8765`. All messages are JSON. The `"type"` field is required — an unknown or missing type returns an error response.

### Movement messages (`"type": "movement"`)

```json
{ "type": "movement", "action": "throttle", "direction": "forward", "speed": 50 }
{ "type": "movement", "action": "throttle", "direction": "backward", "speed": 30 }
{ "type": "movement", "action": "steer", "angle": 30 }
{ "type": "movement", "action": "stop" }
```

### Vision messages (`"type": "vision"`)

```json
{ "type": "vision", "action": "camera-x", "angle": 90 }
{ "type": "vision", "action": "camera-y", "angle": 45 }
```

### Behavior

- **Idle timeout** — if no message is received for 300 ms, the robot smooth-stops (connection stays alive).
- **Disconnect** — smooth-stop is triggered automatically on client disconnect.

### Responses

```json
{ "status": "ok", "message": "" }
{ "status": "error", "message": "Unknown type: ..." }
{ "status": "error", "message": "Invalid JSON" }
```

## Camera Streaming

The Pi streams video over WebRTC. The client connects to the signaling server at `ws://<pi-ip>:8766`, sends an SDP offer, and receives an H.264 answer. ICE gathering completes on the Pi before the answer is sent (vanilla ICE — no trickle).

## Configuration

All pin assignments and tuning values live in `config/hardware.yaml` — never hardcoded in source files.

Key motor parameters:

| Key | Default | Description |
|-----|---------|-------------|
| `max_speed` | 14 | PCA9685 duty cycle ceiling |
| `accelerate_rate` | 10 | Units/sec ramp-up — applied as `rate × dt` per `set_speed()` call |
| `decelerate_rate` | 30 | Units/sec ramp-down — applied at 50 Hz in `smooth_stop()` |

## Planned Features

- Autonomous mode (perception → decision → action loop)
- Hand gesture control
- SLAM (simultaneous localization and mapping)
- Speech recognition via `sherpa-ncnn`

## Dependencies

| Package | Purpose |
|---------|---------|
| `adafruit-circuitpython-pca9685` | PCA9685 PWM controller (motors + servos) |
| `adafruit-circuitpython-motor` | Motor abstractions |
| `rpi-lgpio` | GPIO on Raspberry Pi 5 |
| `websockets` | WebSocket server |
| `aiortc` | WebRTC peer connection + media |
| `aiohttp` | HTTP server used by aiortc |
| `picamera2` | Pi Camera capture |
| `pyyaml` | Config file parsing |

> All hardware code requires a Raspberry Pi. Scripts importing `gpiozero`, `adafruit_pca9685`, or `rpi_ws281x` will not run on other platforms.
