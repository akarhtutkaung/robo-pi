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
│   ├── hardware.yaml              # GPIO pins, I2C addresses, PWM settings
│   └── modes.yaml                 # Mode-specific settings (autonomous vs remote)
├── src/
│   ├── hardware/                  # Low-level hardware drivers (Pi-only)
│   │   ├── motors.py              # Rear DC motor via PCA9685
│   │   ├── servos.py              # Servo 1 (steering), 2 (head L/R), 3 (head U/D)
│   │   ├── leds.py                # RGB LEDs + WS2812 strip
│   │   ├── buzzer.py              # Tonal buzzer output
│   │   └── sensors/
│   │       ├── ultrasonic.py      # Distance (GPIO 23/24)
│   │       ├── line_tracking.py   # Line sensors (GPIO 22/27/17)
│   │       ├── light_tracking.py  # ADC light sensors (ADS7830)
│   │       └── battery.py         # Battery voltage monitoring
│   ├── perception/                # Sensor data → interpreted signals
│   │   ├── camera.py              # CameraVideoTrack — picamera2 YUV420 → aiortc VideoStreamTrack
│   │   ├── vision/
│   │   │   ├── stream.py          # configure_h264(pc) — forces H.264 codec on RTCPeerConnection
│   │   │   ├── gesture.py         # Hand gesture → movement command
│   │   │   └── object_detection.py
│   │   └── speech/
│   │       ├── recognizer.py      # sherpa-ncnn wrapper
│   │       └── commands.py        # Recognized text → robot command
│   ├── navigation/                # Movement logic and mapping
│   │   ├── controller.py          # High-level drive commands (forward, turn, stop)
│   │   ├── planner.py             # Path planning
│   │   └── slam/
│   │       ├── mapper.py          # Build and update map
│   │       └── localizer.py       # Estimate position within map
│   ├── ai/
│   │   ├── inference.py           # Run on-device AI models
│   │   └── models/                # Model weight files (gitignored if large)
│   ├── comms/                     # WebSocket communication layer
│   │   ├── websocket_server.py    # Control WebSocket server (port 8765)
│   │   ├── webrtc_server.py       # WebRTC signaling WS (port 8766) + camera stream
│   │   ├── protocols/             # Per-domain message schemas and parsing
│   │   │   ├── base.py            # build_response() — shared by all handlers
│   │   │   ├── movement.py        # throttle, steer, stop
│   │   │   ├── vision.py          # camera-x, camera-y (future: gesture, stream)
│   │   │   └── voice.py           # command/text — placeholder for sherpa-ncnn
│   │   └── handlers/
│   │       ├── dispatch.py        # Central router — routes by "type" field to domain handler
│   │       ├── movement.py        # Handle throttle/steer/stop (type: "movement")
│   │       ├── vision.py          # Handle camera pan/tilt (type: "vision")
│   │       └── query.py           # Handle state/sensor queries from remote
│   └── core/
│       ├── robot.py               # Top-level Robot class — wires all modules together
│       ├── config.py              # Loads and exposes config/hardware.yaml
│       └── modes/
│           ├── autonomous.py      # Pi processes locally: perception → decision → action
│           └── remote.py          # Runs control WS + WebRTC signaling WS concurrently
├── tests/
└── examples/                      # Adeept reference scripts — read-only hardware reference
```

### Two Operating Modes

The `src/core/modes/` distinction maps directly to the two ways a feature (e.g. gesture control) can work:

| Mode | Flow | When to use |
|------|------|-------------|
| **autonomous** | camera → `perception/vision/gesture.py` → `navigation/controller.py` → hardware | Pi runs standalone, no network needed |
| **remote** | WebSocket message → `comms/handlers/dispatch.py` → domain handler → `navigation/controller.py` → hardware | Controlled from another device over local WiFi |

Both modes share the same `hardware/` and `navigation/controller.py` — only the input source differs.

## Planned System Components

The system is being built incrementally:

- **Motor/movement control** — `src/hardware/motors.py`, `src/hardware/servos.py`, `src/navigation/controller.py`
  - `RearMotor.set_speed(speed)`: applies one ramp step per call — `step = accelerate_rate × dt` where `dt` is elapsed time since the last call. Called once per incoming message; step size shrinks when messages arrive fast.
  - `RearMotor.smooth_stop()`: async coroutine — decelerates at 50 Hz using `decelerate_rate × 0.02s` per tick until speed < 0.1, then hard-stops.
  - `RearMotor.stop()`: immediate hard cut (throttle = 0) — reserved for disconnect/emergency only.
  - `RearMotor` takes an already-initialised `pca` object — it does not import or construct I2C/PCA9685 itself.
- **WebSocket server** — `src/comms/websocket_server.py`
  - Idle timeout (`IDLE_TIMEOUT = 0.3s`): if no message arrives within 0.3 s, calls `smooth_stop()` when not already stopped; connection stays alive.
  - Single in-flight task: each message cancels the previous `current_task` and creates a new one via `asyncio.create_task(handle(...))`, keeping the recv loop non-blocking.
  - All message routing is delegated entirely to `dispatch.py` — `websocket_server.py` does no action-level parsing itself.
  - Message types: `"movement"` → `handlers/movement.py`, `"vision"` → `handlers/vision.py`
  - Adding a new handler type: create `protocols/<domain>.py` + `handlers/<domain>.py`, then add one entry to `HANDLERS` in `dispatch.py`
  - WebRTC signaling runs on a separate port (8766) and is independent of this server
- **Camera + WebRTC streaming** — `src/perception/camera.py`, `src/perception/vision/stream.py`, `src/comms/webrtc_server.py`
  - `CameraVideoTrack` captures YUV420 frames via picamera2 and feeds them to aiortc
  - `configure_h264(pc)` in `stream.py` forces H.264 codec on the `RTCPeerConnection` before SDP negotiation — must be called after `addTrack()` and before `setRemoteDescription()`
  - `webrtc_server.py` runs a WebSocket signaling server on port 8766; handles SDP offer/answer (vanilla ICE — Pi waits for full ICE gathering before sending answer)
  - Ports: 8765 = control WS, 8766 = WebRTC signaling WS
  - Both servers run concurrently via `asyncio.gather()` in `remote.py`
- **AI integration** — `src/ai/inference.py`
- **Gesture control** — `src/perception/vision/gesture.py`
- **SLAM** — `src/navigation/slam/`
- **Speech recognition** — `src/perception/speech/` (sherpa-ncnn)

## Hardware Reference (Adeept PiCar-B)

All hardware communicates through I2C, GPIO, or SPI on the Raspberry Pi.

| Bus | Device | Address | Usage |
|-----|--------|---------|-------|
| I2C | PCA9685 PWM | `0x5f` | Motors (4x DC), servos — motor config keys: `max_speed` (unitless throttle scale), `accelerate_rate` and `decelerate_rate` (units/sec applied per `set_speed` call or per 50 Hz tick in `smooth_stop`) |
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

Runs the external `sherpa-ncnn` binary at `/home/pi/sherpa-ncnn/build/bin/sherpa-ncnn-alsa`, ALSA input `plughw:3,0`. Outputs to `output.txt` which is tail-polled for recognized text.

## Running on Device

All system code must run on the Raspberry Pi. Scripts that use `gpiozero`, `adafruit_pca9685`, or `rpi_ws281x` will fail on non-Pi hardware.

```bash
python3 examples/01_LED.py   # reference only — test individual hardware
```
