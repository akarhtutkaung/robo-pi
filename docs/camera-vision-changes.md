# Camera & Vision Changes

Four things were built and wired together. Here's what each one does.

---

## 1. Dual Camera Setup

**Problem:** The original system had a single camera. The autonomous drive loop had no way to see behind the robot, and the camera was owned entirely inside `webrtc_server.py`.

**What changed:** Camera ownership moved up to `remote.py`. Two cameras are now created at startup — a wide-angle front camera and a rear camera — wrapped in a `CameraSwitch` object that both the WebRTC server and the autonomous drive loop share.

```
remote.py
  │
  ├── make_camera(front config)   ← Pi Camera V3 Wide Angle, CSI port 0
  ├── make_camera(back config)    ← Rear camera, CSI port 1
  ├── CameraSwitch(front, back)   ← active camera selector
  │
  ├── start_webrtc_server(cameras, stream_w, stream_h)  ← streams active camera to browser
  └── start_server(controller, cameras)
        ├── run_manual(...)                             ← switches on direction
        └── run_autonomous(controller, obstacle, cameras)
```

**Camera specs:**

| Camera | CSI | Main stream | Lores stream | Notes |
|--------|-----|-------------|--------------|-------|
| Front — Pi Camera V3 Wide | 0 | 1920×1080 YUV420 | 640×480 YUV420 | WebRTC + OpenCV |
| Back | 1 | 640×480 YUV420 | 320×240 YUV420 | Mounted upside-down — `rotate_180: true` applies `Transform(hflip, vflip)` |

All camera parameters (`index`, `main_width/height`, `lores_width/height`, `framerate`, `rotate_180`) live in `config/hardware.yaml` under `cameras.front` / `cameras.back`. Nothing is hardcoded in source.

**`CameraSwitch`** — `src/perception/camera.py`:

```python
cameras.use_back()   # WebRTC stream + OpenCV both switch to back camera
cameras.use_front()  # Switch back to front
```

Both `CameraVideoTrack` (WebRTC) and `capture_bgr()` (OpenCV) call `cameras.capture_array()`, so a single switch affects everything simultaneously.

**WebRTC stream resolution** is locked to the front camera's main size (1920×1080) for the entire session. `CameraVideoTrack.recv()` calls `frame.reformat(width, height)` on every frame so the back camera's 640×480 frames are upscaled to match — the WebRTC session never sees a resolution change.

**Automatic switching** — the stream follows drive direction:

| Event | Camera |
|-------|--------|
| Forward throttle | Front |
| Backward throttle | Back |
| Stop / idle timeout / explicit stop action | Front |

This is handled in `run_manual()` (manual mode) and `navigate_step()` (autonomous mode). In manual mode there is a 300 ms idle timeout — `camera.use_front()` is called directly in the `TimeoutError` handler, not inside the idle task, to avoid a race condition where the idle task gets cancelled before it can switch.

---

## 2. Free-Space Detector (`src/perception/vision/free_space.py`)

**What it does:** Takes a BGR frame and answers two questions:
- **Where is the open space?** → `error` (a number from -1.0 to +1.0)
- **How sure are we?** → `confidence` (0.0 to 1.0)

**Reference resolution is now 640×480** (front camera lores). Frames from the back camera (320×240 lores) are automatically resized to 640×480 inside `detect()` so the same tuning constants apply to both cameras.

**How it works — column-wise edge density:**

```
Frame (resized to 640×480 if needed)
│
├── Crop to ROI (rows 200–400) — skip ceiling at top, chassis at bottom
├── Grayscale → Gaussian blur (suppresses carpet/texture noise)
├── Canny edge detection → binary edge image
├── Sum edge pixels per column → density[640]  (high = obstacle, low = open)
├── Smooth with 41-wide 1-D kernel → prevents jitter
└── Find column with lowest density → that's where the free lane is
```

**Visual example:**

```
Frame columns (left → right):
  density: [high, high, LOW, LOW, LOW, high, high]
                          ↑
                    free_col = 320 (centre)
  error = (320 - 320) / 320 = 0.0  → drive straight
```

```
Frame columns (left → right):
  density: [high, high, high, LOW, LOW, LOW, LOW]
                                    ↑
                    free_col = 480 (right of centre)
  error = (480 - 320) / 320 = +0.5  → steer right
```

**Why edge density?** Obstacles (walls, furniture, objects) have sharp edges. The floor has almost no edges in its interior. This works regardless of floor colour or lighting, which is why it was chosen over colour-based segmentation.

**`confidence`** measures how distinct the free lane is from the blocked columns:

```
confidence = 1 - (lowest density / highest density)
```

- Confidence near 1.0 → a clear corridor with obvious obstacles on the sides
- Confidence near 0.0 → either everything is blocked, or nothing is (robot in a wide open room with no visible walls)

**Tunable constants** (top of `free_space.py`):

| Constant | Default | What it controls |
|----------|---------|-----------------|
| `FRAME_W / FRAME_H` | 640 / 480 | Reference resolution — all frames are resized to this before processing |
| `ROI_TOP` | 200 | Top of the analysis band — raise if ceiling appears in the ROI |
| `ROI_BOTTOM` | 400 | Bottom of the analysis band — lower if chassis is visible |
| `BLUR_K` | 9 | Blur strength before edge detection — raise for textured floors |
| `CANNY_LO / HI` | 30 / 80 | Edge sensitivity — raise both if floor texture triggers false edges |
| `SMOOTH_K` | 41 | Column smoothing width — raise if free_col jitters left/right |
| `MIN_CONFIDENCE` | 0.25 | Below this, the output is treated as unreliable |

**Offline tools** — test and tune on your actual environment:

```bash
# Test a saved image — shows annotated window (requires display)
python3 -m src.perception.vision.free_space path/to/frame.jpg

# Live camera with display (local monitor only)
python3 -m src.perception.vision.free_space --live

# Live camera over SSH — prints to stdout, saves debug_live.jpg every 30 frames
python3 -m src.perception.vision.free_space --headless

# Live camera streamed to browser — open http://<pi-ip>:8080
python3 -m src.perception.vision.free_space --stream
```

The debug overlay shows:
- Yellow rectangle: the ROI being analysed
- Grey bars at bottom: edge density per column (taller = more edges = obstacle)
- Green vertical line: detected free lane centre
- Blue vertical line: frame centre
- Top-left text: `err` and `conf` values (green = confident, orange = low confidence)

---

## 3. Camera Servo Fix + Wiring into autonomous.py

**Before:** The camera head (servo1) was used as a scanning tool. When the robot hit an obstacle, it would:
1. Pan the camera 45° right → wait 1 second → read ultrasonic
2. Pan the camera 135° left → wait 1 second → read ultrasonic
3. Compare distances to decide which way to turn

This took 2+ seconds just to decide a direction, and the camera was facing sideways during the scan.

**After:** The camera stays fixed forward. When an obstacle is detected, a single `detect(capture_bgr(camera))` call reads one frame and immediately gives the turn direction from the image.

**The autonomous loop now has three states:**

### State 1 — Blocked (ultrasonic < 30 cm)

```
stop → capture frame → detect free space → K-turn toward open side
       (front cam)                         (back cam during reverse)
```

The `error` sign tells the robot which way to turn:
- `error > 0` (free space is right of centre) → turn right
- `error < 0` (free space is left of centre) → turn left
- `confidence < 0.25` (can't tell) → reverse straight, reassess

During the K-turn, the camera switches to back before reversing (`camera.use_back()`) and returns to front before going forward (`camera.use_front()`).

### State 2 — Approaching (ultrasonic 30–90 cm)

Slow down, drive straight. No change from before.

### State 3 — Clear path (ultrasonic > 90 cm)

```python
error, conf = detect(capture_bgr(camera))
if conf >= MIN_CONFIDENCE:
    steer_angle = round(center_angle - error * half_range)
    controller.steer(steer_angle)
else:
    controller.steer_center()
controller.forward(AUTONOMOUS_SPEED)
```

The robot continuously steers toward the open lane while driving forward. The `error` maps linearly to a servo angle:

```
error = -1.0  →  full left  (140°)
error =  0.0  →  straight   (94.68°)
error = +1.0  →  full right (50°)
```

This is proportional (P-only) control. When the PID controller is added later, this same `error` feeds directly into it — the formula just gets `Kp * error + Ki * integral + Kd * derivative` instead of `error * half_range`.

---

## 4. Tuning the Free-Space Detector

There is no machine learning involved — the detector uses classical computer vision (Canny edges), so "training" means adjusting the constants in `free_space.py` until they work correctly for your specific floor and lighting.

### Step 1 — Collect frames from your actual environment

**Recommended — stream to browser (SSH-friendly):**

```bash
python3 -m src.perception.vision.free_space --stream
```

Open **`http://<pi-ip>:8080`** on your Mac. You see the full debug overlay live — ROI box, density bars, free-lane marker, error and confidence. Press Ctrl+C on the Pi to stop.

**SSH text-only (no browser needed):**

```bash
python3 -m src.perception.vision.free_space --headless
```

Prints `err` / `conf` to stdout every frame and saves `debug_live.jpg` every 30 frames (~1 sec). Copy to your Mac with:

```bash
scp akar@<pi-ip>:~/robo-pi/debug_live.jpg .
```

**With a display connected:**

```bash
python3 -m src.perception.vision.free_space --live
```

Press `s` to save `frame_000.jpg`, `frame_001.jpg`, etc. Point the robot at:

- Clear corridor ahead
- Obstacle on the left
- Obstacle on the right
- Corner (both sides blocked)
- Low-light / shadow on floor

### Step 2 — Replay each frame and read the debug overlay

```bash
python3 -m src.perception.vision.free_space frame_000.jpg
```

### Step 3 — Fix problems by adjusting constants

| What you see | Problem | Fix |
|---|---|---|
| Grey bars everywhere — carpet/texture detected as obstacle | `BLUR_K` too low or `CANNY_LO` too low | Raise `BLUR_K` to 13 or 15 (must stay odd); raise `CANNY_LO` to 50–60 |
| Green line jumps left/right between frames | Density curve too spiky | Raise `SMOOTH_K` to 51 or 61 |
| Yellow ROI includes ceiling or robot chassis | ROI framing is wrong | Raise `ROI_TOP` to push the band down; lower `ROI_BOTTOM` to cut the bottom |
| `conf` always low (< 0.25) in a clear corridor | Floor has no edges so density is uniformly low | Lower `MIN_CONFIDENCE` to 0.1, or tilt camera slightly downward to include more wall base |
| Green line on the wrong side | Inverted reading | Check the camera is centred forward and not physically rotated |

### Step 4 — Verify all scenarios pass

For each saved frame, the expected output is:

| Scenario | Expected `error` | Expected `conf` |
|---|---|---|
| Clear corridor | near 0.0 | > 0.5 |
| Obstacle on left | positive (> 0.3) | > 0.4 |
| Obstacle on right | negative (< −0.3) | > 0.4 |
| Both sides blocked | either sign | < 0.25 (robot will reverse straight) |

Once all saved frames produce sensible readings the detector is ready to run on the robot.

### Constants to adjust, in order of impact

1. `ROI_TOP` / `ROI_BOTTOM` — get the ROI framing right first
2. `BLUR_K` — suppress floor texture
3. `CANNY_LO` / `CANNY_HI` — edge sensitivity
4. `SMOOTH_K` — stability of the free-lane column
5. `MIN_CONFIDENCE` — threshold for trusting the signal
