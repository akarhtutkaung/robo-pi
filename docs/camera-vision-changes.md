# Camera & Vision Changes

Three things were built and wired together. Here's what each one does.

---

## 1. Camera Sharing

**Problem:** The camera was a single `Picamera2` device. Before, it was created and owned entirely inside `webrtc_server.py` — the server that streams video to your browser over WebRTC. The autonomous drive loop had no way to look at camera frames at all.

**What changed:** Camera ownership was moved up to `remote.py`, which is the top-level entry point. It creates the camera once with `make_camera()`, then passes it down to both the WebRTC server and the autonomous drive loop.

```
remote.py
  │
  ├── make_camera()         ← created here, once
  │
  ├── start_webrtc_server(camera)   ← streams to your browser (unchanged)
  └── start_server(controller, camera)
        └── run_autonomous(controller, obstacle, camera)  ← now has camera access
```

**Two streams configured at startup:**

| Stream | Resolution | Format | Used by |
|--------|-----------|--------|---------|
| `main` | 640×480 | YUV420 | WebRTC (unchanged) |
| `lores` | 320×240 | YUV420 | OpenCV / autonomous vision |

Both streams come from the same physical camera simultaneously. WebRTC keeps streaming to your browser while the robot is also using the camera to navigate — they don't interfere.

**New function — `capture_bgr(camera)`** in `src/perception/camera.py`:

```python
def capture_bgr(camera: Picamera2) -> np.ndarray:
    yuv = camera.capture_array("lores")        # grab 320×240 from lores stream
    return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)  # convert to BGR for OpenCV
```

This is the function that autonomous code calls to get a frame. It always reads the small `lores` stream, never touching the main WebRTC stream.

---

## 2. Free-Space Detector (`src/perception/vision/free_space.py`)

**What it does:** Takes a 320×240 BGR frame and answers two questions:
- **Where is the open space?** → `error` (a number from -1.0 to +1.0)
- **How sure are we?** → `confidence` (0.0 to 1.0)

**How it works — column-wise edge density:**

```
Frame (320×240)
│
├── Crop to ROI (rows 100–200) — skip ceiling at top, chassis at bottom
├── Grayscale → Gaussian blur (suppresses carpet/texture noise)
├── Canny edge detection → binary edge image
├── Sum edge pixels per column → density[320]  (high = obstacle, low = open)
├── Smooth with 1-D kernel → prevents jitter
└── Find column with lowest density → that's where the free lane is
```

**Visual example:**

```
Frame columns (left → right):
  density: [high, high, LOW, LOW, LOW, high, high]
                          ↑
                    free_col = 160 (centre)
  error = (160 - 160) / 160 = 0.0  → drive straight
```

```
Frame columns (left → right):
  density: [high, high, high, LOW, LOW, LOW, LOW]
                                    ↑
                    free_col = 240 (right of centre)
  error = (240 - 160) / 160 = +0.5  → steer right
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
| `ROI_TOP` | 100 | Top of the analysis band — raise if ceiling appears in the ROI |
| `ROI_BOTTOM` | 200 | Bottom of the analysis band — lower if chassis is visible |
| `BLUR_K` | 9 | Blur strength before edge detection — raise for textured floors |
| `CANNY_LO / HI` | 30 / 80 | Edge sensitivity — raise both if floor texture triggers false edges |
| `SMOOTH_K` | 21 | Column smoothing width — raise if free_col jitters left/right |
| `MIN_CONFIDENCE` | 0.25 | Below this, the output is treated as unreliable |

**Offline tools** — test and tune on your actual environment before running on the robot:

```bash
# Test a saved image — shows annotated window
python3 -m src.perception.vision.free_space path/to/frame.jpg

# Live camera — press 's' to save frames, 'q' to quit
python3 -m src.perception.vision.free_space --live
```

The debug window shows:
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

This had two problems: it took 2+ seconds just to decide a direction, and the camera was facing sideways instead of forward.

**After:** The camera stays fixed forward. When an obstacle is detected, a single `detect(capture_bgr(camera))` call reads one frame and immediately gives the turn direction from the image.

**The autonomous loop now has three states:**

### State 1 — Blocked (ultrasonic < 30 cm)

```
stop → capture frame → detect free space → K-turn toward open side
```

The `error` sign tells the robot which way to turn:
- `error > 0` (free space is right of centre) → turn right
- `error < 0` (free space is left of centre) → turn left
- `confidence < 0.25` (can't tell) → default right

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

On the Pi, run:

```bash
python3 -m src.perception.vision.free_space --live
```

Point the robot at different real scenarios and press `s` to save each one:

- Clear corridor ahead
- Obstacle on the left
- Obstacle on the right
- Corner (both sides blocked)
- Low-light / shadow on floor

You'll get `frame_000.jpg`, `frame_001.jpg`, etc. in the current directory.

### Step 2 — Replay each frame and read the debug window

```bash
python3 -m src.perception.vision.free_space frame_000.jpg
```

The window shows:

- **Yellow box** — the ROI being analysed (rows 100–200)
- **Grey bars** at bottom — edge density per column (tall bar = obstacle detected there)
- **Green line** — where the detector thinks the free lane is
- **Blue line** — frame centre (straight ahead)
- **Top-left text** — `err` and `conf` values (green = confident, orange = low confidence)

### Step 3 — Fix problems by adjusting constants

| What you see | Problem | Fix |
|---|---|---|
| Grey bars everywhere — carpet/texture detected as obstacle | `BLUR_K` too low or `CANNY_LO` too low | Raise `BLUR_K` to 13 or 15 (must stay odd); raise `CANNY_LO` to 50–60 |
| Green line jumps left/right between frames | Density curve too spiky | Raise `SMOOTH_K` to 31 or 41 |
| Yellow ROI includes ceiling or robot chassis | ROI framing is wrong | Raise `ROI_TOP` to push the band down; lower `ROI_BOTTOM` to cut the bottom |
| `conf` always low (< 0.25) in a clear corridor | Floor has no edges so density is uniformly low — no contrast | Lower `MIN_CONFIDENCE` to 0.1, or tilt camera slightly downward to include more wall base / floor boundary |
| Green line on the wrong side | Inverted reading | Check the camera is centred forward and not physically rotated |

### Step 4 — Verify all scenarios pass

For each saved frame, the expected output is:

| Scenario | Expected `error` | Expected `conf` |
|---|---|---|
| Clear corridor | near 0.0 | > 0.5 |
| Obstacle on left | positive (> 0.3) | > 0.4 |
| Obstacle on right | negative (< −0.3) | > 0.4 |
| Both sides blocked | either sign | < 0.25 (robot will default right) |

Once all saved frames produce sensible readings the detector is ready to run on the robot.

### Constants to adjust, in order of impact

1. `ROI_TOP` / `ROI_BOTTOM` — get the ROI framing right first
2. `BLUR_K` — suppress floor texture
3. `CANNY_LO` / `CANNY_HI` — edge sensitivity
4. `SMOOTH_K` — stability of the free-lane column
5. `MIN_CONFIDENCE` — threshold for trusting the signal
