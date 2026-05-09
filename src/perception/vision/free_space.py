"""
Free-space detector for autonomous steering.

Algorithm: column-wise Canny edge density inside a horizontal ROI.
Free space = the column range with the fewest edges. Works without
knowing the floor color and handles varied indoor lighting.

Public API:
    detect(frame)  →  (error: float, confidence: float)

    error      — [-1, 1]. Negative = free space is left of centre, positive = right.
    confidence — [0, 1].  Low means the scene has no clear free lane
                          (uniform clutter or uniform open space).

Tune the constants at the top of this file.
Run the offline prototype to visualise detection on saved frames:

    python3 -m src.perception.vision.free_space
    python3 -m src.perception.vision.free_space path/to/frame.jpg
"""

import cv2
import numpy as np

# --- Tunable constants ---------------------------------------------------

# Reference resolution — front camera (Pi Camera V3 Wide Angle) lores stream.
# Frames from the back camera (320×240) are resized to this before detection
# so the ROI and kernel constants apply equally to both cameras.
FRAME_W, FRAME_H = 640, 480

# Rows to analyse. Skip the top (background / ceiling) and the very
# bottom (rover chassis). Adjust ROI_TOP / ROI_BOTTOM if the camera
# tilt puts the floor at a different vertical position.
ROI_TOP    = 200
ROI_BOTTOM = 400

# Canny thresholds. Lower values pick up soft edges (carpet texture);
# higher values ignore them. Raise if you get false obstacles on the floor.
CANNY_LO = 30
CANNY_HI = 80

# Gaussian blur kernel before Canny — suppresses fine texture.
BLUR_K = 9  # must be odd

# 1-D column smoothing kernel — prevents tiny dips in density from
# being chosen as the free lane. Wider = more stable, less precise.
SMOOTH_K = 41  # must be odd; scaled from 21 at 320 px → 41 at 640 px

# Confidence threshold below which the signal should be treated as
# unreliable (caller decides what to do — e.g. reduce speed or skip PID).
MIN_CONFIDENCE = 0.25

_CX = FRAME_W / 2.0

# -------------------------------------------------------------------------


def detect(frame: np.ndarray) -> tuple[float, float]:
    """Return (error, confidence) from a BGR frame.

    Frames that don't match FRAME_W×FRAME_H (e.g. back camera at 320×240)
    are resized before processing so tuning constants apply uniformly.

    error      — steering offset, [-1 (full left) … +1 (full right)]
    confidence — detection quality, [0 … 1]
    """
    if frame.shape[1] != FRAME_W or frame.shape[0] != FRAME_H:
        frame = cv2.resize(frame, (FRAME_W, FRAME_H))
    roi   = frame[ROI_TOP:ROI_BOTTOM, :]
    gray  = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur  = cv2.GaussianBlur(gray, (BLUR_K, BLUR_K), 0)
    edges = cv2.Canny(blur, CANNY_LO, CANNY_HI)

    density = edges.sum(axis=0).astype(float)          # (320,) edge count per column

    kernel  = np.ones(SMOOTH_K) / SMOOTH_K
    smooth  = np.convolve(density, kernel, mode="same")

    free_col   = int(np.argmin(smooth))
    error      = (free_col - _CX) / _CX

    d_min      = smooth[free_col]
    d_max      = smooth.max()
    confidence = float(np.clip(1.0 - d_min / (d_max + 1e-6), 0.0, 1.0))

    return float(error), confidence


def draw_debug(frame: np.ndarray, error: float, confidence: float) -> np.ndarray:
    """Annotate a copy of frame with ROI, density bars, and free-lane marker."""
    if frame.shape[1] != FRAME_W or frame.shape[0] != FRAME_H:
        frame = cv2.resize(frame, (FRAME_W, FRAME_H))
    vis = frame.copy()

    # ROI boundary
    cv2.rectangle(vis, (0, ROI_TOP), (FRAME_W - 1, ROI_BOTTOM - 1), (0, 255, 255), 1)

    # Recompute density for visualisation
    roi    = frame[ROI_TOP:ROI_BOTTOM, :]
    gray   = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur   = cv2.GaussianBlur(gray, (BLUR_K, BLUR_K), 0)
    edges  = cv2.Canny(blur, CANNY_LO, CANNY_HI)
    density = edges.sum(axis=0).astype(float)
    kernel  = np.ones(SMOOTH_K) / SMOOTH_K
    smooth  = np.convolve(density, kernel, mode="same")

    # Draw density bars along the bottom of the frame
    if smooth.max() > 0:
        bar_h    = 40
        bar_top  = FRAME_H - bar_h
        norm     = smooth / smooth.max()
        for x, v in enumerate(norm):
            bar_y = int(bar_top + bar_h * (1 - v))
            cv2.line(vis, (x, FRAME_H - 1), (x, bar_y), (180, 180, 180), 1)

    # Free-lane marker
    free_col = int(np.argmin(smooth))
    cv2.line(vis, (free_col, ROI_TOP), (free_col, ROI_BOTTOM - 1), (0, 255, 0), 2)

    # Frame centre
    cx = int(_CX)
    cv2.line(vis, (cx, ROI_TOP), (cx, ROI_BOTTOM - 1), (255, 100, 100), 1)

    # Text overlay
    colour = (0, 255, 0) if confidence >= MIN_CONFIDENCE else (0, 100, 255)
    cv2.putText(vis, f"err={error:+.2f}  conf={confidence:.2f}",
                (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1, cv2.LINE_AA)

    return vis


# ---------------------------------------------------------------------------
# Offline prototype
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import sys

    def _from_file(path: str) -> np.ndarray:
        img = cv2.imread(path)
        if img is None:
            sys.exit(f"Cannot load image: {path}")
        return cv2.resize(img, (FRAME_W, FRAME_H))

    def _from_camera() -> tuple[np.ndarray, "Picamera2"]:
        from src.perception.camera import make_camera, capture_bgr  # type: ignore
        from src.core.config import CAMERA_CFG  # type: ignore
        fc  = CAMERA_CFG["front"]
        cam = make_camera(fc["index"], fc["main_width"], fc["main_height"],
                          fc["lores_width"], fc["lores_height"], fc["framerate"],
                          fc.get("rotate_180", False))
        frame = capture_bgr(cam)
        return frame, cam

    if len(sys.argv) > 1 and sys.argv[1] != "--live":
        # Static image mode: python3 -m src.perception.vision.free_space frame.jpg
        frame = _from_file(sys.argv[1])
        error, confidence = detect(frame)
        print(f"error={error:+.3f}  confidence={confidence:.3f}"
              f"  ({'ok' if confidence >= MIN_CONFIDENCE else 'LOW CONFIDENCE'})")
        cv2.imshow("free_space", draw_debug(frame, error, confidence))
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    else:
        headless = "--headless" in sys.argv or "--live" not in sys.argv

        from src.perception.camera import make_camera, capture_bgr  # type: ignore
        from src.core.config import CAMERA_CFG  # type: ignore
        fc  = CAMERA_CFG["front"]
        cam = make_camera(fc["index"], fc["main_width"], fc["main_height"],
                          fc["lores_width"], fc["lores_height"], fc["framerate"],
                          fc.get("rotate_180", False))

        if headless:
            # Headless mode (SSH) — print to stdout, save debug frame every 30 frames.
            # python3 -m src.perception.vision.free_space --headless
            print("Headless mode — Ctrl+C to stop. Saves debug_live.jpg every 30 frames.")
            tick = 0
            try:
                while True:
                    frame       = capture_bgr(cam)
                    error, conf = detect(frame)
                    status = "ok" if conf >= MIN_CONFIDENCE else "LOW"
                    print(f"err={error:+.3f}  conf={conf:.2f}  [{status}]")
                    if tick % 30 == 0:
                        cv2.imwrite("debug_live.jpg", draw_debug(frame, error, conf))
                    tick += 1
            except KeyboardInterrupt:
                print("Stopped.")
            finally:
                cam.stop()

        else:
            # Live display mode (requires a local display).
            # python3 -m src.perception.vision.free_space --live
            print("Live mode — press 's' to save a frame, 'q' to quit.")
            saved = 0
            try:
                while True:
                    frame          = capture_bgr(cam)
                    error, conf    = detect(frame)
                    vis            = draw_debug(frame, error, conf)
                    cv2.imshow("free_space", vis)
                    key = cv2.waitKey(30) & 0xFF
                    if key == ord("q"):
                        break
                    if key == ord("s"):
                        path = f"frame_{saved:03d}.jpg"
                        cv2.imwrite(path, frame)
                        print(f"Saved {path}  error={error:+.3f}  conf={conf:.3f}")
                        saved += 1
            finally:
                cam.stop()
                cv2.destroyAllWindows()
