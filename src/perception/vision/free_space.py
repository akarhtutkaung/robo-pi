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

FRAME_W, FRAME_H = 320, 240

# Rows to analyse. Skip the top (background / ceiling) and the very
# bottom (rover chassis). Adjust ROI_TOP / ROI_BOTTOM if the camera
# tilt puts the floor at a different vertical position.
ROI_TOP    = 100
ROI_BOTTOM = 200

# Canny thresholds. Lower values pick up soft edges (carpet texture);
# higher values ignore them. Raise if you get false obstacles on the floor.
CANNY_LO = 30
CANNY_HI = 80

# Gaussian blur kernel before Canny — suppresses fine texture.
BLUR_K = 9  # must be odd

# 1-D column smoothing kernel — prevents tiny dips in density from
# being chosen as the free lane. Wider = more stable, less precise.
SMOOTH_K = 21  # must be odd

# Confidence threshold below which the signal should be treated as
# unreliable (caller decides what to do — e.g. reduce speed or skip PID).
MIN_CONFIDENCE = 0.25

_CX = FRAME_W / 2.0

# -------------------------------------------------------------------------


def detect(frame: np.ndarray) -> tuple[float, float]:
    """Return (error, confidence) from a 320×240 BGR frame.

    error      — steering offset, [-1 (full left) … +1 (full right)]
    confidence — detection quality, [0 … 1]
    """
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
        cam   = make_camera()
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
        # Live mode: python3 -m src.perception.vision.free_space [--live]
        print("Live mode — press 's' to save a frame, 'q' to quit.")
        from src.perception.camera import make_camera, capture_bgr  # type: ignore
        cam   = make_camera()
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
