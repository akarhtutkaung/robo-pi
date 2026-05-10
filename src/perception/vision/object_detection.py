"""
Obstacle detection — two levels:

  ObstacleDetector   — ultrasonic-based proximity wrapper (always on, Thread A)
  detect_obstacles() — YOLOv8n camera-based bounding-box detection (Thread B)

YOLO functions:
  detect_obstacles(frame_bgr)                                           → list[dict]
  select_primary_obstacle(detections, frame_width)                      → dict | None
  classify_width_threat(detection, frame_width)                         → "WIDE"|"MEDIUM"|"NARROW"
  pixel_x_to_servo_angle(pixel_x, frame_width)                         → int
  sweep_obstacle(controller, ultrasonic, bbox_left_px, bbox_right_px)  → dict
  calculate_real_width(bbox_pixel_width, distance_cm, focal_length_px) → float

Debug stream (SSH → browser):
  On the Pi:
    cd ~/robo-pi
    python3 -m src.perception.vision.object_detection

  On your Mac, open:
    http://<pi-ip>:8080

  The stream shows live YOLO bounding boxes (label + confidence) drawn on the
  640×480 lores camera frame — the same resolution used by autonomous mode.
  Inference time is printed per frame to stdout and overlaid on the stream.
  Press Ctrl+C on the Pi to stop.
"""

import pathlib
import time
import cv2
import numpy as np

from src.core.config import ULTRASONIC_CFG, OBSTACLE_AVOIDANCE_CFG, SERVO_CFG
from src.hardware.sensors.ultrasonic import UltrasonicSensor

# ---------------------------------------------------------------------------
# ObstacleDetector — ultrasonic proximity (unchanged, used by Thread A)
# ---------------------------------------------------------------------------

STOP_CM        = ULTRASONIC_CFG["stop_cm"]
TURN_CM        = ULTRASONIC_CFG["turn_cm"]
SUDDEN_STOP_CM = ULTRASONIC_CFG["sudden_stop_cm"]


class ObstacleDetector:
    def __init__(self):
        self._sensor = UltrasonicSensor()

    def distance_cm(self) -> float:
        return self._sensor.distance_cm()

    def is_blocked(self) -> bool:
        return self.distance_cm() < STOP_CM

    def is_sudden_stop(self) -> bool:
        return self.distance_cm() < SUDDEN_STOP_CM

    def should_turn(self) -> bool:
        return self.distance_cm() < TURN_CM

    @property
    def sensor(self) -> UltrasonicSensor:
        """Public accessor for the underlying UltrasonicSensor instance.
        Used by sweep_obstacle() in Task 9 to ping during servo sweep.
        """
        return self._sensor

    def cleanup(self):
        self._sensor.cleanup()

    def get_distance(self) -> float:
        return self.distance_cm()


# ---------------------------------------------------------------------------
# YOLOv8n inference — camera-based detection (Thread B)
# ---------------------------------------------------------------------------

_MODELS_DIR      = pathlib.Path(__file__).parent.parent.parent / "ai" / "models"
_MODEL_PATH      = str(_MODELS_DIR / "yolov8n_320.onnx")
_INPUT_SIZE      = 320
_CONF_THRESHOLD  = 0.4
_NMS_THRESHOLD   = 0.45

_net = None


def _load_net():
    global _net
    if _net is None:
        try:
            _net = cv2.dnn.readNetFromONNX(_MODEL_PATH)
        except Exception as e:
            print(f"[object_detection] Failed to load YOLO model: {e}")
    return _net


def detect_obstacles(frame_bgr: np.ndarray) -> list:
    """Run YOLOv8n inference on a BGR frame.

    Returns a list of dicts — one per detected obstacle after NMS:
        {"x1": int, "y1": int, "x2": int, "y2": int,
         "conf": float, "class_id": int}
    Coordinates are in the original frame's pixel space.
    Returns [] on empty frame, missing model, or any inference error.
    """
    if frame_bgr is None or frame_bgr.size == 0:
        return []
    try:
        net = _load_net()
        if net is None:
            return []

        h, w = frame_bgr.shape[:2]

        blob = cv2.dnn.blobFromImage(
            frame_bgr,
            scalefactor=1 / 255.0,
            size=(_INPUT_SIZE, _INPUT_SIZE),
            swapRB=True,
            crop=False,
        )
        net.setInput(blob)
        raw = net.forward()          # [1, 84, num_predictions]

        output = raw[0].T            # [num_predictions, 84]
        class_scores = output[:, 4:]
        class_ids    = class_scores.argmax(axis=1)
        confidences  = class_scores.max(axis=1)

        mask        = confidences >= _CONF_THRESHOLD
        output      = output[mask]
        class_ids   = class_ids[mask]
        confidences = confidences[mask]

        if len(output) == 0:
            return []

        # cx, cy, w_box, h_box in _INPUT_SIZE space → scale to original frame
        scale_x = w / _INPUT_SIZE
        scale_y = h / _INPUT_SIZE

        cx   = output[:, 0] * scale_x
        cy   = output[:, 1] * scale_y
        bw   = output[:, 2] * scale_x
        bh   = output[:, 3] * scale_y

        x1 = np.clip(cx - bw / 2, 0, w).astype(int)
        y1 = np.clip(cy - bh / 2, 0, h).astype(int)
        x2 = np.clip(cx + bw / 2, 0, w).astype(int)
        y2 = np.clip(cy + bh / 2, 0, h).astype(int)

        boxes_xywh = [[int(x1[i]), int(y1[i]), int(x2[i] - x1[i]), int(y2[i] - y1[i])]
                      for i in range(len(x1))]
        indices = cv2.dnn.NMSBoxes(
            boxes_xywh, confidences.tolist(), _CONF_THRESHOLD, _NMS_THRESHOLD
        )

        # NMSBoxes returns a flat array in OpenCV ≥4.7, nested in older versions
        if len(indices) == 0:
            return []
        flat = indices.flatten()

        return [
            {
                "x1":       int(x1[i]),
                "y1":       int(y1[i]),
                "x2":       int(x2[i]),
                "y2":       int(y2[i]),
                "conf":     float(confidences[i]),
                "class_id": int(class_ids[i]),
            }
            for i in flat
        ]

    except Exception as e:
        print(f"[object_detection] detect_obstacles error: {e}")
        return []


# ---------------------------------------------------------------------------
# Task 4 — priority selection and threat classification
# ---------------------------------------------------------------------------

def select_primary_obstacle(detections: list, frame_width: int = 640):
    """Return the highest-priority detection from a list.

    Priority: largest bounding-box area. Tie-break: closest to frame centre.
    Returns None if detections is empty.
    """
    if not detections:
        return None
    cx_frame = frame_width / 2.0
    return max(
        detections,
        key=lambda d: (
            (d["x2"] - d["x1"]) * (d["y2"] - d["y1"]),
            -abs(((d["x1"] + d["x2"]) / 2.0) - cx_frame),
        ),
    )


def classify_width_threat(detection: dict, frame_width: int = 640) -> str:
    """Classify a detection's apparent width relative to the frame.

    WIDE   — ≥ 50 % of frame width → treat as wall/barrier, no passing attempt
    MEDIUM — ≥ 25 % of frame width → measure and decide
    NARROW — < 25 % of frame width → single ping sufficient
    """
    ratio = (detection["x2"] - detection["x1"]) / frame_width
    if ratio >= 0.50:
        return "WIDE"
    elif ratio >= 0.25:
        return "MEDIUM"
    else:
        return "NARROW"


# ---------------------------------------------------------------------------
# Task 5 — pixel-to-servo angle mapping
# ---------------------------------------------------------------------------

_SERVO1_CENTER = SERVO_CFG["servo1"]["center_angle"]   # 89.85°
_SERVO1_MIN    = SERVO_CFG["servo1"]["max_angle"]      # 0°   — full right (smaller angle)
_SERVO1_MAX    = SERVO_CFG["servo1"]["min_angle"]      # 180° — full left  (larger angle)
_HFOV          = OBSTACLE_AVOIDANCE_CFG["camera_hfov_deg"]  # 102°


def pixel_x_to_servo_angle(pixel_x: float, frame_width: int = 640) -> int:
    """Map a camera frame pixel X position to a servo1 (head pan) angle in degrees.

    pixel_x = 0            → full left  (_SERVO1_MAX)
    pixel_x = frame_width/2 → centre    (_SERVO1_CENTER = 89.85°)
    pixel_x = frame_width   → full right (_SERVO1_MIN)

    The result is clamped to the servo's physical range and returned as int.
    """
    offset_frac = (pixel_x - frame_width / 2.0) / (frame_width / 2.0)  # [-1, 1]
    raw_angle   = _SERVO1_CENTER - offset_frac * (_HFOV / 2.0)
    return int(round(max(_SERVO1_MIN, min(_SERVO1_MAX, raw_angle))))


# ---------------------------------------------------------------------------
# Task 6 — ultrasonic sweep and physical width calculation
# ---------------------------------------------------------------------------

_SERVO_SETTLE_S = 0.10  # seconds to wait after each servo move before pinging


def sweep_obstacle(controller, ultrasonic, bbox_left_px: int, bbox_right_px: int,
                   frame_width: int = 640) -> dict:
    """Rotate the head servo to three positions across the bounding box and ping.

    Fires the ultrasonic at the left edge, centre, and right edge of the
    detected bounding box. Returns distances in cm for each position.

    Always restores the head to centre after the sweep regardless of errors.
    Intended to run in a thread-pool executor (blocks for ~300–450 ms total).

    Returns: {"left": float, "center": float, "right": float}
    """
    cx_px = (bbox_left_px + bbox_right_px) / 2.0
    positions = {
        "left":   bbox_left_px,
        "center": cx_px,
        "right":  bbox_right_px,
    }
    readings = {}
    try:
        for label, px in positions.items():
            angle = pixel_x_to_servo_angle(px, frame_width)
            controller.move_camera_to("x", angle)
            time.sleep(_SERVO_SETTLE_S)
            readings[label] = float(ultrasonic.distance_cm())
    finally:
        controller.center_camera()

    return readings


def calculate_real_width(bbox_pixel_width: float, distance_cm: float,
                         focal_length_px: float) -> float:
    """Estimate the physical width of an obstacle in centimetres.

    Formula: real_width = (pixel_width × distance) / focal_length

    focal_length_px is calibrated once and stored in hardware.yaml under
    obstacle_avoidance.focal_length_px. Default estimate: 554 px.
    """
    return (bbox_pixel_width * distance_cm) / focal_length_px


# ---------------------------------------------------------------------------
# Offline / SSH debug — MJPEG stream with YOLO bounding boxes
#
#   python3 -m src.perception.vision.object_detection
#   Then open http://<pi-ip>:8080 in a browser on your Mac.
# ---------------------------------------------------------------------------

# Subset of COCO class names for display; everything else shown as cls<id>.
_COCO_LABELS = {
    0: "person",    1: "bicycle",   2: "car",       3: "motorbike",
    4: "aeroplane", 5: "bus",       6: "train",     7: "truck",
    14: "bird",     15: "cat",      16: "dog",
    56: "chair",    57: "couch",    58: "plant",    59: "bed",
    60: "table",    62: "tv",       63: "laptop",   67: "phone",
    72: "fridge",   73: "book",     74: "clock",    76: "scissors",
}

if __name__ == "__main__":
    import threading
    from http.server import BaseHTTPRequestHandler, HTTPServer

    from src.perception.camera import make_camera, capture_bgr  # type: ignore
    from src.core.config import CAMERA_CFG                      # type: ignore

    _PORT   = 8080
    _shared: dict = {"jpg": None}
    _lock   = threading.Lock()

    class _Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with _lock:
                        jpg = _shared["jpg"]
                    if jpg is None:
                        time.sleep(0.05)
                        continue
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                        + jpg + b"\r\n"
                    )
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *_):
            pass

    fc  = CAMERA_CFG["front"]
    cam = make_camera(
        fc["index"],
        fc["main_width"], fc["main_height"],
        fc["lores_width"], fc["lores_height"],
        fc["framerate"],
        fc.get("rotate_180", False),
    )

    server = HTTPServer(("0.0.0.0", _PORT), _Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"YOLO stream — open http://<pi-ip>:{_PORT} in your browser.")
    print("Ctrl+C to stop.\n")

    try:
        while True:
            t0         = time.perf_counter()
            frame      = capture_bgr(cam)
            detections = detect_obstacles(frame)
            ms         = (time.perf_counter() - t0) * 1000

            vis = frame.copy()
            for d in detections:
                label = _COCO_LABELS.get(d["class_id"], f"cls{d['class_id']}")
                x1, y1, x2, y2 = d["x1"], d["y1"], d["x2"], d["y2"]
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(vis, f"{label} {d['conf']:.2f}",
                            (x1, max(y1 - 6, 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1,
                            cv2.LINE_AA)

            cv2.putText(vis, f"{len(detections)} det  {ms:.0f} ms",
                        (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (0, 200, 255), 1, cv2.LINE_AA)

            _, jpg = cv2.imencode(".jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, 80])
            with _lock:
                _shared["jpg"] = jpg.tobytes()

            if detections:
                names = ", ".join(
                    f"{_COCO_LABELS.get(d['class_id'], d['class_id'])} "
                    f"{d['conf']:.2f}"
                    for d in detections
                )
                print(f"[{ms:5.0f} ms]  {len(detections)} det: {names}")
            else:
                print(f"[{ms:5.0f} ms]  —")

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        server.shutdown()
        cam.stop()
