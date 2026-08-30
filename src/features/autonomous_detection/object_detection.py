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

Multi-model support:
  Models are configured via obstacle_avoidance.yolo_models in hardware.yaml.
  Each enabled model runs independently; detections are merged via union + NMS dedup.
  Activate the custom model by setting enabled: true for its entry and dropping the
  ONNX file into src/components/ai/models/ — no code changes required.

Debug stream (SSH → browser):
  On the Pi:
    cd ~/robo-pi
    python3 -m src.features.autonomous_detection.object_detection

  On your Mac, open:
    http://<pi-ip>:8080

  The stream shows live YOLO bounding boxes (label + confidence) drawn on the
  640×480 lores camera frame — the same resolution used by autonomous mode.
  Inference time is printed per frame to stdout and overlaid on the stream.
  Press Ctrl+C on the Pi to stop.
"""

import logging
import math
import pathlib
import threading
import time
import cv2
import numpy as np

from src.components.core.config import ULTRASONIC_CFG, OBSTACLE_AVOIDANCE_CFG, SERVO_CFG
from src.components.hardware.sensors.ultrasonic import UltrasonicSensor

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# ObstacleDetector — ultrasonic proximity (used by Thread A)
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

_PROJECT_ROOT   = pathlib.Path(__file__).parents[3]
_CONF_THRESHOLD = 0.4
_NMS_THRESHOLD  = 0.45

# Each loaded model: {"net": cv2.dnn_Net, "input_size": int, "class_names": list[str]}
_models: list[dict] = []
_label_lookup: dict[int, str] = {}   # merged from per-model class_names at load time
_models_lock = threading.Lock()


def _read_model_configs() -> list[dict]:
    """Return enabled model specs from hardware.yaml.

    Reads obstacle_avoidance.yolo_models list (new format) if present, otherwise
    falls back to the deprecated scalar yolo_model / yolo_input_size / yolo_class_names keys.
    """
    cfg = OBSTACLE_AVOIDANCE_CFG
    entries = cfg.get("yolo_models")
    if entries:
        return [
            {
                "path":        str(_PROJECT_ROOT / e["path"]),
                "input_size":  int(e["input_size"]),
                "class_names": list(e.get("class_names") or []),
            }
            for e in entries
            if e.get("enabled", True)
        ]
    return [{
        "path":        str(_PROJECT_ROOT / cfg["yolo_model"]),
        "input_size":  int(cfg["yolo_input_size"]),
        "class_names": list(cfg.get("yolo_class_names") or []),
    }]


def _load_models() -> list[dict]:
    """Lazy-load all enabled YOLO models. Thread-safe via double-checked locking."""
    global _models, _label_lookup
    if _models:
        return _models
    with _models_lock:
        if _models:
            return _models
        for spec in _read_model_configs():
            try:
                net = cv2.dnn.readNetFromONNX(spec["path"])
                _models.append({
                    "net":         net,
                    "input_size":  spec["input_size"],
                    "class_names": spec["class_names"],
                })
                for idx, name in enumerate(spec["class_names"]):
                    _label_lookup[idx] = name
                log.info("Loaded model: %s @ %spx", spec['path'], spec['input_size'])
            except Exception:
                log.exception("Failed to load model %s", spec['path'])
    return _models


def _run_single_model(
    frame_bgr: np.ndarray,
    net: cv2.dnn.Net,
    input_size: int,
) -> list[dict]:
    """Run inference on one loaded model. Returns per-model NMS-filtered detections."""
    h, w = frame_bgr.shape[:2]

    blob = cv2.dnn.blobFromImage(
        frame_bgr,
        scalefactor=1 / 255.0,
        size=(input_size, input_size),
        swapRB=True,
        crop=False,
    )
    net.setInput(blob)
    raw = net.forward()          # [1, 84, num_predictions]

    output      = raw[0].T       # [num_predictions, 84]
    class_scores = output[:, 4:]
    class_ids    = class_scores.argmax(axis=1)
    confidences  = class_scores.max(axis=1)

    mask        = confidences >= _CONF_THRESHOLD
    output      = output[mask]
    class_ids   = class_ids[mask]
    confidences = confidences[mask]

    if len(output) == 0:
        return []

    scale_x = w / input_size
    scale_y = h / input_size

    cx  = output[:, 0] * scale_x
    cy  = output[:, 1] * scale_y
    bw  = output[:, 2] * scale_x
    bh  = output[:, 3] * scale_y

    x1 = np.clip(cx - bw / 2, 0, w).astype(int)
    y1 = np.clip(cy - bh / 2, 0, h).astype(int)
    x2 = np.clip(cx + bw / 2, 0, w).astype(int)
    y2 = np.clip(cy + bh / 2, 0, h).astype(int)

    boxes_xywh = [[int(x1[i]), int(y1[i]), int(x2[i] - x1[i]), int(y2[i] - y1[i])]
                  for i in range(len(x1))]
    indices = cv2.dnn.NMSBoxes(
        boxes_xywh, confidences.tolist(), _CONF_THRESHOLD, _NMS_THRESHOLD
    )

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


def _nms_dedup(detections: list[dict]) -> list[dict]:
    """Class-agnostic NMS pass to remove duplicate boxes across models.

    When both models detect the same physical obstacle, the higher-confidence box
    survives. Class-agnostic is correct because the avoidance pipeline never reads
    class_id — only bounding box coordinates matter for navigation decisions.
    """
    if not detections:
        return []
    boxes_xywh  = [[d["x1"], d["y1"], d["x2"] - d["x1"], d["y2"] - d["y1"]]
                   for d in detections]
    confidences = [d["conf"] for d in detections]
    indices = cv2.dnn.NMSBoxes(boxes_xywh, confidences, _CONF_THRESHOLD, _NMS_THRESHOLD)
    if len(indices) == 0:
        return []
    return [detections[i] for i in indices.flatten()]


def detect_obstacles(frame_bgr: np.ndarray) -> list:
    """Run all enabled YOLO models on a BGR frame.

    Returns a merged list of dicts — one per surviving obstacle after per-model NMS
    and a single cross-model NMS dedup pass:
        {"x1": int, "y1": int, "x2": int, "y2": int, "conf": float, "class_id": int}
    Coordinates are in the original frame's pixel space.
    Returns [] on empty frame, no loaded models, or any inference error.

    When only one model is enabled the cross-model dedup pass is skipped entirely
    (single-model fast path — zero overhead vs previous behaviour).
    """
    if frame_bgr is None or frame_bgr.size == 0:
        return []
    try:
        models = _load_models()
        if not models:
            return []

        if len(models) == 1:
            m = models[0]
            return _run_single_model(frame_bgr, m["net"], m["input_size"])

        all_dets: list[dict] = []
        for m in models:
            try:
                all_dets.extend(_run_single_model(frame_bgr, m["net"], m["input_size"]))
            except Exception:
                log.exception("model inference error")

        return _nms_dedup(all_dets)

    except Exception:
        log.exception("detect_obstacles error")
        return []


# ---------------------------------------------------------------------------
# priority selection and threat classification
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

    WIDE   — ≥ 50 % of frame width AND centred within 30 % of the frame midpoint
              → treat as wall/barrier, no passing attempt
    MEDIUM — ≥ 25 % of frame width (or WIDE threshold but off-centre)
              → measure and decide
    NARROW — < 25 % of frame width → single ping sufficient

    The position check on WIDE prevents a wall or large obstacle at the frame edge
    (e.g., a wall to the left) from triggering REVERSE_AND_TURN when turning away
    from it would be the correct manoeuvre.
    """
    ratio = (detection["x2"] - detection["x1"]) / frame_width
    center_x = (detection["x1"] + detection["x2"]) / 2.0
    offset_ratio = abs(center_x - frame_width / 2.0) / (frame_width / 2.0)
    if ratio >= 0.50 and offset_ratio < 0.3:
        return "WIDE"
    elif ratio >= 0.25:
        return "MEDIUM"
    else:
        return "NARROW"


# ---------------------------------------------------------------------------
# pixel-to-servo angle mapping
# ---------------------------------------------------------------------------

_SERVO1_CENTER   = SERVO_CFG["servo1"]["center_angle"]        # 89.85°
_SERVO1_MIN      = SERVO_CFG["servo1"]["max_angle"]           # 0°   — full right (smaller angle)
_SERVO1_MAX      = SERVO_CFG["servo1"]["min_angle"]           # 180° — full left  (larger angle)
_FOCAL_LENGTH_PX = OBSTACLE_AVOIDANCE_CFG["focal_length_px"]  # 259 px (geometric; calibrate on device)


def pixel_x_to_servo_angle(pixel_x: float, frame_width: int = 640) -> int:
    """Map a camera frame pixel X position to a servo1 (head pan) angle in degrees.

    pixel_x = 0              → full left  (≈ _SERVO1_MAX)
    pixel_x = frame_width/2  → centre     (_SERVO1_CENTER = 89.85°)
    pixel_x = frame_width    → full right (≈ _SERVO1_MIN)

    Uses the pinhole/atan model rather than a linear approximation so that
    the mapping stays accurate across the full 102° horizontal FOV of the
    Pi Camera V3 Wide Angle. Result is clamped to the servo's physical range.
    """
    offset_px = pixel_x - frame_width / 2.0
    angle_deg = math.degrees(math.atan2(offset_px, _FOCAL_LENGTH_PX))
    raw_angle = _SERVO1_CENTER - angle_deg
    return int(round(max(_SERVO1_MIN, min(_SERVO1_MAX, raw_angle))))


# ---------------------------------------------------------------------------
# ultrasonic sweep and physical width calculation
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
# Annotation helper — used by the debug stream server and __main__
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


def _class_label(class_id: int) -> str:
    """Resolve a class_id to a display label.

    Priority: per-model class_names from config → COCO labels → "cls<N>".
    _label_lookup is populated from all enabled models at load time; custom
    model names (e.g. "obstacle" at id 0) override COCO for debug display.
    """
    if class_id in _label_lookup:
        return _label_lookup[class_id]
    return _COCO_LABELS.get(class_id, f"cls{class_id}")


def draw_detections(vis: np.ndarray, detections: list) -> np.ndarray:
    """Draw YOLO bounding boxes and labels onto vis (modifies in-place, returns vis).

    Intended to be called after draw_debug() from free_space so both overlays
    appear on the same frame.
    """
    for d in detections:
        label = _class_label(d["class_id"])
        x1, y1, x2, y2 = d["x1"], d["y1"], d["x2"], d["y2"]
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(vis, f"{label} {d['conf']:.2f}",
                    (x1, max(y1 - 6, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)
    return vis


# ---------------------------------------------------------------------------
# Offline / SSH debug — MJPEG stream with YOLO bounding boxes
#
#   python3 -m src.features.autonomous_detection.object_detection
#   Then open http://<pi-ip>:8080 in a browser on your Mac.
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    from http.server import BaseHTTPRequestHandler, HTTPServer

    from src.components.camera.camera import make_camera, capture_bgr  # type: ignore
    from src.components.core.config import CAMERA_CFG                      # type: ignore

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
    obstacle = ObstacleDetector()

    server = HTTPServer(("0.0.0.0", _PORT), _Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"YOLO stream — open http://<pi-ip>:{_PORT} in your browser.")
    print("Ctrl+C to stop.\n")

    try:
        while True:
            t0         = time.perf_counter()
            frame      = capture_bgr(cam)
            detections = detect_obstacles(frame)
            dist_cm    = obstacle.distance_cm()
            ms         = (time.perf_counter() - t0) * 1000

            vis = draw_detections(frame.copy(), detections)

            # Ultrasonic distance — colour indicates phase
            if dist_cm <= STOP_CM:
                dist_colour = (0, 0, 255)    # red   — blocked
            elif dist_cm <= TURN_CM:
                dist_colour = (0, 165, 255)  # orange — approaching
            else:
                dist_colour = (0, 255, 0)    # green  — clear

            cv2.putText(vis, f"{dist_cm:.1f} cm",
                        (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        dist_colour, 1, cv2.LINE_AA)
            cv2.putText(vis, f"{len(detections)} det  {ms:.0f} ms",
                        (6, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (0, 200, 255), 1, cv2.LINE_AA)

            _, jpg = cv2.imencode(".jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, 80])
            with _lock:
                _shared["jpg"] = jpg.tobytes()

            phase = ("BLOCKED" if dist_cm <= STOP_CM
                     else "APPROACH" if dist_cm <= TURN_CM
                     else "CLEAR")
            if detections:
                names = ", ".join(
                    f"{_class_label(d['class_id'])} {d['conf']:.2f}"
                    for d in detections
                )
                print(f"[{ms:5.0f} ms]  {dist_cm:5.1f} cm [{phase}]  {len(detections)} det: {names}")
            else:
                print(f"[{ms:5.0f} ms]  {dist_cm:5.1f} cm [{phase}]  —")

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        cam.stop()
        obstacle.cleanup()
