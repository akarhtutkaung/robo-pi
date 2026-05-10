"""
Obstacle detection — two levels:

  ObstacleDetector   — ultrasonic-based proximity wrapper (always on, Thread A)
  detect_obstacles() — YOLOv8n camera-based bounding-box detection (Thread B)

YOLO functions (tasks 3–6):
  detect_obstacles(frame_bgr)                      → list[dict]
  select_primary_obstacle(detections, frame_width) → dict | None
  classify_width_threat(detection, frame_width)    → "WIDE" | "MEDIUM" | "NARROW"
"""

import pathlib
import cv2
import numpy as np

from src.core.config import ULTRASONIC_CFG, OBSTACLE_AVOIDANCE_CFG
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
