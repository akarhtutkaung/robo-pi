"""
Face detection — OpenCV Haar cascade (haarcascade_frontalface_default.xml),
no model file to source, ships inside OpenCV itself. Classical CV, matching
this codebase's free_space.py approach — no learned weights.
"""
import logging
import pathlib
import threading

import cv2

from src.components.core.config import FACE_TRACKING_CFG

log = logging.getLogger(__name__)

_MIN_FACE_SIZE_PX      = FACE_TRACKING_CFG["min_face_size_px"]
_CASCADE_PATH_OVERRIDE = FACE_TRACKING_CFG["cascade_path"]

_cascade = None
_cascade_lock = threading.Lock()


def _resolve_cascade_path() -> str:
    """Find a usable haarcascade_frontalface_default.xml.

    Tries, in order: an explicit hardware.yaml override, cv2.data.haarcascades
    (present with pip's opencv-python, not always with apt's python3-opencv —
    this project's Pi dependency per requirements.txt), then the common
    Debian / Raspberry Pi OS apt install locations.
    """
    if _CASCADE_PATH_OVERRIDE:
        return _CASCADE_PATH_OVERRIDE

    candidates = []
    data_module = getattr(cv2, "data", None)
    if data_module is not None:
        candidates.append(str(pathlib.Path(data_module.haarcascades) / "haarcascade_frontalface_default.xml"))
    candidates += [
        "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
        "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
        "/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
    ]
    for path in candidates:
        if pathlib.Path(path).is_file():
            return path

    raise FileNotFoundError(
        "Could not find haarcascade_frontalface_default.xml — set face_tracking.cascade_path "
        "in hardware.yaml, or install the OpenCV data files."
    )


def load_cascade() -> cv2.CascadeClassifier:
    """Lazy-load the cascade classifier. Thread-safe via double-checked locking
    (mirrors object_detection.py's _load_models). Public so callers (tracker.py's
    setup()) can force-load it up front and fail fast before the loop starts.
    """
    global _cascade
    if _cascade is not None:
        return _cascade
    with _cascade_lock:
        if _cascade is not None:
            return _cascade
        path = _resolve_cascade_path()
        cascade = cv2.CascadeClassifier(path)
        if cascade.empty():
            raise IOError(f"Failed to load cascade classifier from {path}")
        log.info("Loaded face cascade: %s", path)
        _cascade = cascade
        return _cascade


def detect_faces(frame_bgr) -> list[dict]:
    """Detect faces in a BGR frame. Returns a list of {x1,y1,x2,y2} boxes,
    filtered to at least min_face_size_px on a side.
    """
    cascade = load_cascade()
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    boxes = cascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(_MIN_FACE_SIZE_PX, _MIN_FACE_SIZE_PX),
    )
    return [
        {"x1": int(x), "y1": int(y), "x2": int(x + w), "y2": int(y + h)}
        for (x, y, w, h) in boxes
    ]
