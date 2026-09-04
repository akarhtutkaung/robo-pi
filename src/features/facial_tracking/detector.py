"""
Face detection — two backends, picked at load time by _select_backend():

  YuNet (preferred)   cv2.FaceDetectorYN over a ~340 KB ONNX model. A real
                      detector, not a frontal template: holds a face through
                      roughly +/-60-90 deg of yaw (head turned left/right),
                      through roll (head tilted), and through partial
                      occlusion. ~5-15 ms per 640x480 frame on a Pi.

  Cascade ensemble    Automatic fallback when the ONNX file is missing or the
  (fallback)          installed OpenCV can't load it. Runs the frontal cascade
                      plus the profile cascade twice — once normally, once on
                      a mirrored frame, since haarcascade_profileface only
                      fires on faces turned one way — and merges the boxes
                      with NMS. Recovers yaw, but Haar is not rotation
                      invariant so roll beyond ~15-20 deg is still lost.

Both backends return the same shape: list of {x1, y1, x2, y2, score} boxes,
filtered to at least min_face_size_px on a side.

Deploying the YuNet model (see setup.sh):
    curl -L -o src/components/ai/models/face_detection_yunet.onnx \\
      https://github.com/opencv/opencv_zoo/raw/main/models/face_detection_yunet/face_detection_yunet_2023mar.onnx
"""
import logging
import pathlib
import threading

import cv2
import numpy as np

from src.components.core.config import FACE_TRACKING_CFG

log = logging.getLogger(__name__)

_PROJECT_ROOT = pathlib.Path(__file__).parents[3]

_MIN_FACE_SIZE_PX      = FACE_TRACKING_CFG["min_face_size_px"]
_CASCADE_PATH_OVERRIDE = FACE_TRACKING_CFG["cascade_path"]
_PROFILE_PATH_OVERRIDE = FACE_TRACKING_CFG.get("profile_cascade_path", "")
_BACKEND_CHOICE        = FACE_TRACKING_CFG.get("detector", "auto")

_YUNET_MODEL      = FACE_TRACKING_CFG.get("yunet_model", "")
_YUNET_SCORE_THRESH = FACE_TRACKING_CFG.get("yunet_score_threshold", 0.6)
_YUNET_NMS_THRESH   = FACE_TRACKING_CFG.get("yunet_nms_threshold", 0.3)
_YUNET_TOP_K        = FACE_TRACKING_CFG.get("yunet_top_k", 50)

# Nominal confidences for the score-less cascade backend, used only to order
# NMS: a frontal hit outranks a profile hit covering the same region.
_FRONTAL_SCORE = 1.0
_PROFILE_SCORE = 0.9
_CASCADE_NMS_THRESHOLD = 0.3

_FRONTAL_CASCADE_FILE = "haarcascade_frontalface_default.xml"
_PROFILE_CASCADE_FILE = "haarcascade_profileface.xml"

_backend = None
_backend_lock = threading.Lock()
# YuNet's detect() is stateful — setInputSize() mutates the detector — so calls
# are serialised. detect_faces runs in an executor thread; only one tick is in
# flight at a time, so this never actually contends.
_detect_lock = threading.Lock()


# ---------------------------------------------------------------------------
# model / cascade file resolution
# ---------------------------------------------------------------------------

def _resolve_cascade_path(filename: str = _FRONTAL_CASCADE_FILE, override: str = "") -> str:
    """Find a usable Haar cascade XML.

    Tries, in order: an explicit hardware.yaml override, cv2.data.haarcascades
    (present with pip's opencv-python, not always with apt's python3-opencv —
    this project's Pi dependency per requirements.txt), then the common
    Debian / Raspberry Pi OS apt install locations.
    """
    if override:
        return override

    candidates = []
    data_module = getattr(cv2, "data", None)
    if data_module is not None:
        candidates.append(str(pathlib.Path(data_module.haarcascades) / filename))
    candidates += [
        f"/usr/share/opencv4/haarcascades/{filename}",
        f"/usr/share/opencv/haarcascades/{filename}",
        f"/usr/local/share/opencv4/haarcascades/{filename}",
    ]
    for path in candidates:
        if pathlib.Path(path).is_file():
            return path

    raise FileNotFoundError(
        f"Could not find {filename} — set face_tracking.cascade_path in "
        "hardware.yaml, or install the OpenCV data files."
    )


def _yunet_model_path() -> pathlib.Path | None:
    """Absolute path to the YuNet ONNX file, or None if not configured/present."""
    if not _YUNET_MODEL:
        return None
    path = pathlib.Path(_YUNET_MODEL)
    if not path.is_absolute():
        path = _PROJECT_ROOT / path
    return path if path.is_file() else None


# ---------------------------------------------------------------------------
# backends
# ---------------------------------------------------------------------------

class _YuNetBackend:
    """cv2.FaceDetectorYN — pose-tolerant, needs the ONNX model file."""

    name = "yunet"

    def __init__(self, model_path: str):
        # Constructor moved from a free function to a static method in OpenCV 4.6;
        # apt's python3-opencv on Raspberry Pi OS may still ship the older name.
        create = getattr(cv2, "FaceDetectorYN", None)
        create = getattr(create, "create", None) or getattr(cv2, "FaceDetectorYN_create", None)
        if create is None:
            raise AttributeError(f"OpenCV {cv2.__version__} has no FaceDetectorYN")
        self._det = create(
            model_path, "", (320, 320),
            _YUNET_SCORE_THRESH, _YUNET_NMS_THRESH, _YUNET_TOP_K,
        )
        self._input_size = (320, 320)
        # Warm-up, and a load-time check that inference actually runs: an ONNX the
        # installed OpenCV can construct but not execute (2023mar on OpenCV < 4.7)
        # must fall back to cascades here, not raise on every tick once live.
        self.detect(np.zeros((240, 320, 3), dtype=np.uint8))

    def detect(self, frame_bgr) -> list[dict]:
        h, w = frame_bgr.shape[:2]
        if (w, h) != self._input_size:
            self._det.setInputSize((w, h))
            self._input_size = (w, h)
        _, faces = self._det.detect(frame_bgr)
        if faces is None:
            return []
        # Each row: x, y, w, h, then 5 landmark x/y pairs, then the score last.
        boxes = []
        for row in faces:
            x, y, fw, fh = row[:4]
            boxes.append({
                "x1": int(x), "y1": int(y),
                "x2": int(x + fw), "y2": int(y + fh),
                "score": float(row[-1]),
            })
        return boxes


class _CascadeBackend:
    """Haar ensemble: frontal + profile + mirrored profile, merged with NMS."""

    name = "cascade"

    def __init__(self):
        self._frontal = self._load(_resolve_cascade_path(_FRONTAL_CASCADE_FILE, _CASCADE_PATH_OVERRIDE))
        try:
            self._profile = self._load(_resolve_cascade_path(_PROFILE_CASCADE_FILE, _PROFILE_PATH_OVERRIDE))
        except (FileNotFoundError, IOError):
            # Frontal-only is degraded but still usable — don't fail the mode over it.
            log.warning("Profile cascade unavailable — faces turned left/right will be missed.")
            self._profile = None

    @staticmethod
    def _load(path: str) -> cv2.CascadeClassifier:
        cascade = cv2.CascadeClassifier(path)
        if cascade.empty():
            raise IOError(f"Failed to load cascade classifier from {path}")
        log.info("Loaded face cascade: %s", path)
        return cascade

    @staticmethod
    def _run(cascade: cv2.CascadeClassifier, gray) -> list:
        return cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(_MIN_FACE_SIZE_PX, _MIN_FACE_SIZE_PX),
        )

    def detect(self, frame_bgr) -> list[dict]:
        gray = cv2.equalizeHist(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY))
        width = gray.shape[1]

        rects: list[list[int]] = []
        scores: list[float] = []

        for (x, y, w, h) in self._run(self._frontal, gray):
            rects.append([int(x), int(y), int(w), int(h)])
            scores.append(_FRONTAL_SCORE)

        if self._profile is not None:
            for (x, y, w, h) in self._run(self._profile, gray):
                rects.append([int(x), int(y), int(w), int(h)])
                scores.append(_PROFILE_SCORE)
            # haarcascade_profileface only fires on faces turned one way; the
            # mirrored pass catches the other. Boxes are un-mirrored back.
            flipped = cv2.flip(gray, 1)
            for (x, y, w, h) in self._run(self._profile, flipped):
                rects.append([int(width - x - w), int(y), int(w), int(h)])
                scores.append(_PROFILE_SCORE)

        return _nms(rects, scores, _CASCADE_NMS_THRESHOLD)


def _nms(rects: list[list[int]], scores: list[float], threshold: float) -> list[dict]:
    """Deduplicate overlapping boxes across the ensemble's passes."""
    if not rects:
        return []
    keep = cv2.dnn.NMSBoxes(rects, scores, score_threshold=0.0, nms_threshold=threshold)
    if len(keep) == 0:
        return []
    return [
        {
            "x1": rects[i][0], "y1": rects[i][1],
            "x2": rects[i][0] + rects[i][2], "y2": rects[i][1] + rects[i][3],
            "score": scores[i],
        }
        for i in np.asarray(keep).flatten()
    ]


# ---------------------------------------------------------------------------
# loading + public API
# ---------------------------------------------------------------------------

def _select_backend():
    """Build the best available backend. YuNet if the model is deployed and the
    installed OpenCV can load it, cascade ensemble otherwise.
    """
    if _BACKEND_CHOICE != "cascade":
        model_path = _yunet_model_path()
        if model_path is None:
            if _BACKEND_CHOICE == "yunet":
                raise FileNotFoundError(
                    f"face_tracking.detector is 'yunet' but the model is missing: {_YUNET_MODEL}"
                )
            log.warning(
                "YuNet model not found at %s — falling back to Haar cascades. Face tracking "
                "will lose the target when the head turns or tilts; see setup.sh to deploy it.",
                _YUNET_MODEL or "(unset)",
            )
        else:
            try:
                backend = _YuNetBackend(str(model_path))
                log.info("Loaded YuNet face detector: %s (OpenCV %s)", model_path, cv2.__version__)
                return backend
            except Exception:
                if _BACKEND_CHOICE == "yunet":
                    raise
                # A 2023mar model on an OpenCV older than 4.7 lands here.
                log.exception(
                    "YuNet failed to load on OpenCV %s — falling back to Haar cascades.",
                    cv2.__version__,
                )

    return _CascadeBackend()


def load_detector():
    """Lazy-load the face detector. Thread-safe via double-checked locking
    (mirrors object_detection.py's _load_models). Public so callers (tracker.py's
    setup()) can force-load it up front and fail fast before the loop starts.
    """
    global _backend
    if _backend is not None:
        return _backend
    with _backend_lock:
        if _backend is None:
            _backend = _select_backend()
        return _backend


def detector_name() -> str:
    """Which backend is active — 'yunet' or 'cascade'. For startup logging."""
    return load_detector().name


def detect_faces(frame_bgr) -> list[dict]:
    """Detect faces in a BGR frame. Returns a list of {x1,y1,x2,y2,score} boxes,
    filtered to at least min_face_size_px on a side.
    """
    backend = load_detector()
    with _detect_lock:
        boxes = backend.detect(frame_bgr)
    return [
        b for b in boxes
        if (b["x2"] - b["x1"]) >= _MIN_FACE_SIZE_PX and (b["y2"] - b["y1"]) >= _MIN_FACE_SIZE_PX
    ]