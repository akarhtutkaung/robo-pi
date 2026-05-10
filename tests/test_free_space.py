"""
Tests for src/perception/vision/free_space.py

All tests use synthetic numpy frames — no camera hardware needed.
"""
import numpy as np
import pytest

from src.perception.vision.free_space import (
    detect,
    MIN_CONFIDENCE,
    ROI_LEFT, ROI_RIGHT, ROI_TOP, ROI_BOTTOM,
    FRAME_W, FRAME_H,
)


def _white(w=FRAME_W, h=FRAME_H):
    """Uniform white-tile frame (high passability everywhere)."""
    return np.full((h, w, 3), 200, dtype=np.uint8)


def _black(w=FRAME_W, h=FRAME_H):
    return np.zeros((h, w, 3), dtype=np.uint8)


# ---------------------------------------------------------------------------
# Output range
# ---------------------------------------------------------------------------

def test_detect_returns_error_in_range():
    err, _ = detect(_white())
    assert -1.0 <= err <= 1.0


def test_detect_returns_confidence_in_range():
    _, conf = detect(_white())
    assert 0.0 <= conf <= 1.0


# ---------------------------------------------------------------------------
# High-confidence frame — split floors with strong lateral signal
# ---------------------------------------------------------------------------

def test_detect_split_frame_has_confidence():
    """A clearly split frame (bright left, dark right) should exceed MIN_CONFIDENCE."""
    frame = np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)
    frame[:, :FRAME_W // 2] = 200
    _, conf = detect(frame)
    assert conf >= MIN_CONFIDENCE, (
        f"Split frame should have detectable free-space signal, got conf={conf:.3f}"
    )


def test_detect_black_frame_low_confidence():
    _, conf = detect(_black())
    assert conf < MIN_CONFIDENCE


# ---------------------------------------------------------------------------
# Auto-resize — frames smaller than FRAME_W×FRAME_H are accepted
# ---------------------------------------------------------------------------

def test_detect_small_frame_auto_resized():
    small = _white(320, 240)
    err, conf = detect(small)
    assert -1.0 <= err <= 1.0
    assert 0.0 <= conf <= 1.0


# ---------------------------------------------------------------------------
# Lateral free-space detection
# Each test paints bright floor colour on one half and darkness on the other.
# The floor-colour signal should dominate and pull error toward the bright side.
# ---------------------------------------------------------------------------

def _split_frame(bright_left: bool) -> np.ndarray:
    """640×480 frame: one half white (floor), one half dark (obstacle)."""
    frame = np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)
    mid = FRAME_W // 2
    if bright_left:
        frame[:, :mid] = 200
    else:
        frame[:, mid:] = 200
    return frame


def test_detect_left_free_space_error_negative():
    """White left half → free lane is left of centre → error < 0."""
    err, conf = detect(_split_frame(bright_left=True))
    assert conf >= MIN_CONFIDENCE
    assert err < 0, f"Expected negative error for left free space, got {err:+.3f}"


def test_detect_right_free_space_error_positive():
    """White right half → free lane is right of centre → error > 0."""
    err, conf = detect(_split_frame(bright_left=False))
    assert conf >= MIN_CONFIDENCE
    assert err > 0, f"Expected positive error for right free space, got {err:+.3f}"


# ---------------------------------------------------------------------------
# ROI constants sanity
# ---------------------------------------------------------------------------

def test_roi_left_less_than_roi_right():
    assert ROI_LEFT < ROI_RIGHT


def test_roi_top_less_than_roi_bottom():
    assert ROI_TOP < ROI_BOTTOM


def test_roi_within_frame():
    assert ROI_LEFT >= 0
    assert ROI_RIGHT <= FRAME_W
    assert ROI_TOP >= 0
    assert ROI_BOTTOM <= FRAME_H
