"""
Capture training frames from the robot's front camera.

Saves 640×480 JPEG frames to data/frames/<YYYY-MM-DD_HH-MM-SS>/ at a
configurable rate. Run on the Pi while driving the robot through the
environment you want to detect obstacles in.

Usage:
    python3 tools/collect_frames.py                       # 1 fps, stop with Ctrl+C
    python3 tools/collect_frames.py --fps 2               # 2 fps
    python3 tools/collect_frames.py --fps 2 --max 500     # stop after 500 frames
    python3 tools/collect_frames.py --out /tmp/frames     # custom output directory
"""

import argparse
import pathlib
import sys
import time
from datetime import datetime

import cv2

# Project root on sys.path so src.* imports resolve when run as a script.
_ROOT = pathlib.Path(__file__).parent.parent
sys.path.insert(0, str(_ROOT))

from src.perception.camera import make_camera, capture_bgr  # noqa: E402
from src.core.config import CAMERA_CFG                      # noqa: E402


def _parse_args():
    p = argparse.ArgumentParser(description="Capture training frames from the robot camera.")
    p.add_argument("--fps",  type=float, default=1.0,
                   help="Capture rate in frames per second (default: 1.0, recommended max: 2.0)")
    p.add_argument("--max",  type=int,   default=0,
                   help="Stop after this many frames (default: 0 = unlimited)")
    p.add_argument("--out",  type=str,   default="",
                   help="Output parent directory (default: data/frames relative to project root)")
    return p.parse_args()


def main():
    args = _parse_args()

    fc  = CAMERA_CFG["front"]
    cam = make_camera(
        fc["index"],
        fc["main_width"], fc["main_height"],
        fc["lores_width"], fc["lores_height"],
        fc["framerate"],
        fc.get("rotate_180", False),
    )

    ts      = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    out_dir = pathlib.Path(args.out or (_ROOT / "data" / "frames")) / ts
    out_dir.mkdir(parents=True, exist_ok=True)

    interval  = 1.0 / max(args.fps, 0.1)
    count     = 0
    start     = time.monotonic()

    print(f"Saving to:  {out_dir}")
    print(f"Rate:       {args.fps:.1f} fps  (1 frame every {interval:.2f} s)")
    print(f"Max frames: {args.max if args.max else 'unlimited'}")
    print("Press Ctrl+C to stop.\n")

    try:
        while True:
            t0    = time.monotonic()
            frame = capture_bgr(cam)
            name  = out_dir / f"{count:05d}.jpg"
            cv2.imwrite(str(name), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            count += 1
            elapsed = time.monotonic() - start
            print(f"\r  {count} frames  {elapsed:.0f}s elapsed", end="", flush=True)

            if args.max and count >= args.max:
                print(f"\nReached --max {args.max}. Done.")
                break

            remaining = interval - (time.monotonic() - t0)
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        print(f"\nStopped. {count} frames saved to {out_dir}")
    finally:
        cam.stop()


if __name__ == "__main__":
    main()
