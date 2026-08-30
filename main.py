import argparse
import sys
from src.features.manual_movement.remote import run

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Robo-Pi Robot System")
    parser.add_argument(
        "--mode",
        choices=["manual", "autonomous"],
        default="manual",
        help="Operating mode to run"
    )
    args = parser.parse_args()

    if args.mode == "autonomous":
        print("[!] Autonomous mode not yet implemented.")
        sys.exit(1)

    run()
