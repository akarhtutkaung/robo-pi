import argparse
import sys

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Robo-Pi Robot System")
    parser.add_argument(
        "--mode",
        choices=["remote", "autonomous"],
        default="remote",
        help="Operating mode to run"
    )
    args = parser.parse_args()

    if args.mode == "remote":
        from src.core.modes.remote import run
    elif args.mode == "autonomous":
        print("[!] Autonomous mode not yet implemented.")
        sys.exit(1)

    run()
