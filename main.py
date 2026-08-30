import argparse
import logging
import sys
from src.components.core.logger import setup_logging
from src.features.manual_movement.remote import run

log = logging.getLogger(__name__)

if __name__ == "__main__":
    setup_logging()

    parser = argparse.ArgumentParser(description="Robo-Pi Robot System")
    parser.add_argument(
        "--mode",
        choices=["manual", "autonomous"],
        default="manual",
        help="Operating mode to run"
    )
    args = parser.parse_args()

    if args.mode == "autonomous":
        log.error("Autonomous mode not yet implemented.")
        sys.exit(1)

    run()
