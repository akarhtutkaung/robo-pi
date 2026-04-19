"""
TUTORIAL: main.py
==================
PURPOSE:
    Entry point. Run this file on the Pi to start the robot system.
    Selects the operating mode and hands off to it.

STEP 1 — Import the mode you want to run
    from src.core.modes.remote import run

STEP 2 — Call run() inside a if __name__ == "__main__" guard

    Example:
        if __name__ == "__main__":
            run()

STEP 3 — (Optional) Accept a mode argument from the command line
    As you add more modes (autonomous, speech), you can use sys.argv or
    argparse to let you choose the mode at launch time instead of editing
    this file.

    Example with argparse:
        import argparse

        parser = argparse.ArgumentParser()
        parser.add_argument(
            "--mode",
            choices=["remote", "autonomous"],
            default="remote"
        )
        args = parser.parse_args()

        if args.mode == "remote":
            from src.core.modes.remote import run
        elif args.mode == "autonomous":
            from src.core.modes.autonomous import run

        run()

STEP 4 — How to run on the Pi
    From the repo root:
        python3 main.py
        python3 main.py --mode remote    # once you add the argparse step

STEP 5 — (Optional) Run as a systemd service so it starts on boot
    Create /etc/systemd/system/robo-pi.service:

        [Unit]
        Description=Robo-Pi Robot System
        After=network.target

        [Service]
        ExecStart=/usr/bin/python3 /home/pi/robo-pi/main.py
        WorkingDirectory=/home/pi/robo-pi
        Restart=on-failure
        User=pi

        [Install]
        WantedBy=multi-user.target

    Then enable and start it:
        sudo systemctl enable robo-pi
        sudo systemctl start robo-pi
"""
import argparse

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
        print("[Coming Soon] Autonomous mode not implemented yet.")
        from src.core.modes.autonomous import run  # Placeholder for future mode

    run()