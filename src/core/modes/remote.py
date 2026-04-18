"""
TUTORIAL: src/core/modes/remote.py
=====================================
PURPOSE:
    Wires everything together for remote (WebSocket-driven) mode.
    This is the top-level setup for the current use case:
        Mac (OpenCV) → WebSocket → Pi → motors + servos

    Creates the RobotController, starts the WebSocket server, and handles
    clean shutdown on Ctrl+C or SIGTERM.

STEP 1 — Import everything
    import asyncio
    import signal
    from src.navigation.controller import RobotController
    from src.comms.websocket_server import start_server

STEP 2 — Write a run() function
    This is the single entry point called by main.py.
    It should:
      a) Create the RobotController instance (this initializes hardware).
      b) Register a cleanup handler for SIGINT/SIGTERM.
      c) Start the asyncio event loop and run start_server().

    Example:
        def run():
            controller = RobotController()

            try:
                asyncio.run(start_server(controller))
            except KeyboardInterrupt:
                print("Shutting down...")
            finally:
                controller.cleanup()
                print("Robot stopped safely.")

STEP 3 — Handle shutdown signals (optional but recommended on Pi)
    When the Pi receives SIGTERM (e.g. from systemd or kill command),
    you want the robot to stop cleanly. Use signal.signal() to catch it
    and call controller.cleanup() before exiting.

    Example (add inside run(), before asyncio.run()):
        def on_shutdown(sig, frame):
            controller.cleanup()
            raise SystemExit(0)

        signal.signal(signal.SIGTERM, on_shutdown)

STEP 4 — Startup checklist before running
    Before running for the first time, verify:
      [ ] PCA9685 is connected to the Pi I2C pins (SCL, SDA)
      [ ] I2C is enabled on the Pi: run `sudo raspi-config` → Interface Options → I2C
      [ ] I2C address 0x5f is visible: run `i2cdetect -y 1` and confirm 0x5f appears
      [ ] Pi and Mac are on the same WiFi network
      [ ] Pi's IP is known: run `hostname -I` on the Pi

NOTE:
    Only one mode runs at a time. main.py decides which mode to start.
    In the future, an autonomous.py mode will be added here that uses
    the camera and AI instead of WebSocket input.
"""
