"""
TUTORIAL: src/core/config.py
=============================
PURPOSE:
    Load config/hardware.yaml once at startup and expose its values to the
    rest of the system. Every other module imports from here instead of
    reading the YAML file themselves.

STEP 1 — Install PyYAML (on the Pi)
    pip install pyyaml

STEP 2 — Find the repo root path
    This file lives at src/core/config.py, so the repo root is two levels up.
    Use pathlib to build the path reliably regardless of where you run from.

    Example:
        import pathlib
        _ROOT = pathlib.Path(__file__).parent.parent.parent

STEP 3 — Load the YAML file
    Open config/hardware.yaml and parse it into a Python dictionary.

    Example:
        import yaml
        with open(_ROOT / "config" / "hardware.yaml") as f:
            _cfg = yaml.safe_load(f)

STEP 4 — Expose sub-sections as module-level variables
    Pull out each section so callers can do clean imports:
        from src.core.config import MOTOR_CFG, SERVO_CFG, WS_CFG

    Example:
        PCA_CFG   = _cfg["pca9685"]   # i2c_address, frequency
        MOTOR_CFG = _cfg["motors"]    # rear motor channels
        SERVO_CFG = _cfg["servos"]    # servo0/1/2 channels and angle limits
        WS_CFG    = _cfg["websocket"] # host and port

STEP 5 — No logic here
    This file only loads and exposes data. No classes, no functions.
    Other modules decide what to do with the values.
"""
