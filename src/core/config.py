"""
PURPOSE:
    Load config/hardware.yaml once at startup and expose its values to the
    rest of the system. Every other module imports from here instead of
    reading the YAML file themselves.
"""
import pathlib
import yaml

_ROOT = pathlib.Path(__file__).parent.parent.parent
with open(_ROOT / "config" / "hardware.yaml") as f:
    _cfg = yaml.safe_load(f)

PCA_CFG    = _cfg["pca9685"]   # i2c_address, frequency
MOTOR_CFG  = _cfg["motors"]    # rear motor channels
SERVO_CFG  = _cfg["servos"]    # servo0/1/2 channels and angle limits
WS_CFG     = _cfg["websocket"] # host and port
WEBRTC_CFG = _cfg["webrtc"]    # host and port for WebRTC signaling WS