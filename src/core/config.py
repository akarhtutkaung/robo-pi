"""
PURPOSE:
    Load config/hardware.yaml and config/modes.yaml once at startup and expose
    their values to the rest of the system. Every other module imports from here
    instead of reading the YAML files themselves.
"""
import pathlib
import yaml

_ROOT = pathlib.Path(__file__).parent.parent.parent

with open(_ROOT / "config" / "hardware.yaml") as f:
    _hw = yaml.safe_load(f)

with open(_ROOT / "config" / "modes.yaml") as f:
    _modes = yaml.safe_load(f)

PCA_CFG               = _hw["pca9685"]            # i2c_address, frequency
MOTOR_CFG             = _hw["motors"]             # rear motor channels
SERVO_CFG             = _hw["servos"]             # servo0/1/2 channels and angle limits
WS_CFG                = _hw["websocket"]          # host and port
WEBRTC_CFG            = _hw["webrtc"]             # host and port for WebRTC signaling WS
ULTRASONIC_CFG        = _hw["ultrasonic"]         # trigger and echo pins
CAMERA_CFG            = _hw["cameras"]            # front/back index and resolution
DEBUG_STREAM_CFG      = _hw["debug_stream"]       # enabled, port, fps
OBSTACLE_AVOIDANCE_CFG = _hw["obstacle_avoidance"] # robot_width_cm, clearance_buffer_cm, focal_length_px, camera_hfov_deg

AUTONOMOUS_CFG = _modes["autonomous"]  # speed constants for autonomous mode