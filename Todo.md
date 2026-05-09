# Autonomous Steering Pipeline

## Camera-Based Free Space Detection
- [x] Camera sharing: shared picamera2 capture_array() — main 640×480 YUV420 for WebRTC, lores 320×240 YUV420 for OpenCV. Camera owned by remote.py, passed to both servers and autonomous loop.
- [x] `src/perception/vision/free_space.py` — column-wise Canny edge density → (error, confidence)
  - run offline: `python3 -m src.perception.vision.free_space frame.jpg`
  - run live + save frames: `python3 -m src.perception.vision.free_space --live`
  - tune ROI_TOP/ROI_BOTTOM, CANNY_LO/HI, BLUR_K to match your floor/lighting
- [x] Fix camera servo to center-forward in autonomous mode (pan scan removed — direction now from detect())

## PID Steering
- [ ] `src/navigation/pid.py` — minimal PID class with integral windup clamp
- [ ] Add Kp, Ki, Kd, max_speed, min_speed to config/modes.yaml
- [ ] Speed ↔ steering coupling: `speed = base_speed * (1 - k * |normalized_error|)`

## Refactor autonomous.py
- [ ] Calculate if steer-forward can clear the obstacle before deciding to reverse
  - only back up when forward path would collide even after steering
- [ ] Soft collision avoidance — steer away from obstacle without stopping, then return to lane
  - replace harsh stop with proportional steer correction
- [ ] Replace K-turn logic in navigate_step with PID loop
- [ ] Keep ultrasonic is_sudden_stop() as the only emergency hard-stop trigger

## Realife movement
- [ ] Make head move like real life boring movement if nothing had been doing over threshold
- [ ] Make body move like real life boring movement if nothing had been doing over threshold

## Microphone
- [ ] Instead of controlling from pc, control the auto drive via microphone
- [ ] Speech recognization and turn the head directly to where it was calling, then mark the possible location. Then move to that location.
- [ ]