#!/bin/bash
set -e

echo "Updating system..."
sudo apt update && sudo apt upgrade -y

echo "Installing system dependencies..."
sudo apt install -y \
    python3-pip \
    python3-venv \
    python3-dev \
    ffmpeg \
    libavformat-dev \
    libavcodec-dev \
    libavdevice-dev \
    libavutil-dev \
    libavfilter-dev \
    libswscale-dev \
    libswresample-dev \
    libopus-dev \
    libvpx-dev \
    pkg-config \
    libcamera-dev \
    python3-libcamera \
    python3-picamera2 \
    libcamera-apps \
    python3-lgpio \
    python3-opencv \
    python3-numpy

echo "Creating virtual environment (with system packages)..."
python3 -m venv .venv --system-site-packages

echo "Activating venv..."
source .venv/bin/activate

echo "Upgrading pip..."
pip install --upgrade pip

echo "Installing Python dependencies..."
pip install -r requirements.txt

echo "Creating model directory..."
mkdir -p src/components/ai/models

echo "Setup complete ✅"
echo ""
echo "Next steps — copy the ONNX models to the Pi. Both are gitignored, so they are"
echo "not in the clone. Run these on your Mac:"
echo ""
echo "  1. YOLOv8n — required for autonomous mode:"
echo "     pip install ultralytics onnx"
echo "     python3 -c \"from ultralytics import YOLO; YOLO('yolov8n.pt').export(format='onnx', imgsz=320)\""
echo "     scp yolov8n.onnx pi@<pi-ip>:~/robo-pi/src/components/ai/models/yolov8n_320.onnx"
echo ""
echo "  2. YuNet — face detector for facial tracking mode (~230 KB). Without it the"
echo "     mode falls back to Haar cascades and loses the face whenever your head"
echo "     turns or tilts:"
echo "     curl -L -o face_detection_yunet.onnx \\"
echo "       https://github.com/opencv/opencv_zoo/raw/main/models/face_detection_yunet/face_detection_yunet_2023mar.onnx"
echo "     scp face_detection_yunet.onnx pi@<pi-ip>:~/robo-pi/src/components/ai/models/face_detection_yunet.onnx"
echo ""
echo "     The 2023mar model needs OpenCV >= 4.7. Check the Pi's version with:"
echo "       python3 -c 'import cv2; print(cv2.__version__)'"
echo "     If it is older, download face_detection_yunet_2022mar.onnx from the same"
echo "     opencv_zoo directory's history instead and save it under the same filename."