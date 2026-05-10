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
mkdir -p src/ai/models

echo "Setup complete ✅"
echo ""
echo "Next step — copy the YOLOv8n ONNX model to the Pi (required for autonomous mode):"
echo "  On your Mac:"
echo "    pip install ultralytics onnx"
echo "    python3 -c \"from ultralytics import YOLO; YOLO('yolov8n.pt').export(format='onnx', imgsz=320)\""
echo "    scp yolov8n.onnx pi@<pi-ip>:~/robo-pi/src/ai/models/yolov8n_320.onnx"