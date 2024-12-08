FROM ubuntu:20.04

# Install basic tools
RUN apt-get update && apt-get install -y \
    python3 python3-pip python3-dev build-essential cmake git libusb-1.0-0-dev \
    --fix-missing && rm -rf /var/lib/apt/lists/*

# Install RealSense SDK
RUN apt-get update && apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev

# Install Python bindings for RealSense
RUN pip3 install pyrealsense2 numpy opencv-python

# Set working directory
WORKDIR /app

# Copy local project files to the container
COPY . .

# Run the application by default (optional)
CMD ["python3", "camera_to_board.py"]
