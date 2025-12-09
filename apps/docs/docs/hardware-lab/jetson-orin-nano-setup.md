# Jetson Orin Nano Setup Guide - Edge Deployment

## Overview

The NVIDIA Jetson Orin Nano is a compact AI computer for edge robotics, offering:
- **1024-core NVIDIA Ampere GPU** with 32 Tensor Cores
- **6-core ARM Cortex-A78AE CPU** @ 1.5 GHz
- **8GB LPDDR5 RAM** (shared memory)
- **15W / 25W / 40W power modes**
- **$499** Developer Kit

## Hardware Setup

### Jetson Orin Nano Developer Kit Contents

1. **Jetson Orin Nano Module** (8GB)
2. **Reference Carrier Board**
3. **USB-C power supply** (5V/3A)
4. **microSD card** (64GB recommended)

### Required Accessories

- **Power Supply**: 5V/4A USB-C or barrel jack (9-19V, 30W+)
- **microSD Card**: 128GB+ UHS-I (SanDisk Extreme recommended)
- **USB Keyboard + Mouse**
- **HDMI Monitor**
- **Ethernet Cable** or WiFi adapter
- **USB Webcam** or CSI camera module

## JetPack 5.1.2 Installation

### Method 1: SD Card Image (Easiest)

```bash
# On Ubuntu host PC:
# 1. Download JetPack SD Card Image
wget https://developer.nvidia.com/downloads/embedded/l4t/r35_release_v4.1/jp512-orin-nano-sd-card-image.zip

# 2. Extract
unzip jp512-orin-nano-sd-card-image.zip

# 3. Flash to microSD (replace sdX with your card)
sudo dd if=sd-blob.img of=/dev/sdX bs=4M status=progress && sync

# 4. Insert microSD into Jetson and power on
# 5. Complete Ubuntu setup wizard
```

### Method 2: SDK Manager (Advanced)

```bash
# On Ubuntu 20.04/22.04 host:
# 1. Download SDK Manager
wget https://developer.nvidia.com/sdkmanager

# 2. Install
sudo dpkg -i sdkmanager_*.deb

# 3. Launch and follow GUI
sdkmanager

# 4. Select Jetson Orin Nano, JetPack 5.1.2
# 5. Connect Jetson via USB-C and flash
```

## Initial Configuration

### 1. First Boot Setup

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install essential tools
sudo apt install -y build-essential git curl wget vim \
  htop jtop nano python3-pip

# Enable CUDA
echo 'export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}' >> ~/.bashrc
source ~/.bashrc

# Verify CUDA
nvcc --version
# Expected: Cuda compilation tools, release 11.4
```

### 2. Power Mode Configuration

```bash
# Check current mode
sudo nvpmodel -q

# Available modes:
# - Mode 0: 15W (4 cores)
# - Mode 1: 25W (6 cores, default)
# - Mode 2: 40W (6 cores, max performance)

# Set to 15W (battery/thermal constrained)
sudo nvpmodel -m 0

# Set to 25W (balanced)
sudo nvpmodel -m 1

# Set to 40W (max performance, requires active cooling)
sudo nvpmodel -m 2

# Monitor power
sudo tegrastats
```

### 3. Enable MAX Clocks (Performance)

```bash
# Maximize GPU/CPU clocks (Mode 2 required)
sudo jetson_clocks

# Verify clocks
sudo jetson_clocks --show
```

## ROS 2 Humble Installation

```bash
# Add ROS 2 repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Base (lighter than Desktop)
sudo apt update
sudo apt install -y ros-humble-ros-base

# Install build tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update

# Add to bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify
ros2 --version
```

## Isaac ROS Installation (Docker)

```bash
# Install Docker
sudo apt install -y docker.io
sudo usermod -aG docker $USER
newgrp docker

# Clone Isaac ROS Common
cd ~
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
cd isaac_ros_common

# Build Docker image (takes ~30 minutes)
./scripts/run_dev.sh --platform jetson

# Inside container, build packages
cd /workspaces/isaac_ros-dev
colcon build --packages-up-to isaac_ros_apriltag

# Test AprilTag detection
ros2 launch isaac_ros_apriltag isaac_ros_apriltag.launch.py
```

## Camera Setup

### USB Webcam

```bash
# Install v4l2 tools
sudo apt install -y v4l-utils ros-humble-v4l2-camera

# List cameras
v4l2-ctl --list-devices

# Launch camera node
ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[640,480]"

# View image
ros2 run image_view image_view --ros-args --remap /image:=/image_raw
```

### CSI Camera (IMX219/IMX477)

```bash
# Detect CSI camera
v4l2-ctl --list-devices
# Should show /dev/video0

# Test with GStreamer
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! \
  'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1' ! \
  nvvidconv ! autovideosink

# Install CSI camera driver
sudo apt install -y ros-humble-image-transport-plugins

# Launch CSI camera node
# (Use custom package like jetson_camera or argus_camera)
```

## Performance Optimization

### 1. Swap File (Increase Virtual Memory)

```bash
# Create 8GB swap file
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Make permanent
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

# Verify
free -h
```

### 2. Reduce Desktop Overhead

```bash
# Disable GUI on boot (headless mode)
sudo systemctl set-default multi-user.target

# Re-enable if needed
sudo systemctl set-default graphical.target

# Start GUI manually
sudo systemctl start gdm3
```

### 3. Optimize TensorRT

```bash
# Install TensorRT (included in JetPack)
dpkg -l | grep TensorRT
# Should show libnvinfer8

# Convert ONNX to TensorRT FP16
/usr/src/tensorrt/bin/trtexec \
  --onnx=model.onnx \
  --saveEngine=model_fp16.engine \
  --fp16 \
  --workspace=2048  # 2GB workspace
```

## Common ROS 2 Packages for Jetson

```bash
# Navigation
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup

# SLAM
sudo apt install -y ros-humble-slam-toolbox ros-humble-cartographer-ros

# Vision
sudo apt install -y ros-humble-vision-msgs ros-humble-image-pipeline

# Control
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers

# Sensors
sudo apt install -y ros-humble-realsense2-camera ros-humble-rplidar-ros
```

## Benchmarks

### GPU Performance

```bash
# Install jetson-stats
sudo pip3 install -U jetson-stats

# Monitor GPU/CPU/Memory
jtop
```

**Expected Performance:**
- **15W Mode**: GPU ~500 MHz, 4 cores @ 1.2 GHz
- **25W Mode**: GPU ~750 MHz, 6 cores @ 1.5 GHz
- **40W Mode**: GPU ~1 GHz, 6 cores @ 2.0 GHz (requires fan)

### Isaac ROS AprilTag

| Power Mode | Resolution | FPS |
|------------|-----------|-----|
| 15W | 640x480 | 30 |
| 25W | 1280x720 | 40 |
| 40W | 1920x1080 | 60 |

### Visual SLAM

| Power Mode | Map Size | FPS | Power Draw |
|------------|----------|-----|------------|
| 15W | Small | 15 | ~12W |
| 25W | Medium | 25 | ~20W |
| 40W | Large | 30 | ~35W |

## Thermal Management

### Check Temperature

```bash
# Monitor temperature
watch -n 1 cat /sys/devices/virtual/thermal/thermal_zone*/temp

# Or use jtop
jtop
# Check "TEMP" section
```

### Add Active Cooling

**Recommended Fan:** Noctua NF-A4x10 5V PWM

```bash
# Install fan control
sudo apt install -y fancontrol

# Configure
sudo pwmconfig
# Follow wizard to set fan curve

# Enable on boot
sudo systemctl enable fancontrol
```

## Troubleshooting

### CUDA Not Found

```bash
# Verify CUDA installation
ls /usr/local/cuda

# Re-add to PATH
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
```

### Out of Memory

```bash
# Check memory usage
free -h

# Kill memory-heavy processes
sudo systemctl stop gdm3  # Stop GUI

# Increase swap (see above)
```

### Slow Performance

```bash
# Check power mode
sudo nvpmodel -q

# Enable max clocks
sudo jetson_clocks

# Monitor with jtop
jtop
```

### ROS 2 Communication Issues

```bash
# Check ROS_DOMAIN_ID (must match across devices)
echo $ROS_DOMAIN_ID

# Set domain ID (0-101)
export ROS_DOMAIN_ID=42

# Check network
ros2 topic list  # Should show topics from other nodes
```

## Deployment Best Practices

### 1. Power Budget

- **15W**: Outdoor robots (battery-powered)
- **25W**: Indoor robots (USB-C powered)
- **40W**: Stationary applications (barrel jack + fan)

### 2. Storage

- Use microSD for OS, NVMe SSD for data/models
- Enable log rotation to prevent disk fill

```bash
# Install log rotation
sudo apt install -y logrotate

# Configure ROS 2 logging
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] {message}"
export RCUTILS_LOGGING_BUFFERED_STREAM=1
```

### 3. Headless Operation

```bash
# Disable GUI
sudo systemctl set-default multi-user.target

# Auto-start ROS 2 nodes
sudo nano /etc/systemd/system/ros2-robot.service

# Add:
# [Unit]
# Description=ROS 2 Robot Service
# After=network.target
#
# [Service]
# Type=simple
# User=jetson
# ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && ros2 launch my_robot bringup.launch.py"
# Restart=always
#
# [Install]
# WantedBy=multi-user.target

sudo systemctl enable ros2-robot
sudo systemctl start ros2-robot
```

## Key Takeaways

- ✅ Jetson Orin Nano balances performance and power efficiency
- ✅ 25W mode is optimal for most robotics applications
- ✅ Active cooling enables 40W mode for maximum performance
- ✅ Isaac ROS provides GPU-accelerated perception at 30-60 FPS
- ✅ Swap file essential for memory-intensive workloads
- ✅ Headless mode recommended for production deployment

## Resources

- [Jetson Orin Nano Developer Kit](https://developer.nvidia.com/embedded/jetson-orin-nano-developer-kit)
- [JetPack SDK](https://developer.nvidia.com/embedded/jetpack)
- [Isaac ROS](https://nvidia-isaac-ros.github.io/)
- [jetson-stats](https://github.com/rbonghi/jetson_stats)
