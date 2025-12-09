# Troubleshooting Guide - Common Issues & Solutions

## ROS 2 Common Issues

### 1. Package Not Found

**Symptom**:
```bash
ros2 run my_package my_node
# Package 'my_package' not found
```

**Solutions**:

```bash
# Solution 1: Source workspace
source ~/ros2_ws/install/setup.bash

# Solution 2: Rebuild package
cd ~/ros2_ws
colcon build --packages-select my_package
source install/setup.bash

# Solution 3: Check package exists
ros2 pkg list | grep my_package

# Solution 4: Verify package.xml
cat ~/ros2_ws/src/my_package/package.xml  # Must exist
```

### 2. Nodes Can't Communicate

**Symptom**:
```bash
ros2 topic list  # Topics missing
ros2 node list   # Nodes on different machines not visible
```

**Solutions**:

```bash
# Solution 1: Check ROS_DOMAIN_ID (must match)
echo $ROS_DOMAIN_ID  # Should be same across all devices

# Set domain ID (0-101)
export ROS_DOMAIN_ID=42
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc

# Solution 2: Check network connectivity
ping other_device_ip

# Solution 3: Check firewall
sudo ufw status
sudo ufw allow 7400:7500/tcp  # DDS discovery
sudo ufw allow 7400:7500/udp

# Solution 4: Use localhost for single-machine
export ROS_LOCALHOST_ONLY=1
```

### 3. Build Failures

**Symptom**:
```bash
colcon build
# --- stderr: my_package
# error: package 'rclpy' not found
```

**Solutions**:

```bash
# Solution 1: Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Solution 2: Check package.xml dependencies
cat src/my_package/package.xml
# Ensure <depend>rclpy</depend> exists

# Solution 3: Clean build
rm -rf build/ install/ log/
colcon build

# Solution 4: Install missing ROS 2 packages
sudo apt update
sudo apt install ros-humble-rclpy
```

### 4. Import Error (Python)

**Symptom**:
```python
import my_package.my_module
# ModuleNotFoundError: No module named 'my_package'
```

**Solutions**:

```bash
# Solution 1: Create __init__.py
touch ~/ros2_ws/src/my_package/my_package/__init__.py

# Solution 2: Rebuild with symlink
colcon build --symlink-install --packages-select my_package
source install/setup.bash

# Solution 3: Verify setup.py entry points
cat src/my_package/setup.py
# Ensure 'my_module' is in console_scripts

# Solution 4: Check PYTHONPATH
echo $PYTHONPATH  # Should include install path
```

## Gazebo/Simulation Issues

### 5. Gazebo Crashes on Launch

**Symptom**:
```bash
ros2 launch gazebo_ros gazebo.launch.py
# Segmentation fault (core dumped)
```

**Solutions**:

```bash
# Solution 1: Update graphics drivers
ubuntu-drivers devices
sudo ubuntu-drivers autoinstall
sudo reboot

# Solution 2: Use CPU rendering (slow but stable)
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch gazebo_ros gazebo.launch.py

# Solution 3: Check NVIDIA driver
nvidia-smi  # Should show driver version

# Solution 4: Reinstall Gazebo
sudo apt remove --purge ros-humble-gazebo-*
sudo apt install ros-humble-gazebo-ros-pkgs
```

### 6. Slow Simulation (Low FPS)

**Symptom**:
```
Real-time factor: 0.2 (should be ~1.0)
```

**Solutions**:

```bash
# Solution 1: Reduce physics iterations
# Edit world file, change max_step_size
<physics>
  <max_step_size>0.01</max_step_size>  <!-- Increase from 0.001 -->
</physics>

# Solution 2: Simplify collision meshes
<collision>
  <geometry>
    <box size="0.1 0.1 0.1"/>  <!-- Use box instead of mesh -->
  </geometry>
</collision>

# Solution 3: Reduce number of models
# Remove unnecessary objects from world

# Solution 4: Enable GPU acceleration
export LIBGL_ALWAYS_INDIRECT=0
```

### 7. URDF/Xacro Errors

**Symptom**:
```bash
xacro my_robot.urdf.xacro
# error: undefined macro 'my_macro'
```

**Solutions**:

```bash
# Solution 1: Check macro definition
<xacro:macro name="my_macro" params="prefix">
  <!-- Ensure macro is defined before use -->
</xacro:macro>

# Solution 2: Include xacro namespace
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">

# Solution 3: Validate URDF
check_urdf my_robot.urdf

# Solution 4: Debug xacro expansion
xacro my_robot.urdf.xacro --check-order
```

## Navigation/SLAM Issues

### 8. Nav2 Planner Fails

**Symptom**:
```
[planner_server]: Goal is occupied or out of map bounds
```

**Solutions**:

```bash
# Solution 1: Check map bounds
ros2 topic echo /map --once
# Verify goal is within map dimensions

# Solution 2: Adjust costmap inflation
# In nav2_params.yaml
inflation_layer:
  inflation_radius: 0.35  # Reduce if too aggressive

# Solution 3: Reload map
ros2 service call /global_costmap/clear_entirely_global_costmap std_srvs/srv/Empty

# Solution 4: Visualize costmap in RViz
# Add -> CostMap2D -> Topic: /global_costmap/costmap
```

### 9. Robot Not Localizing

**Symptom**:
```
AMCL particle cloud not converging
```

**Solutions**:

```bash
# Solution 1: Increase particle count
# In amcl_params.yaml
amcl:
  ros__parameters:
    max_particles: 5000  # Increase from 2000

# Solution 2: Provide initial pose
ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped '{...}'

# Solution 3: Check laser scan
ros2 topic echo /scan --once
# Verify scan data is reasonable

# Solution 4: Adjust AMCL parameters
min_particles: 500
update_min_d: 0.1  # Update more frequently
```

### 10. SLAM Map Quality Poor

**Symptom**:
- Walls appear duplicated
- Map drifts over time

**Solutions**:

```bash
# Solution 1: Reduce robot speed during mapping
ros2 param set /slam_toolbox minimum_travel_distance 0.2

# Solution 2: Improve odometry
# Check wheel encoder resolution
# Calibrate wheel diameter and base width

# Solution 3: Add loop closure detection
# In slam_params.yaml
mode: localization  # After initial map creation

# Solution 4: Use higher quality lidar
# Increase scan frequency
# Ensure lidar is level and stable
```

## Hardware-Specific Issues

### 11. RTX GPU Not Detected

**Symptom**:
```bash
nvidia-smi
# NVIDIA-SMI has failed because it couldn't communicate with the NVIDIA driver
```

**Solutions**:

```bash
# Solution 1: Install/Reinstall driver
ubuntu-drivers devices
sudo ubuntu-drivers autoinstall
sudo reboot

# Solution 2: Check secure boot
mokutil --sb-state
# If enabled, disable in BIOS or enroll MOK

# Solution 3: Remove conflicting drivers
sudo apt purge nvidia-*
sudo apt autoremove
sudo ubuntu-drivers autoinstall

# Solution 4: Manual driver installation
wget https://us.download.nvidia.com/XFree86/Linux-x86_64/545.23.06/NVIDIA-Linux-x86_64-545.23.06.run
sudo bash NVIDIA-Linux-x86_64-545.23.06.run
```

### 12. Jetson Boot Issues

**Symptom**:
Jetson won't boot, stuck at NVIDIA logo

**Solutions**:

```bash
# Solution 1: Re-flash with SDK Manager
# Boot into recovery mode:
# 1. Power off Jetson
# 2. Hold RECOVERY button
# 3. Press POWER button
# 4. Release RECOVERY after 2 seconds

# Solution 2: Check power supply
# Jetson Orin Nano requires 5V/4A minimum
# Use official power adapter

# Solution 3: Try different microSD card
# Use high-quality UHS-I card (SanDisk Extreme)

# Solution 4: Boot from NVMe
# Flash NVMe with JetPack instead of microSD
```

### 13. Jetson Overheating

**Symptom**:
```bash
jtop  # Shows temperature > 80°C
```

**Solutions**:

```bash
# Solution 1: Add heatsink and fan
# Recommended: Noctua NF-A4x10 5V PWM

# Solution 2: Reduce power mode
sudo nvpmodel -m 0  # 15W mode

# Solution 3: Limit CPU frequency
sudo jetson_clocks --show
sudo jetson_clocks  # Disable to reduce heat

# Solution 4: Improve airflow
# Ensure Jetson has clearance around it
# Add case with ventilation holes
```

## Camera/Sensor Issues

### 14. Camera No Image

**Symptom**:
```bash
ros2 topic echo /camera/image_raw
# No messages received
```

**Solutions**:

```bash
# Solution 1: Check camera connection
ls /dev/video*  # Should show /dev/video0

# Solution 2: Test with v4l2
v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext

# Solution 3: Check permissions
sudo usermod -a -G video $USER
newgrp video

# Solution 4: Launch camera node with correct params
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p image_size:="[640,480]" \
  -p camera_frame_id:=camera_link
```

### 15. Lidar No Data

**Symptom**:
```bash
ros2 topic echo /scan
# No messages received
```

**Solutions**:

```bash
# Solution 1: Check USB connection
lsusb  # Should show lidar device

# Solution 2: Grant permissions
sudo chmod 666 /dev/ttyUSB0

# Solution 3: Install lidar driver
sudo apt install ros-humble-rplidar-ros

# Solution 4: Check baud rate
# RPLidar A1: 115200
# RPLidar A2/A3: 256000
ros2 run rplidar_ros rplidar_node --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p serial_baudrate:=115200
```

## Performance Issues

### 16. High CPU Usage

**Symptom**:
```bash
htop  # ROS 2 nodes using 100% CPU
```

**Solutions**:

```bash
# Solution 1: Reduce topic frequency
ros2 param set /camera_node publish_frequency 10.0  # Reduce from 30

# Solution 2: Use QoS BEST_EFFORT
# In code:
qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)

# Solution 3: Disable debug logging
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] {message}"
export RCUTILS_COLORIZED_OUTPUT=0

# Solution 4: Optimize callbacks
# Avoid heavy computation in callbacks
# Use separate threads for processing
```

### 17. Out of Memory (OOM)

**Symptom**:
```bash
dmesg | grep -i "out of memory"
# Process killed by OOM killer
```

**Solutions**:

```bash
# Solution 1: Add swap (Ubuntu)
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Solution 2: Reduce ROS 2 buffer sizes
# In code:
publisher = self.create_publisher(String, 'topic', 1)  # Reduce from 10

# Solution 3: Disable GUI on Jetson
sudo systemctl set-default multi-user.target

# Solution 4: Close unnecessary applications
pkill firefox
pkill thunderbird
```

## Network/Communication Issues

### 18. High Latency

**Symptom**:
```bash
ros2 topic hz /camera/image_raw
# average rate: 5.000 (expected 30.000)
```

**Solutions**:

```bash
# Solution 1: Use wired Ethernet
# WiFi adds 10-50ms latency

# Solution 2: Increase ROS_DOMAIN_ID space
export ROS_DOMAIN_ID=42  # Avoid default 0

# Solution 3: Use DDS tuning
# Create cyclonedds.xml with optimized settings

# Solution 4: Compress large messages
# Use image_transport for image compression
```

### 19. Package Version Conflicts

**Symptom**:
```bash
colcon build
# Dependency 'package_a' requires 'package_b' version >= 2.0, found 1.5
```

**Solutions**:

```bash
# Solution 1: Update packages
sudo apt update
sudo apt install ros-humble-package-b

# Solution 2: Use rosdep
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Solution 3: Create workspace overlay
# Build newer version in separate workspace
mkdir -p ~/overlay_ws/src
# Clone newer package
colcon build
source ~/overlay_ws/install/setup.bash

# Solution 4: Pin version in package.xml
<depend version_gte="2.0">package_b</depend>
```

## Quick Reference Commands

### Diagnostic Commands

```bash
# ROS 2 nodes
ros2 node list
ros2 node info /my_node

# Topics
ros2 topic list
ros2 topic info /my_topic
ros2 topic hz /my_topic
ros2 topic bw /my_topic

# Services
ros2 service list
ros2 service type /my_service

# Parameters
ros2 param list
ros2 param get /my_node my_param

# System
nvidia-smi  # GPU stats
htop  # CPU/RAM
jtop  # Jetson stats
dmesg  # Kernel messages
```

### Reset/Restart Commands

```bash
# Restart ROS 2 daemon
ros2 daemon stop
ros2 daemon start

# Clear ROS 2 logs
rm -rf ~/.ros/log/*

# Restart network
sudo systemctl restart networking

# Reboot system
sudo reboot
```

## Getting Help

### Resources

- **ROS 2 Documentation**: https://docs.ros.org/en/humble/
- **ROS Answers**: https://answers.ros.org/
- **ROS Discourse**: https://discourse.ros.org/
- **GitHub Issues**: Check package repositories

### Reporting Bugs

When reporting issues, include:

1. **ROS 2 version**: `ros2 --version`
2. **OS version**: `lsb_release -a`
3. **Hardware**: CPU, GPU, RAM
4. **Error messages**: Full terminal output
5. **Steps to reproduce**: Minimal example
6. **Launch files**: Relevant configuration

## Key Takeaways

- ✅ Always source workspace after building
- ✅ Check ROS_DOMAIN_ID for multi-device communication
- ✅ Use `rosdep` for dependency management
- ✅ Monitor system resources (CPU, GPU, RAM)
- ✅ Test with minimal examples before full system
- ✅ Keep ROS 2 and drivers updated
- ✅ Check logs and error messages carefully

---

**Need More Help?** Join the ROS 2 community on Discord, Discourse, or ROS Answers!
