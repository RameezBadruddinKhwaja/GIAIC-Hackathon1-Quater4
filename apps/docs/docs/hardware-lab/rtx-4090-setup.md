# RTX 4090 Setup Guide - Simulation Workstation

## Overview

The NVIDIA RTX 4090 is the recommended GPU for robotics simulation, offering:
- **24GB GDDR6X VRAM** for large-scale simulations
- **16,384 CUDA cores** for parallel physics computation
- **512 Tensor Cores (4th gen)** for AI inference
- **RT Cores (3rd gen)** for ray-traced rendering

## System Requirements

### Minimum Specifications
- **CPU**: Intel i7-12700K or AMD Ryzen 7 5800X (8+ cores)
- **RAM**: 32GB DDR4/DDR5
- **Storage**: 1TB NVMe SSD
- **PSU**: 850W 80+ Gold (RTX 4090 draws 450W)
- **Cooling**: High-airflow case + CPU cooler

### Recommended Specifications
- **CPU**: Intel i9-13900K or AMD Ryzen 9 7950X (16+ cores)
- **RAM**: 64GB DDR5
- **Storage**: 2TB NVMe SSD (Gen 4)
- **PSU**: 1000W 80+ Platinum
- **Cooling**: AIO liquid cooler + 6+ case fans

## Ubuntu 22.04 LTS Installation

### 1. Create Bootable USB

```bash
# Download Ubuntu 22.04 LTS
wget https://releases.ubuntu.com/22.04/ubuntu-22.04.3-desktop-amd64.iso

# Create bootable USB (on Linux)
sudo dd if=ubuntu-22.04.3-desktop-amd64.iso of=/dev/sdX bs=4M status=progress && sync

# On Windows, use Rufus: https://rufus.ie/
```

### 2. Install Ubuntu

1. Boot from USB
2. Select "Install Ubuntu"
3. Choose "Normal installation"
4. Select "Erase disk and install Ubuntu" (or dual-boot)
5. Create user account
6. Complete installation and reboot

### 3. Post-Installation Setup

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install build essentials
sudo apt install -y build-essential git curl wget vim \
  net-tools htop neofetch

# Install development tools
sudo apt install -y cmake pkg-config libssl-dev
```

## NVIDIA Driver Installation

### Method 1: Ubuntu Driver Repository (Recommended)

```bash
# Add graphics drivers PPA
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update

# List available drivers
ubuntu-drivers devices

# Install recommended driver (should be 545+)
sudo ubuntu-drivers autoinstall

# Reboot
sudo reboot

# Verify installation
nvidia-smi
# Expected output:
# +-----------------------------------------------------------------------------+
# | NVIDIA-SMI 545.23.06    Driver Version: 545.23.06    CUDA Version: 12.3     |
# |-------------------------------+----------------------+----------------------+
# | GPU  Name        TCC/WDDM | Bus-Id        Disp.A | Volatile Uncorr. ECC |
# | Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
# |===============================+======================+======================|
# |   0  NVIDIA GeForce ...  Off  | 00000000:01:00.0  On |                  Off |
# |  0%   35C    P8    20W / 450W |    723MiB / 24564MiB |      0%      Default |
# +-------------------------------+----------------------+----------------------+
```

### Method 2: NVIDIA Official Installer

```bash
# Download driver
wget https://us.download.nvidia.com/XFree86/Linux-x86_64/545.23.06/NVIDIA-Linux-x86_64-545.23.06.run

# Stop display manager
sudo systemctl stop gdm3

# Install driver
sudo bash NVIDIA-Linux-x86_64-545.23.06.run

# Reboot
sudo reboot
```

## CUDA Toolkit Installation

```bash
# Install CUDA 12.3
wget https://developer.download.nvidia.com/compute/cuda/12.3.0/local_installers/cuda_12.3.0_545.23.06_linux.run

sudo sh cuda_12.3.0_545.23.06_linux.run

# Add to PATH (~/.bashrc)
echo 'export PATH=/usr/local/cuda-12.3/bin${PATH:+:${PATH}}' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-12.3/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}' >> ~/.bashrc

source ~/.bashrc

# Verify CUDA
nvcc --version
# Expected: Cuda compilation tools, release 12.3
```

## cuDNN Installation (for Deep Learning)

```bash
# Download cuDNN from NVIDIA (requires login)
# https://developer.nvidia.com/cudnn

# Extract and install
tar -xvf cudnn-linux-x86_64-8.9.5.30_cuda12-archive.tar.xz
cd cudnn-linux-x86_64-8.9.5.30_cuda12-archive

sudo cp include/cudnn*.h /usr/local/cuda/include
sudo cp lib/libcudnn* /usr/local/cuda/lib64
sudo chmod a+r /usr/local/cuda/include/cudnn*.h /usr/local/cuda/lib64/libcudnn*
```

## ROS 2 Humble Installation

```bash
# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install -y ros-humble-desktop

# Install development tools
sudo apt install -y ros-dev-tools python3-colcon-common-extensions

# Add to bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 --version
# Expected: ros2 cli version 0.18.5
```

## Isaac Sim Installation (Omniverse)

```bash
# Download Omniverse Launcher
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage

# In Omniverse Launcher:
# 1. Exchange tab
# 2. Search "Isaac Sim"
# 3. Install Isaac Sim 2023.1.1 (~10GB)

# Launch Isaac Sim
~/.local/share/ov/pkg/isaac_sim-2023.1.1/isaac-sim.sh
```

## Performance Optimization

### 1. GPU Power Management

```bash
# Set maximum performance mode
sudo nvidia-smi -pm 1  # Persistence mode
sudo nvidia-smi -pl 450  # Power limit (450W max)

# Monitor GPU stats
watch -n 1 nvidia-smi
```

### 2. CPU Performance

```bash
# Set performance governor
sudo apt install cpufrequtils
sudo cpufreq-set -r -g performance

# Verify
cpufreq-info
```

### 3. System Monitoring

```bash
# Install monitoring tools
sudo apt install -y htop nvtop iotop

# Monitor GPU
nvtop

# Monitor CPU/RAM
htop

# Monitor disk I/O
sudo iotop
```

## Benchmark Tests

### CUDA Performance

```bash
# Install CUDA samples
git clone https://github.com/NVIDIA/cuda-samples.git
cd cuda-samples/Samples/1_Utilities/deviceQuery
make

# Run device query
./deviceQuery

# Expected output:
# Device 0: "NVIDIA GeForce RTX 4090"
#   CUDA Capability Major/Minor version number:    8.9
#   Total amount of global memory:                 24564 MBytes
#   GPU Max Clock rate:                            2520 MHz
#   Memory Clock rate:                             10501 MHz
#   Memory Bus Width:                              384-bit
```

### Gazebo Performance

```bash
# Install Gazebo
sudo apt install -y ros-humble-gazebo-ros-pkgs

# Launch test world
ros2 launch gazebo_ros gazebo.launch.py

# Expected FPS: 60+ (real-time factor 1.0)
```

### Isaac Sim Performance

```bash
# Run Isaac Sim benchmark
~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh \
  standalone_examples/api/omni.isaac.benchmark/benchmark.py

# Expected results:
# - 4K rendering: 60 FPS
# - Physics (10k rigid bodies): 200+ FPS
# - Ray tracing: 30 FPS
```

## Troubleshooting

### Driver Issues

```bash
# If nvidia-smi fails, reinstall driver
sudo apt purge nvidia-*
sudo ubuntu-drivers autoinstall
sudo reboot
```

### CUDA Not Found

```bash
# Verify CUDA path
ls /usr/local/cuda-12.3

# Re-add to PATH
export PATH=/usr/local/cuda-12.3/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-12.3/lib64:$LD_LIBRARY_PATH
```

### Out of Memory (OOM)

```bash
# Check GPU memory usage
nvidia-smi

# Reduce batch size or parallel environments
# For Isaac Orbit: env_cfg.scene.num_envs = 4096 (reduce if OOM)
```

## Maintenance

### Weekly

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Clean package cache
sudo apt autoremove -y
sudo apt autoclean
```

### Monthly

```bash
# Update NVIDIA driver (if available)
ubuntu-drivers devices
sudo ubuntu-drivers autoinstall

# Clean CUDA cache
rm -rf ~/.nv/ComputeCache/*
```

## Key Takeaways

- ✅ RTX 4090 provides 10-100x speedup over CPU for robotics workloads
- ✅ Ubuntu 22.04 LTS + Driver 545+ + CUDA 12.3 is the recommended stack
- ✅ 64GB RAM enables large-scale parallel simulations
- ✅ Proper cooling is essential for sustained performance
- ✅ Monitor GPU temperature (keep below 85°C under load)

## Resources

- [NVIDIA Driver Downloads](https://www.nvidia.com/download/index.aspx)
- [CUDA Toolkit](https://developer.nvidia.com/cuda-downloads)
- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)
