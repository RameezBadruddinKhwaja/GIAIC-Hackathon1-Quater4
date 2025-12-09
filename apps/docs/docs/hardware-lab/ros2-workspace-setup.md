# ROS 2 Workspace Setup Guide

## Overview

A ROS 2 workspace is a directory structure for organizing and building ROS 2 packages. This guide covers workspace setup, package creation, and best practices.

## Workspace Structure

```
~/ros2_ws/                  # Workspace root
├── src/                    # Source space (your packages)
│   ├── my_robot_description/
│   ├── my_robot_control/
│   └── my_robot_bringup/
├── build/                  # Build artifacts (auto-generated)
├── install/                # Installed packages (auto-generated)
└── log/                    # Build logs (auto-generated)
```

## Creating a Workspace

### 1. Create Directory Structure

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Initialize workspace (optional but recommended)
colcon build  # Creates build/, install/, log/

# Source workspace
source install/setup.bash

# Add to .bashrc for auto-sourcing
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 2. Create First Package (Python)

```bash
cd ~/ros2_ws/src

# Create Python package
ros2 pkg create --build-type ament_python my_robot_control \
  --dependencies rclpy std_msgs geometry_msgs

# Package structure:
# my_robot_control/
# ├── my_robot_control/
# │   └── __init__.py
# ├── package.xml
# ├── setup.py
# ├── setup.cfg
# ├── resource/
# └── test/
```

**Add a Node**:

```bash
cd ~/ros2_ws/src/my_robot_control/my_robot_control

# Create node
cat > velocity_controller.py << 'EOF'
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Velocity controller started')

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.1
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

chmod +x velocity_controller.py
```

**Update setup.py**:

```python
# Edit ~/ros2_ws/src/my_robot_control/setup.py

entry_points={
    'console_scripts': [
        'velocity_controller = my_robot_control.velocity_controller:main',
    ],
},
```

### 3. Create C++ Package

```bash
cd ~/ros2_ws/src

# Create C++ package
ros2 pkg create --build-type ament_cmake my_robot_cpp \
  --dependencies rclcpp std_msgs geometry_msgs

# Package structure:
# my_robot_cpp/
# ├── include/
# │   └── my_robot_cpp/
# ├── src/
# ├── CMakeLists.txt
# └── package.xml
```

**Add a Node**:

```bash
cd ~/ros2_ws/src/my_robot_cpp/src

cat > velocity_controller.cpp << 'EOF'
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class VelocityController : public rclcpp::Node
{
public:
    VelocityController() : Node("velocity_controller")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&VelocityController::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Velocity controller started");
    }

private:
    void timer_callback()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 0.5;
        msg.angular.z = 0.1;
        publisher_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityController>());
    rclcpp::shutdown();
    return 0;
}
EOF
```

**Update CMakeLists.txt**:

```cmake
# Add to ~/ros2_ws/src/my_robot_cpp/CMakeLists.txt

add_executable(velocity_controller src/velocity_controller.cpp)
ament_target_dependencies(velocity_controller rclcpp geometry_msgs)

install(TARGETS
  velocity_controller
  DESTINATION lib/${PROJECT_NAME}
)
```

## Building the Workspace

### Build All Packages

```bash
cd ~/ros2_ws
colcon build

# Build with parallel jobs
colcon build --parallel-workers 8

# Build specific package
colcon build --packages-select my_robot_control

# Build with symlink install (Python packages)
colcon build --symlink-install
```

### Build Options

```bash
# Clean build (delete build/ and install/)
rm -rf build/ install/ log/
colcon build

# Build with verbose output
colcon build --event-handlers console_direct+

# Build in Release mode (C++ optimization)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Running Nodes

### Source Workspace

```bash
# Must source workspace before running
source ~/ros2_ws/install/setup.bash

# Verify package is found
ros2 pkg list | grep my_robot

# Run Python node
ros2 run my_robot_control velocity_controller

# Run C++ node
ros2 run my_robot_cpp velocity_controller
```

### Launch Files

Create `~/ros2_ws/src/my_robot_bringup/launch/robot.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_control',
            executable='velocity_controller',
            name='velocity_controller',
            output='screen',
            parameters=[
                {'linear_speed': 1.0},
                {'angular_speed': 0.5}
            ]
        ),
        Node(
            package='my_robot_description',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen'
        )
    ])
```

```bash
# Run launch file
ros2 launch my_robot_bringup robot.launch.py
```

## Package Dependencies

### Add Dependencies

**Python (package.xml)**:

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>nav_msgs</exec_depend>
```

**C++ (package.xml + CMakeLists.txt)**:

```xml
<!-- package.xml -->
<depend>rclcpp</depend>
<depend>geometry_msgs</depend>
```

```cmake
# CMakeLists.txt
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

ament_target_dependencies(my_node rclcpp geometry_msgs)
```

### Install Dependencies

```bash
# Install all workspace dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Install specific package dependencies
rosdep install --from-paths src/my_robot_control --ignore-src -r -y
```

## Multi-Package Workspace

### Organize by Category

```
~/ros2_ws/src/
├── my_robot_description/   # URDF, meshes
├── my_robot_control/        # Controllers
├── my_robot_bringup/        # Launch files
├── my_robot_perception/     # Vision nodes
├── my_robot_navigation/     # Nav2 config
└── my_robot_interfaces/     # Custom messages
```

### Custom Messages

Create `~/ros2_ws/src/my_robot_interfaces/`:

```bash
ros2 pkg create --build-type ament_cmake my_robot_interfaces

mkdir -p ~/ros2_ws/src/my_robot_interfaces/msg
mkdir -p ~/ros2_ws/src/my_robot_interfaces/srv
mkdir -p ~/ros2_ws/src/my_robot_interfaces/action
```

**msg/SensorData.msg**:
```
std_msgs/Header header
float64 temperature
float64 humidity
string sensor_id
```

**Update CMakeLists.txt**:
```cmake
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorData.msg"
  DEPENDENCIES std_msgs
)
```

**Update package.xml**:
```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

## Workspace Overlays

### Create Overlay

```bash
# Base workspace
source /opt/ros/humble/setup.bash

# Create overlay
mkdir -p ~/overlay_ws/src
cd ~/overlay_ws/src

# Clone external packages
git clone https://github.com/ros-planning/navigation2.git -b humble

cd ~/overlay_ws
colcon build

# Source overlay (sources base automatically)
source ~/overlay_ws/install/setup.bash
```

### Chain Multiple Workspaces

```bash
# Base ROS 2
source /opt/ros/humble/setup.bash

# Workspace 1 (shared packages)
source ~/shared_ws/install/setup.bash

# Workspace 2 (robot-specific)
source ~/robot_ws/install/setup.bash

# Verify precedence
ros2 pkg prefix my_package  # Shows which workspace provides package
```

## Version Control (Git)

### Initialize Git

```bash
cd ~/ros2_ws/src/my_robot_control

git init
git add .
git commit -m "Initial commit"

# Create .gitignore
cat > .gitignore << EOF
build/
install/
log/
__pycache__/
*.pyc
.vscode/
EOF

git add .gitignore
git commit -m "Add .gitignore"
```

### Workspace .gitignore

```bash
# Root workspace .gitignore
cd ~/ros2_ws

cat > .gitignore << EOF
# Build artifacts
build/
install/
log/

# IDE
.vscode/
.idea/
*.swp

# Python
__pycache__/
*.pyc

# Backup files
*~
EOF
```

## Testing

### Add Unit Tests (Python)

```python
# ~/ros2_ws/src/my_robot_control/test/test_velocity_controller.py

import unittest
from my_robot_control.velocity_controller import VelocityController

class TestVelocityController(unittest.TestCase):
    def test_initialization(self):
        # Test node creation
        pass

if __name__ == '__main__':
    unittest.main()
```

### Run Tests

```bash
cd ~/ros2_ws
colcon test

# View test results
colcon test-result --all
colcon test-result --verbose
```

## Best Practices

### 1. Workspace Organization

- ✅ One workspace per robot/project
- ✅ Separate description, control, perception packages
- ✅ Use overlays for experimental features

### 2. Build Management

- ✅ Use `--symlink-install` for Python packages
- ✅ Build in Release mode for production
- ✅ Clean rebuild when dependencies change

### 3. Dependency Management

- ✅ Declare all dependencies in package.xml
- ✅ Use rosdep for automatic installation
- ✅ Pin versions for critical dependencies

### 4. Version Control

- ✅ Git ignore build/, install/, log/
- ✅ Commit package.xml, CMakeLists.txt, setup.py
- ✅ Use submodules for external packages

## Troubleshooting

### Package Not Found

```bash
# Rebuild workspace
cd ~/ros2_ws
colcon build

# Source workspace
source install/setup.bash

# Verify package
ros2 pkg prefix my_robot_control
```

### Build Failures

```bash
# Clean build
rm -rf build/ install/ log/
colcon build --event-handlers console_direct+

# Check dependencies
rosdep check --from-paths src --ignore-src
```

### Import Errors (Python)

```bash
# Ensure __init__.py exists
touch ~/ros2_ws/src/my_robot_control/my_robot_control/__init__.py

# Rebuild with symlink
colcon build --symlink-install --packages-select my_robot_control
```

## Key Takeaways

- ✅ Use `colcon build` to build all packages
- ✅ Source workspace with `source install/setup.bash`
- ✅ Organize packages by functionality
- ✅ Use `rosdep` for dependency management
- ✅ Test packages with `colcon test`
- ✅ Version control with Git (.gitignore build artifacts)

## Resources

- [ROS 2 Workspace Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
- [Colcon Documentation](https://colcon.readthedocs.io/)
- [Package Creation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
