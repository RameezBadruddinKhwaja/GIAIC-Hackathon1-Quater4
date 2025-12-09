# Week 6: Gazebo Simulation - The Digital Twin

## Learning Objectives

By the end of this week, you will be able to:
- ✅ Understand Gazebo Classic vs Gazebo (formerly Ignition)
- ✅ Create custom Gazebo worlds with models and plugins
- ✅ Simulate sensors (lidar, cameras, IMU) in Gazebo
- ✅ Integrate Gazebo with ROS 2 using gazebo_ros_pkgs
- ✅ Test robot behaviors in simulation before hardware deployment

## Introduction to Gazebo

Gazebo is the industry-standard robot simulator providing physics simulation, sensor emulation, and 3D visualization.

### Gazebo Classic vs Gazebo (Ignition/Harmonic)

| Feature | Gazebo Classic | Gazebo (Harmonic) |
|---------|----------------|-------------------|
| Physics Engine | ODE, Bullet, DART | DART, Bullet |
| Rendering | OGRE 1.x | OGRE 2.x (PBR) |
| File Format | SDF | SDF (improved) |
| ROS 2 Support | gazebo_ros_pkgs | ros_gz |
| Performance | Good | Better (multi-threading) |

**Recommendation**: Use **Gazebo Harmonic** for new projects.

## Installation

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

```bash
# Gazebo Harmonic
sudo apt install ros-humble-ros-gz

# Gazebo Classic (if needed for legacy projects)
sudo apt install ros-humble-gazebo-ros-pkgs
```

## Creating a Custom World

**my_world.sdf**:
```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="my_world">
    <!-- Physics -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Sun -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Obstacle -->
    <model name="box">
      <pose>2 2 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>1.67</ixx>
            <iyy>1.67</iyy>
            <izz>1.67</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

## Sensor Simulation

### Lidar Sensor

Add to robot URDF/SDF:
```xml
<gazebo reference="lidar_link">
  <sensor name="lidar" type="gpu_lidar">
    <pose>0 0 0.1 0 0 0</pose>
    <topic>scan</topic>
    <update_rate>10</update_rate>
    <lidar>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1.0</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.12</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </lidar>
    <alwaysOn>true</alwaysOn>
    <visualize>true</visualize>
  </sensor>
</gazebo>
```

### Camera Sensor

```xml
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <update_rate>30</update_rate>
    <topic>camera/image_raw</topic>
  </sensor>
</gazebo>
```

## Gazebo ROS 2 Integration

<Tabs>
  <TabItem value="launch" label="Launch File" default>

**gazebo.launch.py**:
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Gazebo world file
    world_file = os.path.join(
        get_package_share_directory('my_robot_simulation'),
        'worlds',
        'my_world.sdf'
    )

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_robot'],
        output='screen'
    )

    # Bridge ROS 2 ↔ Gazebo topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        bridge
    ])
```

  </TabItem>
</Tabs>

## Differential Drive Plugin

```xml
<gazebo>
  <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
    <left_joint>wheel_left_joint</left_joint>
    <right_joint>wheel_right_joint</right_joint>
    <wheel_separation>0.4</wheel_separation>
    <wheel_radius>0.1</wheel_radius>
    <max_linear_acceleration>1.0</max_linear_acceleration>
    <max_angular_acceleration>2.0</max_angular_acceleration>
    <odom_topic>odom</odom_topic>
    <tf_topic>tf</tf_topic>
  </plugin>
</gazebo>
```

## Hardware Considerations

<Tabs>
  <TabItem value="rtx4090" label="RTX 4090 (Simulation)" default>

**Advantages:**
- Real-time physics at high frequencies
- GPU-accelerated ray tracing for lidar
- Multiple robot instances (10+)

**Configuration:**
```xml
<physics name="fast" type="ignored">
  <max_step_size>0.001</max_step_size>  <!-- 1ms timestep -->
  <real_time_factor>1.0</real_time_factor>
</physics>

<!-- GPU lidar -->
<sensor name="lidar" type="gpu_lidar">
  <update_rate>30</update_rate>  <!-- High frequency -->
</sensor>
```

  </TabItem>
  <TabItem value="jetson" label="Jetson Orin Nano (Edge)">

**Use Case**: Simulation on Jetson for edge testing

**Optimization:**
```xml
<physics name="realtime" type="ignored">
  <max_step_size>0.01</max_step_size>  <!-- 10ms timestep -->
  <real_time_factor>0.5</real_time_factor>  <!-- Half speed -->
</physics>

<!-- CPU lidar (slower but works) -->
<sensor name="lidar" type="lidar">
  <update_rate>5</update_rate>  <!-- Reduced frequency -->
</sensor>
```

  </TabItem>
</Tabs>

## Key Takeaways

- ✅ Gazebo provides physics-based robot simulation
- ✅ SDF format defines worlds, models, and sensors
- ✅ Gazebo Harmonic offers better performance than Classic
- ✅ ros_gz bridge connects Gazebo and ROS 2 topics
- ✅ Sensor simulation enables algorithm testing before hardware

## Resources

- [Gazebo Documentation](https://gazebosim.org/docs)
- [ros_gz Repository](https://github.com/gazebosim/ros_gz)
- [SDF Specification](http://sdformat.org/)

---

**🎓 Quiz**: [Week 6 Quiz](./quiz.md)
**💻 Challenge**: [Custom World Challenge](./challenge.md)
