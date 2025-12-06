---
id: sensors
title: Sensors & Perception
sidebar_label: Sensors & Perception
description: Integrate LiDAR, cameras, and IMU sensors into your robot simulation
---

# Sensors & Perception: Robot Awareness

Sensors are the eyes and ears of your robot. This guide covers integrating realistic sensor models into Gazebo, including LiDAR for obstacle detection, cameras for vision, and IMU for orientation tracking.

## LiDAR Sensors

LiDAR (Light Detection and Ranging) is essential for autonomous navigation, providing 360° obstacle detection through laser rangefinding.

### Gazebo LiDAR Plugin

Add a LiDAR sensor to your URDF with the `gazebo_ros_ray_sensor` plugin:

```xml
<gazebo reference="lidar_link">
  <sensor name="lidar" type="ray">
    <always_on>true</always_on>
    <visualize>true</visualize>
    <update_rate>10.0</update_rate>

    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1.0</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.12</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </ray>

    <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**Key Parameters:**
- **samples**: Number of laser beams (720 = 0.5° resolution)
- **update_rate**: Scans per second (10 Hz is typical)
- **min/max**: Range limits (0.12m - 30m for RPLiDAR A1)
- **noise**: Gaussian noise with mean and standard deviation

### Processing LiDAR Data in ROS 2

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg: LaserScan):
        # Find closest obstacle
        min_range = min(msg.ranges)
        min_index = msg.ranges.index(min_range)

        angle = msg.angle_min + (min_index * msg.angle_increment)

        self.get_logger().info(
            f'Closest obstacle: {min_range:.2f}m at {angle:.2f} rad'
        )

        # Detect obstacles in front (±30°)
        front_ranges = msg.ranges[330:390]  # Assumes 720 samples
        front_min = min(front_ranges)

        if front_min < 0.5:
            self.get_logger().warn('Obstacle ahead! Stop or turn.')

def main():
    rclpy.init()
    processor = LidarProcessor()
    rclpy.spin(processor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## RGB-D Cameras

RGB-D cameras provide both color images and depth information, crucial for object detection and manipulation.

### Intel RealSense D435 Simulation

```xml
<gazebo reference="camera_link">
  <sensor name="realsense_d435" type="depth">
    <always_on>true</always_on>
    <visualize>true</visualize>
    <update_rate>30.0</update_rate>

    <camera>
      <horizontal_fov>1.50098</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>

    <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
      <ros>
        <remapping>~/image_raw:=camera/image_raw</remapping>
        <remapping>~/camera_info:=camera/camera_info</remapping>
        <remapping>~/depth/image_raw:=camera/depth/image_raw</remapping>
      </ros>
      <camera_name>realsense</camera_name>
      <frame_name>camera_optical_frame</frame_name>
      <hack_baseline>0.07</hack_baseline>
    </plugin>
  </sensor>
</gazebo>
```

### Processing RGB-D Data

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DepthProcessor(Node):
    def __init__(self):
        super().__init__('depth_processor')
        self.bridge = CvBridge()

        self.rgb_sub = self.create_subscription(
            Image, 'camera/image_raw', self.rgb_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, 'camera/depth/image_raw', self.depth_callback, 10
        )

    def rgb_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Process RGB image (e.g., object detection)
        cv2.imshow('RGB Camera', cv_image)
        cv2.waitKey(1)

    def depth_callback(self, msg: Image):
        cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

        # Normalize depth for visualization
        depth_viz = cv2.normalize(cv_depth, None, 0, 255, cv2.NORM_MINMAX)
        depth_viz = np.uint8(depth_viz)
        depth_colored = cv2.applyColorMap(depth_viz, cv2.COLORMAP_JET)

        # Calculate average depth in center region
        h, w = cv_depth.shape
        center_depth = cv_depth[h//2-50:h//2+50, w//2-50:w//2+50]
        avg_depth = np.nanmean(center_depth)

        self.get_logger().info(f'Center depth: {avg_depth:.2f}m')
        cv2.imshow('Depth Camera', depth_colored)
        cv2.waitKey(1)

def main():
    rclpy.init()
    processor = DepthProcessor()
    rclpy.spin(processor)
    processor.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
```

## IMU (Inertial Measurement Unit)

IMUs provide orientation, angular velocity, and linear acceleration data for odometry and state estimation.

### Gazebo IMU Plugin

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100.0</update_rate>

    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <remapping>~/out:=imu/data</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>

    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
  </sensor>
</gazebo>
```

### Processing IMU Data

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class ImuProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')
        self.subscription = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10
        )

    def imu_callback(self, msg: Imu):
        # Extract orientation (quaternion to Euler angles)
        q = msg.orientation
        roll = math.atan2(
            2.0 * (q.w * q.x + q.y * q.z),
            1.0 - 2.0 * (q.x**2 + q.y**2)
        )
        pitch = math.asin(2.0 * (q.w * q.y - q.z * q.x))
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y**2 + q.z**2)
        )

        # Extract angular velocity
        omega = msg.angular_velocity

        # Extract linear acceleration
        accel = msg.linear_acceleration

        self.get_logger().info(
            f'Orientation: Roll={math.degrees(roll):.1f}° '
            f'Pitch={math.degrees(pitch):.1f}° '
            f'Yaw={math.degrees(yaw):.1f}°'
        )

def main():
    rclpy.init()
    processor = ImuProcessor()
    rclpy.spin(processor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Sensor Fusion with robot_localization

Combine multiple sensor sources for accurate state estimation:

```yaml
# config/ekf.yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    sensor_timeout: 0.1
    two_d_mode: true

    # Odometry input
    odom0: /odom
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  true,  false,
                   false, false, true,
                   false, false, false]

    # IMU input
    imu0: /imu/data
    imu0_config: [false, false, false,
                  true,  true,  true,
                  false, false, false,
                  true,  true,  true,
                  true,  true,  true]

    # Output
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom
```

## Sensor Calibration Tips

1. **LiDAR Calibration:**
   - Verify scan orientation matches robot's front direction
   - Check that `angle_min` and `angle_max` cover expected range
   - Validate `range_min` and `range_max` against real sensor specs

2. **Camera Calibration:**
   - Use `camera_calibration` package for intrinsic parameters
   - Ensure RGB and depth frames are aligned (extrinsic calibration)
   - Test different lighting conditions in simulation

3. **IMU Calibration:**
   - Verify gravity vector points downward when stationary
   - Check angular velocity readings during rotation
   - Tune noise parameters to match real sensor specifications

## Common Issues & Solutions

| Issue | Cause | Solution |
|-------|-------|----------|
| LiDAR scans are noisy | High noise stddev | Reduce `noise.stddev` to 0.01 or lower |
| Camera images are black | Wrong frame or topic | Verify `frame_name` and topic remapping |
| IMU drift over time | Missing bias correction | Enable `imu_remove_gravitational_acceleration` |
| Depth values are NaN | Objects out of range | Adjust `clip.near` and `clip.far` |

## Next Steps

With sensors integrated into your simulation, you're ready to move to advanced navigation. Proceed to [Module 3: Isaac Sim](../module-3/isaac-sim-setup.md) to learn NVIDIA's photorealistic simulator with enhanced sensor models and Nav2 integration.
