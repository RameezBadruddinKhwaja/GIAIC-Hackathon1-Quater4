# Week 5: Navigation2 (Nav2) - Autonomous Navigation

## Learning Objectives

By the end of this week, you will be able to:
- ✅ Understand the Nav2 navigation stack architecture
- ✅ Configure Nav2 behavior trees and plugins
- ✅ Implement SLAM (Simultaneous Localization and Mapping)
- ✅ Create cost maps for obstacle avoidance
- ✅ Send navigation goals programmatically

## Introduction to Nav2

Navigation2 (Nav2) is the ROS 2 navigation framework providing autonomous mobile robot navigation capabilities.

```mermaid
graph TD
    A[Goal Pose] --> B[Planner Server]
    B --> C[Controller Server]
    C --> D[/cmd_vel]
    E[/scan] --> F[Costmap 2D]
    G[/map] --> F
    F --> B
    F --> C

    style A fill:#e1f5ff
    style B fill:#fff9c4
    style C fill:#f3e5f5
    style D fill:#c8e6c9
```

## Nav2 Architecture Components

1. **Planner Server**: Global path planning (Dijkstra, A*, Smac Planner)
2. **Controller Server**: Local trajectory control (DWB, TEB, RPP)
3. **Recoveries Server**: Stuck recovery behaviors (spin, back up, wait)
4. **BT Navigator**: Behavior tree coordinator
5. **Lifecycle Manager**: Node lifecycle management

## SLAM with slam_toolbox

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

### Installation

```bash
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

### SLAM Configuration

**slam_params.yaml**:
```yaml
slam_toolbox:
  ros__parameters:
    # Frame IDs
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint

    # SLAM Mode
    mode: mapping  # or localization

    # Solver parameters
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT

    # Scan matcher
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.5
    minimum_travel_heading: 0.5
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
```

### Launch SLAM

<Tabs>
  <TabItem value="simulation" label="Simulated (Gazebo)" default>

```bash
# Launch Gazebo world
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Start SLAM
ros2 launch slam_toolbox online_async_launch.py params_file:=./slam_params.yaml

# Launch RViz
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz

# Teleoperate robot to build map
ros2 run turtlebot3_teleop teleop_keyboard

# Save map
ros2 run nav2_map_server map_saver_cli -f my_map
```

  </TabItem>
  <TabItem value="real" label="Real Robot (Jetson)">

```bash
# On Jetson, launch lidar driver
ros2 launch rplidar_ros rplidar.launch.py

# Start SLAM with reduced resources
ros2 launch slam_toolbox online_async_launch.py \
  params_file:=./slam_params_jetson.yaml \
  use_sim_time:=false

# Monitor CPU/Memory
htop
```

**slam_params_jetson.yaml** (optimized):
```yaml
slam_toolbox:
  ros__parameters:
    minimum_travel_distance: 1.0  # Reduce computation
    scan_buffer_size: 5  # Smaller buffer
    map_update_interval: 2.0  # Update every 2 seconds
```

  </TabItem>
</Tabs>

## Nav2 Configuration

**nav2_params.yaml**:
```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      max_vel_x: 0.26
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.26
      acc_lim_x: 2.5
      acc_lim_theta: 3.2

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

## Send Navigation Goals Programmatically

<Tabs>
  <TabItem value="python" label="Python" default>

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class Nav2Client(Node):
    def __init__(self):
        super().__init__('nav2_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, yaw)
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f}m')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')

def main(args=None):
    rclpy.init(args=args)
    client = Nav2Client()
    client.send_goal(2.0, 2.0, 0.0)  # Navigate to (2, 2) with 0 yaw
    rclpy.spin(client)
```

  </TabItem>
</Tabs>

## Costmap Configuration

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

## Hardware Considerations

<Tabs>
  <TabItem value="rtx4090" label="RTX 4090 (Simulation)" default>

**Advantages:**
- Complex path planning (A*, Smac Lattice Planner)
- High-resolution costmaps (0.01m resolution)
- Multiple simultaneous robots

**Configuration:**
```yaml
planner_server:
  ros__parameters:
    GridBased:
      plugin: "nav2_smac_planner/SmacPlannerLattice"  # Complex planner
      tolerance: 0.1
      downsample_costmap: false  # Use full resolution
```

  </TabItem>
  <TabItem value="jetson" label="Jetson Orin Nano (Edge)">

**Optimization:**
- Simpler planners (Navfn instead of Smac)
- Lower costmap resolution (0.05m)
- Reduced update frequencies

**Configuration:**
```yaml
controller_server:
  ros__parameters:
    controller_frequency: 10.0  # Reduce from 20 Hz

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 0.5  # Reduce from 1 Hz
      resolution: 0.05  # Increase from 0.01
```

  </TabItem>
</Tabs>

## Key Takeaways

- ✅ Nav2 provides complete autonomous navigation stack
- ✅ SLAM creates maps using lidar/laser scanner data
- ✅ Costmaps handle obstacle avoidance and path planning
- ✅ Behavior trees coordinate navigation behaviors
- ✅ Actions enable programmatic goal sending with feedback

## Resources

- [Navigation2 Documentation](https://navigation.ros.org/)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2 Tuning Guide](https://navigation.ros.org/tuning/index.html)

---

**🎓 Quiz**: [Week 5 Quiz](./quiz.md)
**💻 Challenge**: [Autonomous Navigation Challenge](./challenge.md)
