# ROS 2 Mastery Skill

**Description**: Deep knowledge of ROS 2 architecture, concepts, and implementation

**Core Knowledge**:

## ROS 2 Architecture
- **Nodes**: Executables that perform computation (publishers, subscribers, services, clients)
- **Topics**: Named buses for message passing (pub/sub pattern, asynchronous)
- **Services**: Request/response pattern (synchronous, client-server)
- **Actions**: Long-running tasks with feedback and cancellation
- **Parameters**: Configuration values for nodes
- **Launch Files**: Start multiple nodes with configuration

## Key Concepts
- **DDS (Data Distribution Service)**: Underlying middleware (eProsima Fast-DDS default)
- **QoS (Quality of Service)**: Reliability, durability, history settings
- **URDF (Unified Robot Description Format)**: XML robot model description
- **TF (Transform)**: Coordinate frame transformations
- **Navigation2**: Autonomous navigation stack (SLAM, path planning, recovery)
- **ROS 2 Humble**: LTS release for Ubuntu 22.04 (recommended for production)

## Common Commands
```bash
# List nodes
ros2 node list

# List topics
ros2 topic list

# Echo topic messages
ros2 topic echo /topic_name

# Publish to topic
ros2 topic pub /topic_name std_msgs/msg/String "{data: 'hello'}"

# Call service
ros2 service call /service_name std_srvs/srv/Empty

# Launch file
ros2 launch package_name launch_file.py
```

## Python API
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.subscription = self.create_subscription(
            String, 'topic', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

## C++ API
```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MyNode : public rclcpp::Node {
public:
  MyNode() : Node("my_node") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MyNode::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
  }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNode>());
  rclcpp::shutdown();
  return 0;
}
```

## Keyword Triggers
- **Load when query contains**: SLAM, ROS, ROS2, node, topic, service, action, URDF, TF, Navigation2, Nav2, DDS, QoS

**Invocation**:
Automatically loaded when user asks questions about ROS 2 concepts, commands, or implementation.
