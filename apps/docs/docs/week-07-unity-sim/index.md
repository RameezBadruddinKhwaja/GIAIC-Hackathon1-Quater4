# Week 7: Unity Robotics Hub - ML-Ready Simulation

## Learning Objectives

By the end of this week, you will be able to:
- ✅ Set up Unity with ROS 2 integration
- ✅ Use Unity ML-Agents for reinforcement learning
- ✅ Create photorealistic robot simulations
- ✅ Implement ROS-TCP-Connector for Unity ↔ ROS 2 communication
- ✅ Collect synthetic training data in Unity

## Introduction to Unity for Robotics

Unity provides high-fidelity graphics, physics, and ML capabilities, making it ideal for:
- **Photorealistic rendering** (computer vision algorithm training)
- **Reinforcement learning** (Unity ML-Agents)
- **Large-scale data generation** (synthetic datasets)

### Unity vs Gazebo

| Feature | Gazebo | Unity |
|---------|--------|-------|
| Graphics Quality | Good | Excellent (photorealistic) |
| Physics | Excellent | Good |
| ML Integration | Limited | Native (ML-Agents) |
| ROS 2 Integration | Native | Via ROS-TCP-Connector |
| Learning Curve | Moderate | Steep |

## Installation

### Unity Hub & Editor

```bash
# Download Unity Hub
wget https://public-cdn.cloud.unity3d.com/hub/prod/UnityHubSetup.AppImage

# Install Unity Editor 2022.3 LTS
# (Use Unity Hub GUI)
```

### ROS-TCP-Connector

```bash
# Clone ROS-TCP-Endpoint (ROS 2 side)
cd ~/ros2_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git -b main-ros2

# Build
cd ~/ros2_ws
colcon build --packages-select ros_tcp_endpoint

# Launch endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

## Unity Project Setup

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

### 1. Install Unity Packages

In Unity Editor:
1. **Window → Package Manager**
2. **Add package from git URL**:
   - `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
   - `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer`

### 2. Configure ROS Settings

**Robotics → ROS Settings**:
```
ROS IP Address: 192.168.1.100  (your ROS 2 machine IP)
ROS Port: 10000
Protocol: ROS 2
Serializer: MessageGeneration
```

### 3. Import URDF

```csharp
// Unity C# Script: ImportRobot.cs
using UnityEngine;
using Unity.Robotics.UrdfImporter;

public class ImportRobot : MonoBehaviour
{
    void Start()
    {
        // Import URDF
        var urdfPath = "Assets/URDF/my_robot.urdf";
        var robot = UrdfRobotExtensions.Create(urdfPath);

        // Configure articulation body (Unity physics)
        var articulationBody = robot.GetComponent<ArticulationBody>();
        articulationBody.immovable = false;
    }
}
```

## ROS 2 ↔ Unity Communication

<Tabs>
  <TabItem value="unity" label="Unity (C#)" default>

**Publisher.cs**:
```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class UnityPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "unity_data";
    public float publishRate = 10f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(topicName);
        InvokeRepeating(nameof(PublishMessage), 1f, 1f / publishRate);
    }

    void PublishMessage()
    {
        var msg = new StringMsg
        {
            data = $"Unity simulation time: {Time.time}"
        };
        ros.Publish(topicName, msg);
    }
}
```

**Subscriber.cs**:
```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class UnitySubscriber : MonoBehaviour
{
    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>("cmd_vel", ReceiveCmdVel);
    }

    void ReceiveCmdVel(TwistMsg msg)
    {
        Debug.Log($"Received cmd_vel: linear.x={msg.linear.x}, angular.z={msg.angular.z}");
        // Apply to robot movement
        transform.Translate(Vector3.forward * msg.linear.x * Time.deltaTime);
        transform.Rotate(Vector3.up, msg.angular.z * Mathf.Rad2Deg * Time.deltaTime);
    }
}
```

  </TabItem>
  <TabItem value="ros2" label="ROS 2 (Python)">

**unity_controller.py**:
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class UnityController(Node):
    def __init__(self):
        super().__init__('unity_controller')

        # Publish cmd_vel to Unity
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribe to Unity data
        self.unity_sub = self.create_subscription(
            String, 'unity_data', self.unity_callback, 10
        )

        # Send commands at 10 Hz
        self.timer = self.create_timer(0.1, self.send_commands)

    def send_commands(self):
        msg = Twist()
        msg.linear.x = 1.0  # Move forward at 1 m/s
        msg.angular.z = 0.5  # Turn at 0.5 rad/s
        self.cmd_vel_pub.publish(msg)

    def unity_callback(self, msg):
        self.get_logger().info(f'Unity says: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = UnityController()
    rclpy.spin(node)
```

  </TabItem>
</Tabs>

## Unity ML-Agents Integration

### Install ML-Agents

```bash
pip3 install mlagents==0.30.0
```

### Create RL Agent

**RobotAgent.cs**:
```csharp
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine;

public class RobotAgent : Agent
{
    public Transform target;
    Rigidbody rBody;

    void Start()
    {
        rBody = GetComponent<Rigidbody>();
    }

    public override void OnEpisodeBegin()
    {
        // Reset robot position
        transform.localPosition = new Vector3(0, 0.5f, 0);
        rBody.velocity = Vector3.zero;

        // Randomize target position
        target.localPosition = new Vector3(
            Random.Range(-4f, 4f),
            0.5f,
            Random.Range(-4f, 4f)
        );
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Observation space: 8 dimensions
        sensor.AddObservation(target.localPosition);  // 3
        sensor.AddObservation(transform.localPosition);  // 3
        sensor.AddObservation(rBody.velocity.x);  // 1
        sensor.AddObservation(rBody.velocity.z);  // 1
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Action space: 2 dimensions (linear, angular)
        float linear = actions.ContinuousActions[0];
        float angular = actions.ContinuousActions[1];

        Vector3 controlSignal = Vector3.zero;
        controlSignal.x = angular;
        controlSignal.z = linear;
        rBody.AddForce(controlSignal * 10f);

        // Reward shaping
        float distanceToTarget = Vector3.Distance(transform.localPosition, target.localPosition);

        if (distanceToTarget < 1.42f)
        {
            SetReward(1.0f);
            EndEpisode();
        }
        else if (transform.localPosition.y < 0)
        {
            EndEpisode();
        }

        // Distance-based reward
        AddReward(-0.001f * distanceToTarget);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Manual control for testing
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Horizontal");
        continuousActionsOut[1] = Input.GetAxis("Vertical");
    }
}
```

### Train ML Agent

```bash
# Start training
mlagents-learn config/robot_config.yaml --run-id=robot_navigation_v1

# Press Play in Unity Editor when prompted
```

## Synthetic Data Collection

```csharp
// CameraCapture.cs
using UnityEngine;
using System.IO;

public class CameraCapture : MonoBehaviour
{
    public Camera renderCamera;
    public int captureWidth = 1920;
    public int captureHeight = 1080;
    public string savePath = "Assets/Captures/";

    void Start()
    {
        InvokeRepeating(nameof(CaptureImage), 1f, 0.1f);  // 10 Hz
    }

    void CaptureImage()
    {
        RenderTexture rt = new RenderTexture(captureWidth, captureHeight, 24);
        renderCamera.targetTexture = rt;

        Texture2D screenshot = new Texture2D(captureWidth, captureHeight, TextureFormat.RGB24, false);
        renderCamera.Render();

        RenderTexture.active = rt;
        screenshot.ReadPixels(new Rect(0, 0, captureWidth, captureHeight), 0, 0);

        renderCamera.targetTexture = null;
        RenderTexture.active = null;
        Destroy(rt);

        byte[] bytes = screenshot.EncodeToPNG();
        string filename = $"{savePath}frame_{Time.frameCount}.png";
        File.WriteAllBytes(filename, bytes);

        Debug.Log($"Saved: {filename}");
    }
}
```

## Hardware Considerations

<Tabs>
  <TabItem value="rtx4090" label="RTX 4090 (Simulation)" default>

**Advantages:**
- Ray tracing enabled (photorealistic rendering)
- Train ML agents at 100+ FPS
- Generate 1000+ synthetic images/second

**Configuration (Unity Graphics Settings)**:
```
Quality: Ultra
Ray Tracing: Enabled
Resolution: 1920x1080
VSync: Disabled (max FPS)
```

  </TabItem>
  <TabItem value="cloud" label="Cloud (AWS/GCP)">

**Use Case**: Massively parallel training

**AWS EC2 Instance**: `g4dn.xlarge` (T4 GPU)
```bash
# Headless Unity training
xvfb-run --auto-servernum --server-args='-screen 0 640x480x24' \
  mlagents-learn config.yaml --run-id=cloud_training_v1
```

  </TabItem>
</Tabs>

## Key Takeaways

- ✅ Unity provides photorealistic simulation for computer vision
- ✅ ROS-TCP-Connector bridges Unity and ROS 2
- ✅ ML-Agents enables reinforcement learning training
- ✅ Synthetic data generation accelerates ML model development
- ✅ Unity complements Gazebo for visual fidelity needs

## Resources

- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ML-Agents Documentation](https://github.com/Unity-Technologies/ml-agents)
- [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)

---

**🎓 Quiz**: [Week 7 Quiz](./quiz.md)
**💻 Challenge**: [RL Navigation Challenge](./challenge.md)
