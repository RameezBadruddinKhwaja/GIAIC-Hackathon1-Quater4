# Week 8: NVIDIA Isaac Sim - GPU-Accelerated Robotics

## Learning Objectives

By the end of this week, you will be able to:
- ✅ Install and configure NVIDIA Isaac Sim with Omniverse
- ✅ Understand USD (Universal Scene Description) format
- ✅ Create photorealistic simulation environments
- ✅ Use PhysX for accurate physics simulation
- ✅ Integrate Isaac Sim with ROS 2

## Introduction to Isaac Sim

NVIDIA Isaac Sim is a GPU-accelerated robotics simulator built on NVIDIA Omniverse, providing:
- **RTX ray tracing** for photorealistic rendering
- **PhysX 5** for accurate physics
- **USD** for scene composition
- **Synthetic data generation** for AI training
- **Native ROS 2 support**

```mermaid
graph TD
    A[Isaac Sim] --> B[Omniverse Kit]
    B --> C[PhysX 5]
    B --> D[RTX Renderer]
    B --> E[USD Scene]
    A --> F[ROS 2 Bridge]
    F --> G[/cmd_vel]
    F --> H[/camera/rgb]

    style A fill:#76b900
    style B fill:#e1f5ff
    style F fill:#fff9c4
```

## Installation

### Prerequisites

- **GPU**: NVIDIA RTX 2060 or higher (RTX 4090 recommended)
- **Driver**: 525.60.11 or later
- **OS**: Ubuntu 20.04/22.04 or Windows 10/11

### Install Omniverse Launcher

```bash
# Download Omniverse Launcher
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Make executable
chmod +x omniverse-launcher-linux.AppImage

# Run launcher
./omniverse-launcher-linux.AppImage
```

### Install Isaac Sim

1. Open Omniverse Launcher
2. **Exchange** tab → Search "Isaac Sim"
3. **Install Isaac Sim 2023.1.1**
4. Wait for download (~10GB)

### Verify Installation

```bash
# Launch Isaac Sim
~/.local/share/ov/pkg/isaac_sim-2023.1.1/isaac-sim.sh

# Check ROS 2 support
~/.local/share/ov/pkg/isaac_sim-2023.1.1/standalone_examples/api/omni.isaac.ros2_bridge/camera.py
```

## USD Scene Description

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

### Create Simple USD Scene

**simple_scene.usda**:
```usd
#usda 1.0
(
    defaultPrim = "World"
    upAxis = "Z"
)

def Xform "World"
{
    def Xform "GroundPlane"
    {
        def Mesh "Plane"
        {
            float3[] extent = [(-100, -100, 0), (100, 100, 0)]
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 1, 2, 3]
            point3f[] points = [(-100, -100, 0), (100, -100, 0), (100, 100, 0), (-100, 100, 0)]
        }
    }

    def Cube "Box" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI"]
    )
    {
        double size = 1.0
        double3 xformOp:translate = (0, 0, 5)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
}
```

### Load USD in Isaac Sim

<Tabs>
  <TabItem value="python" label="Python API" default>

```python
from omni.isaac.kit import SimulationApp

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

import omni.usd
from pxr import UsdGeom, Gf

# Get stage
stage = omni.usd.get_context().get_stage()

# Load USD scene
omni.usd.get_context().open_stage("simple_scene.usda")

# Add physics scene
from pxr import UsdPhysics
UsdPhysics.Scene.Define(stage, "/physicsScene")

# Play simulation
simulation_app.update()

# Keep running
import time
for i in range(1000):
    simulation_app.update()
    time.sleep(0.01)

# Cleanup
simulation_app.close()
```

  </TabItem>
  <TabItem value="gui" label="GUI">

1. **File → Open** → Select `simple_scene.usda`
2. **Create → Physics → Physics Scene**
3. Click **Play** button
4. Box falls due to gravity

  </TabItem>
</Tabs>

## ROS 2 Integration

### Enable ROS 2 Bridge

```python
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

# Enable ROS 2 extension
import omni.isaac.core.utils.extensions as extensions_utils
extensions_utils.enable_extension("omni.isaac.ros2_bridge")

import rclpy
from geometry_msgs.msg import Twist

# Initialize ROS 2
rclpy.init()

# Simulation loop with ROS 2
while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
rclpy.shutdown()
```

### Camera Publisher

```python
from omni.isaac.sensor import Camera
import omni.isaac.core.utils.numpy.rotations as rot_utils

# Create camera
camera = Camera(
    prim_path="/World/Camera",
    position=np.array([2.0, 2.0, 2.0]),
    frequency=30,
    resolution=(640, 480)
)

# Initialize camera
camera.initialize()

# Get RGB image
rgba = camera.get_rgba()

# Publish to ROS 2 (automatic with ROS2 Camera component)
camera.add_ros2_camera_publishers()
```

### Robot Control via ROS 2

```python
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction

# Create robot from USD
robot = Robot(prim_path="/World/my_robot")
robot.initialize()

# ROS 2 subscriber callback
def cmd_vel_callback(msg):
    # Convert Twist to joint velocities
    linear_vel = msg.linear.x
    angular_vel = msg.angular.z

    # Differential drive kinematics
    wheel_radius = 0.1
    wheel_separation = 0.4

    left_vel = (linear_vel - angular_vel * wheel_separation / 2) / wheel_radius
    right_vel = (linear_vel + angular_vel * wheel_separation / 2) / wheel_radius

    # Apply velocities
    robot.apply_wheel_actions(
        ArticulationAction(joint_velocities=[left_vel, right_vel])
    )

# Subscribe to cmd_vel
import rclpy
from geometry_msgs.msg import Twist

node = rclpy.create_node('isaac_controller')
node.create_subscription(Twist, 'cmd_vel', cmd_vel_callback, 10)

# Simulation loop
while simulation_app.is_running():
    rclpy.spin_once(node, timeout_sec=0)
    simulation_app.update()
```

## PhysX Configuration

### Rigid Body Physics

```python
from pxr import UsdPhysics, PhysxSchema

# Add rigid body
rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(prim)

# Set mass
mass_api = UsdPhysics.MassAPI.Apply(prim)
mass_api.CreateMassAttr(10.0)

# Set collision shape
collision_api = UsdPhysics.CollisionAPI.Apply(prim)

# PhysX material (friction, restitution)
material = UsdPhysics.Material.Define(stage, "/World/PhysicsMaterial")
material.CreateStaticFrictionAttr(0.8)
material.CreateDynamicFrictionAttr(0.6)
material.CreateRestitutionAttr(0.3)

# Bind material to collision
binding_api = UsdPhysics.MaterialBindingAPI.Apply(prim)
binding_api.Bind(material)
```

### Articulation (Joints)

```python
# Create revolute joint
joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/Robot/wheel_joint")
joint.CreateBody0Rel().SetTargets(["/World/Robot/base_link"])
joint.CreateBody1Rel().SetTargets(["/World/Robot/wheel_link"])
joint.CreateAxisAttr("Z")
joint.CreateLowerLimitAttr(-180)
joint.CreateUpperLimitAttr(180)

# Add drive
drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
drive.CreateTypeAttr("force")
drive.CreateMaxForceAttr(100.0)
drive.CreateTargetVelocityAttr(0.0)
```

## Synthetic Data Generation

```python
from omni.isaac.synthetic_utils import SyntheticDataHelper
import omni.replicator.core as rep

# Setup synthetic data
sd_helper = SyntheticDataHelper()

# Create writer for semantic segmentation
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="_output",
    rgb=True,
    semantic_segmentation=True,
    instance_segmentation=True,
    distance_to_camera=True,
    bounding_box_2d_tight=True
)

# Attach to camera
render_product = rep.create.render_product("/World/Camera", (1280, 720))
writer.attach([render_product])

# Trigger data collection
for i in range(1000):
    rep.orchestrator.step()
    simulation_app.update()
```

## Hardware Considerations

<Tabs>
  <TabItem value="rtx4090" label="RTX 4090 (Recommended)" default>

**Performance:**
- 4K rendering at 60 FPS
- Ray tracing enabled
- Multiple robot instances (10+)
- Real-time synthetic data generation

**Configuration:**
```python
simulation_app = SimulationApp({
    "width": 1920,
    "height": 1080,
    "anti_aliasing": 3,  # TAA
    "renderer": "RayTracedLighting",
    "headless": False
})
```

  </TabItem>
  <TabItem value="rtx3060" label="RTX 3060 (Minimum)">

**Performance:**
- 1080p rendering at 30 FPS
- Limited ray tracing
- Single robot instance
- Slower synthetic data

**Optimization:**
```python
simulation_app = SimulationApp({
    "width": 1280,
    "height": 720,
    "anti_aliasing": 0,
    "renderer": "PathTracing",  # Simpler renderer
    "headless": True  # Headless for better performance
})
```

  </TabItem>
</Tabs>

## Key Takeaways

- ✅ Isaac Sim provides GPU-accelerated photorealistic simulation
- ✅ USD format enables scene composition and reusability
- ✅ PhysX 5 delivers accurate physics simulation
- ✅ Native ROS 2 bridge simplifies integration
- ✅ RTX ray tracing enables synthetic data generation
- ✅ Requires NVIDIA RTX GPU for optimal performance

## Resources

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [USD Tutorials](https://openusd.org/release/tut_usd_tutorials.html)
- [PhysX SDK](https://nvidia-omniverse.github.io/PhysX/)

---

**🎓 Quiz**: [Week 8 Quiz](./quiz.md)
**💻 Challenge**: [Isaac Sim Environment Challenge](./challenge.md)
