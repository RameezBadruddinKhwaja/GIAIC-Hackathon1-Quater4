---
id: vla-introduction
title: Vision-Language-Action Models Introduction
sidebar_label: VLA Introduction
description: Understand VLA models like RT-1 and RT-2 for robotic manipulation
---

# Vision-Language-Action Models: The Future of Robotic Intelligence

Vision-Language-Action (VLA) models represent a paradigm shift in robotics: instead of hand-coding behaviors, robots learn end-to-end policies that map natural language commands and camera observations directly to motor actions. Models like RT-1, RT-2, and Gemini-powered systems are making general-purpose robots a reality.

## What is a VLA Model?

A VLA model is a neural network that takes three inputs and produces robot actions:

```mermaid
graph LR
    A[Camera Image] --> D[VLA Model<br/>Transformer]
    B[Language Command<br/>"Pick up the cup"] --> D
    C[Proprioception<br/>Joint States] --> D

    D -->|Action Distribution| E[Motor Commands]
    E --> F[Robot Actuators]

    style D fill:#4CAF50,color:#fff
    style A fill:#2196F3,color:#fff
    style B fill:#FF9800,color:#fff
    style E fill:#F44336,color:#fff
```

**Key Characteristics:**
- **End-to-End**: No hand-coded perception, planning, or control
- **Multimodal**: Fuses vision, language, and proprioception
- **Generalist**: Single model handles many tasks (unlike task-specific policies)
- **Data-Driven**: Trained on millions of robot demonstrations

## RT-1: Robotics Transformer 1

RT-1 (released by Google DeepMind, 2022) was the first transformer-based VLA model trained on real robot data.

### RT-1 Architecture

```python
# Simplified RT-1 architecture
class RT1Model:
    def __init__(self):
        # Vision backbone: EfficientNet-B3
        self.vision_encoder = EfficientNetB3(pretrained=True)

        # Language encoder: Universal Sentence Encoder
        self.language_encoder = UniversalSentenceEncoder()

        # Transformer: FiLM-conditioned Token Learner
        self.transformer = TokenLearnerTransformer(
            num_layers=8,
            d_model=512,
            num_heads=8
        )

        # Action head: Discretized action bins
        self.action_head = nn.Linear(512, 11 * 256)  # 11 DOF, 256 bins each

    def forward(self, image, text, proprioception):
        # Encode image (224x224 RGB) → tokens
        vision_tokens = self.vision_encoder(image)  # Shape: [batch, 81, 512]

        # Encode text → single embedding
        language_embedding = self.language_encoder(text)  # Shape: [batch, 512]

        # FiLM conditioning: modulate vision tokens with language
        conditioned_tokens = self.film_layer(vision_tokens, language_embedding)

        # Add proprioception as tokens
        proprio_tokens = self.proprio_encoder(proprioception)
        all_tokens = torch.cat([conditioned_tokens, proprio_tokens], dim=1)

        # Transformer processing
        output_tokens = self.transformer(all_tokens)

        # Action prediction (discretized bins)
        action_logits = self.action_head(output_tokens[:, 0])  # Use [CLS] token
        action_logits = action_logits.view(-1, 11, 256)  # [batch, 11 DOF, 256 bins]

        return action_logits
```

### RT-1 Training Data: The Open X-Embodiment Dataset

RT-1 was trained on **130,000 demonstrations** from multiple robot embodiments:

| Robot | Tasks | Demonstrations |
|-------|-------|----------------|
| WidowX 250 | Pick-and-place | 50,000 |
| Franka Panda | Manipulation | 30,000 |
| Google Robot (Everyday Robots) | Mobile manipulation | 50,000 |

**Data Format:**
```python
# Single trajectory example
{
    "observation": {
        "image": np.array([224, 224, 3]),  # RGB camera
        "state": np.array([7]),  # Joint positions
    },
    "action": np.array([7]),  # Joint velocities/positions
    "language_instruction": "pick up the blue cup",
    "success": True
}
```

### Action Discretization

RT-1 discretizes continuous actions into 256 bins per degree of freedom:

```python
def discretize_action(continuous_action, min_val=-1.0, max_val=1.0, num_bins=256):
    """
    Convert continuous action to discrete bin index.

    Args:
        continuous_action: Float in range [min_val, max_val]
        min_val: Minimum action value
        max_val: Maximum action value
        num_bins: Number of discretization bins

    Returns:
        bin_index: Integer in range [0, num_bins-1]
    """
    # Normalize to [0, 1]
    normalized = (continuous_action - min_val) / (max_val - min_val)
    normalized = np.clip(normalized, 0.0, 1.0)

    # Map to bin index
    bin_index = int(normalized * (num_bins - 1))
    return bin_index

def undiscretize_action(bin_index, min_val=-1.0, max_val=1.0, num_bins=256):
    """
    Convert discrete bin index back to continuous action.
    """
    # Map bin to [0, 1]
    normalized = bin_index / (num_bins - 1)

    # Denormalize to [min_val, max_val]
    continuous_action = normalized * (max_val - min_val) + min_val
    return continuous_action

# Example usage
continuous_action = 0.5  # Joint velocity
bin_index = discretize_action(continuous_action)  # → 191
recovered_action = undiscretize_action(bin_index)  # → 0.4996
```

**Why discretize?**
- Transforms regression → classification (easier to train)
- Categorical cross-entropy loss is more stable than MSE for actions
- Enables transformer to use categorical attention

## RT-2: Robotics Transformer 2 (Vision-Language-Action)

RT-2 (2023) extends RT-1 by using a pretrained vision-language model (PaLM-E or Gemini) as the backbone.

### RT-2 Architecture

```mermaid
graph TD
    A[Camera Image] --> B[Vision Encoder<br/>ViT]
    C[Language<br/>"Move forward"] --> D[Text Tokenizer]

    B --> E[Pretrained VLM<br/>PaLM-E/Gemini]
    D --> E

    E --> F[Action Tokens<br/>Fine-tuned]
    F --> G[Action Decoder]
    G --> H[7-DOF Action]

    style E fill:#4CAF50,color:#fff
    style A fill:#2196F3,color:#fff
    style H fill:#F44336,color:#fff
```

**Key Innovation:** RT-2 treats **actions as language tokens**, enabling the model to leverage internet-scale vision-language pretraining.

### Training RT-2 from Pretrained VLM

```python
#!/usr/bin/env python3
"""
Fine-tune a pretrained VLM (e.g., Gemini) for robot actions
"""
import torch
import torch.nn as nn
from transformers import AutoModel, AutoTokenizer

class RT2Model(nn.Module):
    def __init__(self, vlm_model_name="google/paligemma-3b-pt-224", num_actions=7):
        super().__init__()

        # Load pretrained vision-language model
        self.vlm = AutoModel.from_pretrained(vlm_model_name)
        self.tokenizer = AutoTokenizer.from_pretrained(vlm_model_name)

        # Freeze VLM backbone (only fine-tune action head)
        for param in self.vlm.parameters():
            param.requires_grad = False

        # Action head (maps VLM hidden states → robot actions)
        hidden_size = self.vlm.config.hidden_size  # e.g., 2048 for Gemma
        self.action_head = nn.Sequential(
            nn.Linear(hidden_size, 512),
            nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(512, num_actions * 256)  # 7 DOF × 256 bins
        )

    def forward(self, image, text_instruction):
        """
        Args:
            image: [batch, 3, 224, 224]
            text_instruction: List of strings

        Returns:
            action_logits: [batch, 7, 256]
        """
        # Tokenize text
        text_inputs = self.tokenizer(
            text_instruction,
            return_tensors="pt",
            padding=True,
            truncation=True
        ).to(image.device)

        # Forward through VLM (image + text → hidden states)
        outputs = self.vlm(
            pixel_values=image,
            input_ids=text_inputs['input_ids'],
            attention_mask=text_inputs['attention_mask']
        )

        # Use [CLS] token or mean pooling
        hidden_state = outputs.last_hidden_state[:, 0, :]  # [batch, hidden_size]

        # Predict actions
        action_logits = self.action_head(hidden_state)
        action_logits = action_logits.view(-1, 7, 256)  # [batch, 7 DOF, 256 bins]

        return action_logits

# Training loop
model = RT2Model()
optimizer = torch.optim.AdamW(model.action_head.parameters(), lr=1e-4)
criterion = nn.CrossEntropyLoss()

for batch in dataloader:
    images, texts, actions_gt = batch  # actions_gt: [batch, 7] (discretized bins)

    # Forward pass
    action_logits = model(images, texts)  # [batch, 7, 256]

    # Compute loss for each DOF
    loss = 0
    for dof in range(7):
        loss += criterion(action_logits[:, dof, :], actions_gt[:, dof])
    loss /= 7

    # Backward pass
    optimizer.zero_grad()
    loss.backward()
    optimizer.step()
```

## Gemini for Robot Control

Google's Gemini 2.0 Flash (multimodal LLM) can be used as a VLA backbone for real-time robot control.

### Gemini VLA Pipeline

```python
#!/usr/bin/env python3
"""
Use Gemini 2.0 Flash for robot action prediction
"""
import google.generativeai as genai
from PIL import Image
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage, JointState
from std_msgs.msg import String
from cv_bridge import CvBridge

genai.configure(api_key="YOUR_GEMINI_API_KEY")

class GeminiVLARobot(Node):
    def __init__(self):
        super().__init__('gemini_vla_robot')

        # Gemini model
        self.model = genai.GenerativeModel('gemini-2.0-flash-exp')

        # ROS subscribers
        self.camera_sub = self.create_subscription(
            ROSImage, '/camera/image_raw', self.camera_callback, 10
        )
        self.command_sub = self.create_subscription(
            String, '/voice_command', self.command_callback, 10
        )

        # ROS publisher
        self.action_pub = self.create_publisher(JointState, '/joint_commands', 10)

        # State
        self.latest_image = None
        self.bridge = CvBridge()

    def camera_callback(self, msg):
        # Convert ROS Image → PIL Image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        self.latest_image = Image.fromarray(cv_image)

    def command_callback(self, msg):
        if self.latest_image is None:
            self.get_logger().warn('No camera image available')
            return

        instruction = msg.data
        self.get_logger().info(f'Executing: {instruction}')

        # Generate action from Gemini
        action = self.predict_action(self.latest_image, instruction)

        # Publish action
        joint_cmd = JointState()
        joint_cmd.header.stamp = self.get_clock().now().to_msg()
        joint_cmd.position = action.tolist()
        self.action_pub.publish(joint_cmd)

    def predict_action(self, image, instruction):
        """
        Use Gemini to predict robot action given image and instruction.

        Returns:
            action: np.array([7]) - joint positions/velocities
        """
        # Construct prompt
        prompt = f"""
You are a robot control system. Given this camera view and the instruction:
"{instruction}", predict the next robot action as 7 joint values (each between -1.0 and 1.0).

Output ONLY a Python list like: [0.1, -0.2, 0.5, 0.0, 0.3, -0.1, 0.0]
DO NOT include any explanation.
"""

        # Call Gemini API
        response = self.model.generate_content([prompt, image])

        # Parse action from response
        try:
            action_text = response.text.strip()
            action = eval(action_text)  # Parse "[0.1, -0.2, ...]"
            action = np.array(action, dtype=np.float32)

            # Clip to [-1, 1]
            action = np.clip(action, -1.0, 1.0)

            self.get_logger().info(f'Predicted action: {action}')
            return action

        except Exception as e:
            self.get_logger().error(f'Failed to parse action: {e}')
            return np.zeros(7, dtype=np.float32)

def main():
    rclpy.init()
    node = GeminiVLARobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Sim-to-Real Transfer

VLA models trained in simulation must transfer to real hardware. Key techniques:

### Domain Randomization

```python
def randomize_visual_domain(image):
    """
    Apply visual domain randomization to simulation images.
    """
    # Randomize lighting
    brightness = np.random.uniform(0.7, 1.3)
    image = np.clip(image * brightness, 0, 255).astype(np.uint8)

    # Randomize contrast
    contrast = np.random.uniform(0.8, 1.2)
    mean = np.mean(image)
    image = np.clip((image - mean) * contrast + mean, 0, 255).astype(np.uint8)

    # Add noise
    noise = np.random.normal(0, 5, image.shape)
    image = np.clip(image + noise, 0, 255).astype(np.uint8)

    # Random color shift
    color_shift = np.random.uniform(-10, 10, 3)
    image = np.clip(image + color_shift, 0, 255).astype(np.uint8)

    return image
```

### Physics Randomization

```python
# In Isaac Sim PhysX configuration
def randomize_physics(scene):
    # Randomize friction
    friction = np.random.uniform(0.5, 1.5)
    scene.set_friction(friction)

    # Randomize object masses (±20%)
    mass_scale = np.random.uniform(0.8, 1.2)
    for obj in scene.objects:
        obj.mass *= mass_scale

    # Randomize actuator gains
    kp_scale = np.random.uniform(0.9, 1.1)
    kd_scale = np.random.uniform(0.9, 1.1)
    robot.set_gains(kp * kp_scale, kd * kd_scale)
```

## Comparison: RT-1 vs RT-2 vs Gemini VLA

| Feature | RT-1 | RT-2 | Gemini VLA |
|---------|------|------|------------|
| **Backbone** | EfficientNet + custom Transformer | Pretrained VLM (PaLM-E) | Gemini 2.0 Flash |
| **Pretraining Data** | Robot demos only (130K) | Internet images + robot demos | Multimodal internet data |
| **Zero-Shot Capability** | Poor (task-specific) | Good (generalizes to novel objects) | Excellent (natural language reasoning) |
| **Latency** | ~50ms (on GPU) | ~100ms | ~200ms (API call) |
| **Deployment** | Local inference | Local inference | Cloud API |
| **Custom Fine-Tuning** | Yes | Yes | Limited (prompt engineering only) |

## Practical Deployment Example

```python
#!/usr/bin/env python3
"""
Deploy RT-2 model on real robot (Franka Panda)
"""
import torch
from rt2_model import RT2Model
from camera_interface import CameraCapture
from robot_controller import FrankaPandaController

# Load trained model
model = RT2Model.from_pretrained("checkpoints/rt2_franka.pth")
model.eval()
model.cuda()

# Initialize hardware
camera = CameraCapture(device='/dev/video0', resolution=(224, 224))
robot = FrankaPandaController(ip='192.168.1.100')

# Control loop (20 Hz)
rate = 1/20.0  # 50ms
while True:
    # Capture image
    image = camera.capture()  # Returns [3, 224, 224] tensor

    # Get user instruction (from voice or UI)
    instruction = "pick up the red block"

    # Predict action
    with torch.no_grad():
        action_logits = model(image.unsqueeze(0).cuda(), [instruction])
        action_bins = torch.argmax(action_logits, dim=-1).squeeze(0)  # [7]

    # Undiscretize actions
    action = undiscretize_action(action_bins.cpu().numpy())

    # Send to robot
    robot.set_joint_velocities(action)

    # Sleep
    time.sleep(rate)
```

## Next Steps

Now that you understand VLA models, you can integrate voice control to make your robot respond to spoken commands. Proceed to [Voice to Action](./voice-to-action.md) to learn about speech recognition with Whisper and command execution.
