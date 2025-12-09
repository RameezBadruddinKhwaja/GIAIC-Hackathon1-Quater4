# quiz-generator Skill

## Purpose
Create interactive quizzes with multiple-choice questions and code challenges for assessment.

## Key Patterns

### Multiple-Choice Quiz Format
```markdown
## Week 1 Quiz

### Question 1: ROS 2 Architecture

What is the primary communication middleware used in ROS 2?

A) MQTT
B) DDS (Data Distribution Service) ✅
C) WebSockets
D) gRPC

<details>
<summary>Explanation</summary>

DDS (Data Distribution Service) is the correct answer. ROS 2 uses DDS as its middleware layer, providing real-time, distributed communication between nodes. Unlike ROS 1's single master architecture, DDS enables peer-to-peer discovery and communication.

</details>

### Question 2: Node Communication

Which command lists all active ROS 2 topics?

A) ros2 node list
B) ros2 topic list ✅
C) ros2 topic show
D) ros2 run

<details>
<summary>Explanation</summary>

`ros2 topic list` displays all active topics in the ROS 2 system. Use `ros2 topic echo /topic_name` to see messages being published.

</details>
```

### Code Challenge Format
```markdown
## Challenge: Modify the Publisher

**Objective**: Modify the Hello Robot publisher to publish at 2 Hz instead of 1 Hz.

**Starter Code**:
```python
class HelloPublisher(Node):
    def __init__(self):
        super().__init__('hello_publisher')
        self.publisher_ = self.create_publisher(String, 'robot_says', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # ← Change this
        self.count = 0
```

**Your Task**:
1. Change the timer frequency to 2 Hz
2. Update the message format to include timestamp
3. Run both publisher and subscriber to verify

<details>
<summary>Solution</summary>

```python
class HelloPublisher(Node):
    def __init__(self):
        super().__init__('hello_publisher')
        self.publisher_ = self.create_publisher(String, 'robot_says', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)  # 2 Hz = 0.5 seconds
        self.count = 0

    def timer_callback(self):
        msg = String()
        import time
        timestamp = time.time()
        msg.data = f'Hello Robot! Count: {self.count} | Time: {timestamp:.2f}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1
```

</details>
```

### Difficulty Progression
- **Beginner**: Recall and understanding (Weeks 1-3)
- **Intermediate**: Application and analysis (Weeks 4-7)
- **Advanced**: Synthesis and evaluation (Weeks 8-13)

## Usage Context
- End-of-chapter assessments
- Mid-course checkpoints
- Certification preparation
- Self-study validation
