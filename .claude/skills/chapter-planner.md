# chapter-planner Skill

## Purpose
Divide curriculum into 13 weeks across 4 parts with logical progression and dependency management.

## Key Patterns

### Curriculum Division Logic
**Part 1: The Nervous System (Weeks 1-5)** - ROS 2 Fundamentals
- Week 1: ROS 2 Basics → Prerequisites: None
- Week 2: Nodes & Topics → Prerequisites: Week 1
- Week 3: URDF Modeling → Prerequisites: Week 2
- Week 4: Services & Actions → Prerequisites: Week 2
- Week 5: Navigation2 → Prerequisites: Weeks 2-4

**Part 2: The Digital Twin (Weeks 6-7)** - Simulation
- Week 6: Gazebo Simulation → Prerequisites: Weeks 1-3
- Week 7: Unity Robotics Hub → Prerequisites: Week 6

**Part 3: The Brain (Weeks 8-10)** - NVIDIA Isaac
- Week 8: Isaac Sim Basics → Prerequisites: Week 6
- Week 9: Isaac ROS → Prerequisites: Weeks 5, 8
- Week 10: Isaac Orbit → Prerequisites: Week 9

**Part 4: VLA & Humanoids (Weeks 11-13)** - Advanced
- Week 11: VLA Introduction → Prerequisites: Week 10
- Week 12: DROID Deployment → Prerequisites: Week 11
- Week 13: Humanoid Control → Prerequisites: Weeks 11-12

### Module Grouping Principles
1. **Sequential Dependencies**: Each week builds on previous concepts
2. **Parallel Tracks**: Simulation (Weeks 6-7) can be learned alongside ROS 2 advanced topics
3. **Complexity Progression**: Basics → Intermediate → Advanced
4. **Practical Application**: Each part culminates in hands-on project

### Chapter Manifest Structure
```json
{
  "chapters": [
    {
      "chapter_id": "week-01-ros2-basics",
      "title": "Week 1: ROS 2 Basics",
      "part_number": 1,
      "week_number": 1,
      "learning_objectives": [
        "Understand ROS 2 architecture and DDS middleware",
        "Install ROS 2 Humble on Ubuntu 22.04",
        "Create first publisher/subscriber nodes"
      ],
      "prerequisites": [],
      "estimated_hours": 8
    }
  ]
}
```

## Usage Context
- Initial curriculum design
- Adding new weeks or reordering content
- Validating prerequisite chains
- Generating course roadmap
