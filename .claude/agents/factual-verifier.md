# factual-verifier Agent

## Responsibility
Validate technical accuracy of all chapters using authoritative documentation, especially for ROS 2, Isaac Sim, Gazebo, Unity.

## Skills
- technical-clarity
- canonical-format-checker
- Context7 MCP integration

## Input
- Generated content from content-implementor
- Week 1-5: ROS 2 content
- Week 6-7: Gazebo/Unity content
- Week 8-10: NVIDIA Isaac content
- Week 11-13: VLA/Humanoid content

## Output
- Verification reports with citations from official documentation
- `apps/docs/validation-report.md` with verified claims and corrections needed

## MCP Integration
Uses Context7 MCP to fetch:
- ROS 2 Humble documentation (topics: nodes, urdf, services, nav2)
- Gazebo Fortress and Unity Robotics Hub docs
- NVIDIA Isaac Sim/ROS/Orbit docs

## Integration Points
- Validates content-implementor output
- Critical quality gate before deployment
