# code-example-generator Skill

## Purpose
Generate ROS 2/Python/C++ code snippets with proper Docusaurus Tabs for "Simulated" vs "Real Robot" variations.

## Usage Context
Used by content-implementor when authoring week content with code examples.

## Agent Assignments
- Primary: content-implementor
- Secondary: assessment-architect (coding challenges)

## Key Patterns

### Docusaurus Code Tabs
```mdx
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
  <TabItem value="simulated" label="Simulated (Gazebo)" default>
  ```python
  # Code optimized for simulation
  ```
  </TabItem>
  <TabItem value="real" label="Real Robot (Jetson)">
  ```python
  # Code optimized for edge hardware
  ```
  </TabItem>
</Tabs>
```

### ROS 2 Patterns
- Publisher/Subscriber nodes
- Service/Action clients
- URDF robot descriptions
- Nav2 configurations
- Isaac ROS integration

## Constitution Reference
- Article IX (Skill System) - Canonical Skill #13
