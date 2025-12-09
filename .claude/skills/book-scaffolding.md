# book-scaffolding Skill

## Purpose
Generate complete Docusaurus project structure with proper sidebars, frontmatter, and navigation hierarchy.

## Key Patterns

### Docusaurus Project Structure
```
apps/docs/
├── docs/
│   ├── week-01-ros2-basics/
│   │   ├── index.md
│   │   ├── quiz.md
│   │   └── challenge.md
│   ├── week-02-nodes-topics/
│   └── ...
├── sidebars.js
├── docusaurus.config.ts
└── package.json
```

### Sidebars Configuration
```javascript
module.exports = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Part 1: The Nervous System (ROS 2)',
      items: [
        'week-01-ros2-basics/index',
        'week-02-nodes-topics/index',
        'week-03-urdf-modeling/index',
        'week-04-services-actions/index',
        'week-05-nav2/index',
      ],
    },
    {
      type: 'category',
      label: 'Part 2: The Digital Twin (Simulation)',
      items: [
        'week-06-gazebo-sim/index',
        'week-07-unity-sim/index',
      ],
    },
    // ... more parts
  ],
};
```

### MDX Frontmatter Pattern
```markdown
---
sidebar_position: 1
title: Week 1 - ROS 2 Basics
description: Introduction to ROS 2 fundamentals
keywords: [ros2, robotics, middleware, dds]
---
```

### Navigation Hierarchy
- Use `sidebar_position` to control order
- Group related content in directories
- Use `index.md` as entry point for each week
- Add quiz.md and challenge.md for assessments

## Usage Context
- Initial Docusaurus setup
- Creating new curriculum sections
- Reorganizing content structure
- Adding new weeks or modules
