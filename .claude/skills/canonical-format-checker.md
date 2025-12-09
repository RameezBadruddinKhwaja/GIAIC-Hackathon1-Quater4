# canonical-format-checker Skill

## Purpose
Validate markdown format, frontmatter, and content structure compliance.

## Key Patterns

### Frontmatter Validation
```yaml
---
sidebar_position: 1  # Required
title: Week 1 - ROS 2 Basics  # Required
description: Introduction to ROS 2  # Required
keywords: [ros2, robotics, dds]  # Required (min 3)
---
```

### Content Structure Requirements
1. **H1 Title** (matches frontmatter title)
2. **Learning Objectives** section with checkboxes
3. **Introduction** with concept overview
4. **Code Examples** with proper syntax highlighting
5. **Key Takeaways** section
6. **Next Steps** or **Resources** section

### Format Rules
- Use `##` for main sections, `###` for subsections
- Code blocks must specify language: ```python, ```cpp, ```bash
- Images require alt text: `![Alt text](path)`
- Links must be relative for internal, absolute for external
- Lists use `-` for unordered, `1.` for ordered

### Validation Checklist
```markdown
- [ ] Frontmatter present and complete
- [ ] Title hierarchy correct (H1 → H2 → H3)
- [ ] All code blocks have language specified
- [ ] All images have alt text
- [ ] Learning objectives use checkbox format
- [ ] No broken internal links
- [ ] Proper Docusaurus component imports
```

## Usage Context
- Pre-commit validation
- Content review process
- CI/CD pipeline checks
- Quality gates
