# technical-clarity Skill

## Purpose
Ensure technical accuracy and clarity in all technical explanations and code examples.

## Key Patterns

### Technical Accuracy Guidelines
1. **Version Specificity**: Always specify exact versions (ROS 2 Humble, Ubuntu 22.04)
2. **Code Testing**: All code examples must be tested and executable
3. **Error Handling**: Include common errors and troubleshooting
4. **Platform Specificity**: Clarify Ubuntu vs Docker vs Jetson differences

### Clarity Standards
- Use active voice: "The node publishes messages" not "Messages are published"
- Define acronyms on first use: "DDS (Data Distribution Service)"
- Provide context before code: Explain *why* before *how*
- Use consistent terminology throughout

### Example: Clear vs Unclear

**Unclear**:
```markdown
Install ROS 2 and run the demo.
```

**Clear**:
```markdown
Install ROS 2 Humble on Ubuntu 22.04 using the following commands. This will take approximately 10 minutes and requires 2GB of disk space.

```bash
# Add ROS 2 repository
sudo apt update
sudo apt install ros-humble-desktop

# Verify installation
ros2 --version
# Expected output: ros2 cli version 0.18.5
```

### Common Pitfalls to Avoid
- ❌ Assuming prior knowledge without stating prerequisites
- ❌ Using jargon without explanation
- ❌ Providing code without context
- ❌ Mixing Python 2 and Python 3 syntax
- ❌ Omitting import statements

### Validation Checklist
- [ ] All commands include expected output
- [ ] All code examples are complete and runnable
- [ ] Prerequisites are explicitly stated
- [ ] Version numbers are specified
- [ ] Common errors are documented

## Usage Context
- Reviewing all technical content
- Validating code examples
- Technical writing guidelines
- Quality assurance
