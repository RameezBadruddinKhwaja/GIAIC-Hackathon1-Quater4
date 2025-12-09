# prompt-template-designer Skill

## Purpose
Design reusable agent prompts with clear instructions and expected outputs.

## Key Patterns

### Agent Prompt Template Structure
```markdown
# {Agent Name} Prompt Template

## Context
- **Input**: {Description of input data}
- **Output**: {Description of expected output}
- **Constraints**: {Any limitations or requirements}

## Instructions

1. {Step 1 with specific action}
2. {Step 2 with expected behavior}
3. {Step 3 with validation criteria}

## Example

**Input**:
```
{Sample input}
```

**Expected Output**:
```
{Sample output}
```

## Quality Criteria
- [ ] {Criterion 1}
- [ ] {Criterion 2}
- [ ] {Criterion 3}
```

### Content-Implementor Prompt Example
```markdown
# Content-Implementor: Week {N} Generation

## Context
- Input: Week number, topic, learning objectives from chapters-manifest.json
- Output: Complete MDX file with code examples, Mermaid diagrams, Docusaurus Tabs
- Constraints: Use Week 1 as template, maintain consistent structure

## Instructions

1. Read week-01-ros2-basics/index.md as structural template
2. Replace content with Week {N} topic while preserving:
   - Learning Objectives section with checkboxes
   - Mermaid diagrams (minimum 1 per week)
   - Docusaurus Tabs for hardware variations (RTX 4090 vs Jetson)
   - Code examples in Python AND C++
   - Hardware Considerations section
   - Key Takeaways with ✅ bullets
   - Resources section
3. Validate all code examples are runnable
4. Ensure frontmatter includes sidebar_position, title, description, keywords

## Quality Criteria
- [ ] 400+ lines of content
- [ ] Minimum 1 Mermaid diagram
- [ ] Python + C++ code examples
- [ ] Hardware tabs present
- [ ] No broken internal links
```

## Usage Context
- Agent definition files
- Workflow automation
- Content generation
- Quality assurance
