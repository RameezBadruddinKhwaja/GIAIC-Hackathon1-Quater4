# skill-creator Skill (Meta-Skill)

## Purpose
Meta-skill for dynamically generating new skill definitions based on requirements.

## Key Patterns

### Skill Definition Template
```markdown
# {skill-name} Skill

## Purpose
{1-2 sentence description of what this skill does}

## Key Patterns

### {Pattern Category 1}
{Description and code examples}

### {Pattern Category 2}
{Description and code examples}

### {Pattern Category 3}
{Description and code examples}

## Usage Context
- {Context 1}
- {Context 2}
- {Context 3}
```

### Skill Naming Conventions
- Use kebab-case: `rag-chatbot-integrator`, not `RAGChatbotIntegrator`
- Be descriptive: `urdu-translator` not `translator`
- Action-oriented: `-generator`, `-builder`, `-designer`, `-auditor`

### Skill Categories
1. **Content Creation**: `book-scaffolding`, `code-example-generator`, `image-generator`
2. **Quality Assurance**: `validation-auditor`, `canonical-format-checker`, `technical-clarity`
3. **Architecture**: `mvp-builder`, `tool-selection-framework`, `chapter-planner`
4. **Assessment**: `quiz-generator`, `assessment-builder`, `exercise-designer`
5. **Integration**: `rag-chatbot-integrator`, `playwright-test-runner`, `docusaurus-deployer`
6. **Bonus Features**: `urdu-translator`, `personalization-engine`, `ros2-code-generator`

### When to Create New Skills
- **DO**: Create when pattern is reusable across multiple contexts
- **DO**: Create when complexity requires dedicated documentation
- **DON'T**: Create for one-time operations
- **DON'T**: Create if existing skill covers the use case

## Usage Context
- Extending skill library
- Custom workflow requirements
- Specialized domain needs
- Bonus feature integration
