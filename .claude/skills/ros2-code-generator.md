# ros2-code-generator Skill (Bonus)

## Purpose
Generate ROS 2 Python/C++ code from natural language descriptions.

## Key Patterns

### Code Generation Pipeline
```python
# apps/api/src/services/ros2_codegen.py
from openai import OpenAI
import os

class ROS2CodeGenerator:
    def __init__(self):
        self.client = OpenAI(
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
            api_key=os.getenv("GEMINI_API_KEY")
        )

    async def generate_code(
        self,
        description: str,
        language: str = "python",
        node_type: str = "publisher"
    ) -> dict:
        """Generate ROS 2 code from natural language"""

        system_prompt = f"""You are a ROS 2 code generation expert. Generate {language} code for {node_type} nodes.

Rules:
- Use ROS 2 Humble conventions
- Include proper imports
- Add docstrings and comments
- Handle errors appropriately
- Follow PEP 8 (Python) or Google C++ Style Guide
"""

        user_prompt = f"""Generate a ROS 2 {node_type} node in {language}:

{description}

Requirements:
- Complete, runnable code
- Include package.json/CMakeLists.txt snippets if needed
- Add usage instructions
"""

        response = self.client.chat.completions.create(
            model="gemini-2.5-flash",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.3
        )

        code = response.choices[0].message.content
        return {
            "code": code,
            "language": language,
            "node_type": node_type
        }
```

### Frontend Component
```typescript
// apps/docs/src/components/ROS2Playground.tsx
import React from 'react';
import Editor from '@monaco-editor/react';

export const ROS2Playground: React.FC = () => {
  const [description, setDescription] = React.useState('');
  const [code, setCode] = React.useState('');
  const [language, setLanguage] = React.useState<'python' | 'cpp'>('python');
  const [loading, setLoading] = React.useState(false);

  const generateCode = async () => {
    setLoading(true);
    try {
      const response = await fetch('/api/codegen/ros2', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ description, language }),
      });
      const data = await response.json();
      setCode(data.code);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="ros2-playground">
      <textarea
        placeholder="Describe the ROS 2 node you want to create..."
        value={description}
        onChange={(e) => setDescription(e.target.value)}
      />
      <select value={language} onChange={(e) => setLanguage(e.target.value as any)}>
        <option value="python">Python</option>
        <option value="cpp">C++</option>
      </select>
      <button onClick={generateCode} disabled={loading}>
        {loading ? 'Generating...' : 'Generate Code'}
      </button>

      <Editor
        height="400px"
        language={language}
        value={code}
        theme="vs-dark"
        options={{ readOnly: true }}
      />
    </div>
  );
};
```

### Example Prompts
- "Create a publisher that sends joint states at 50 Hz"
- "Generate a service server that computes inverse kinematics"
- "Build an action server for navigation to goal poses"

## Usage Context
- Code learning aid
- Prototyping tool
- Template generation
- Educational demonstrations
