# personalization-engine Skill (Bonus)

## Purpose
Hardware-aware content adaptation using Gemini for personalized learning experiences.

## Key Patterns

### Personalization API Endpoint
```python
# apps/api/src/routes/personalization.py
from fastapi import APIRouter, Depends
from openai import OpenAI
import os
from ..models.user import User
from ..models.personalized_content import PersonalizedContent
from ..auth import get_current_user

router = APIRouter()

@router.post("/personalize/{chapter_id}")
async def personalize_content(
    chapter_id: str,
    current_user: User = Depends(get_current_user)
):
    """Personalize chapter content based on user's hardware profile"""

    # Check cache (7-day TTL)
    cached = await PersonalizedContent.get(
        user_id=current_user.id,
        chapter_id=chapter_id
    )
    if cached and not cached.is_expired():
        return {"content": cached.personalized_mdx}

    # Read original content
    original_content = await read_chapter_content(chapter_id)

    # Generate personalized content
    client = OpenAI(
        base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
        api_key=os.getenv("GEMINI_API_KEY")
    )

    system_prompt = f"""You are a content personalization expert for robotics education.

User Hardware Profile: {current_user.hardware_profile}
Programming Language Preference: {current_user.programming_language}

Task: Adapt the content for this user's hardware setup:
- If RTX 4090: Emphasize simulation, high-compute algorithms, parallel processing
- If Jetson Orin Nano: Emphasize edge deployment, power optimization, real-time constraints

Preserve:
- Original structure and learning objectives
- All code blocks and their functionality
- Markdown formatting

Adapt:
- Code examples (add hardware-specific optimizations)
- Explanations (hardware-relevant context)
- Performance tips
"""

    response = client.chat.completions.create(
        model="gemini-2.5-flash",
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": f"Original content:\n\n{original_content}"}
        ],
        temperature=0.3
    )

    personalized_mdx = response.choices[0].message.content

    # Cache result
    await PersonalizedContent.create(
        user_id=current_user.id,
        chapter_id=chapter_id,
        hardware_profile=current_user.hardware_profile,
        personalized_mdx=personalized_mdx
    )

    return {"content": personalized_mdx}
```

### Frontend Component
```typescript
// apps/docs/src/components/PersonalizeButton.tsx
import React from 'react';

interface PersonalizeButtonProps {
  chapterId: string;
}

export const PersonalizeButton: React.FC<PersonalizeButtonProps> = ({ chapterId }) => {
  const [loading, setLoading] = React.useState(false);
  const [personalized, setPersonalized] = React.useState(false);

  const handlePersonalize = async () => {
    setLoading(true);
    try {
      const response = await fetch(`/api/personalize/${chapterId}`, {
        method: 'POST',
        credentials: 'include',
      });
      const data = await response.json();

      // Replace page content with personalized version
      document.querySelector('.markdown').innerHTML = data.content;
      setPersonalized(true);
    } finally {
      setLoading(false);
    }
  };

  return (
    <button
      className="personalize-button"
      onClick={handlePersonalize}
      disabled={loading || personalized}
    >
      {loading ? 'Personalizing...' : personalized ? 'Personalized ✓' : 'Personalize for My Hardware'}
    </button>
  );
};
```

### Adaptation Examples

**RTX 4090 Version**:
```python
# Optimized for simulation - use full compute power
import numpy as np

# Process multiple robots in parallel
robots = [create_robot(i) for i in range(10)]  # 10 parallel simulations
results = simulate_parallel(robots, use_gpu=True)
```

**Jetson Orin Nano Version**:
```python
# Optimized for edge - conserve power and memory
import numpy as np

# Single robot with power-efficient settings
robot = create_robot(power_mode='5W')  # Low power mode
result = simulate(robot, use_gpu=True, precision='fp16')  # Half precision
```

## Usage Context
- User-specific content
- Hardware-aware learning
- Performance optimization tips
- Deployment considerations
