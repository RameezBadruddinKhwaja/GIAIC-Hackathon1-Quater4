# Matrix Loader Skill

**Description**: Dynamic skill loading logic and keyword mapping for context-aware responses

**Purpose**:
Enable the RAG chatbot to dynamically load relevant skills based on user query keywords, enhancing response quality with specialized knowledge.

**Keyword Mapping**:

| Keyword | Skill to Load | Reason |
|---------|---------------|--------|
| SLAM, Nav2, Navigation | ros2-mastery | Navigation/mapping concepts |
| Jetson, Orin, edge | edge-computing | Edge deployment optimization |
| Isaac, Isaac Sim, Omniverse | nvidia-isaac | NVIDIA Isaac ecosystem |
| Gazebo, simulation, physics | simulation-expert | Robot simulation |
| VLA, vision-language | vision-language-action | Vision-Language-Action models |
| Docusaurus, MDX, admonition | docusaurus-guru | Documentation platform |
| Better-Auth, OAuth, JWT | better-auth | Authentication implementation |
| ROS 2, node, topic | ros2-mastery | ROS 2 fundamentals |

**Implementation Logic**:

```python
# apps/api/src/services/matrix_loader.py
import os
from typing import List, Optional

def load_skill(skill_name: str) -> Optional[str]:
    """
    Load skill content from .claude/skills/{skill_name}.md

    Args:
        skill_name: Name of skill file (without .md extension)

    Returns:
        Skill content as string, or None if file not found
    """
    skill_path = f".claude/skills/{skill_name}.md"
    if os.path.exists(skill_path):
        with open(skill_path, 'r') as f:
            return f.read()
    return None

def detect_keywords(query: str) -> List[str]:
    """
    Detect keywords in user query and map to skills

    Args:
        query: User's query text

    Returns:
        List of skill names to load
    """
    query_lower = query.lower()
    skills_to_load = []

    keyword_map = {
        "slam": "ros2-mastery",
        "jetson": "edge-computing",
        "isaac": "nvidia-isaac",
        "gazebo": "simulation-expert",
        "vla": "vision-language-action",
        "docusaurus": "docusaurus-guru",
        "better-auth": "better-auth",
        "oauth": "better-auth",
        "ros 2": "ros2-mastery",
        "ros2": "ros2-mastery",
        "navigation": "ros2-mastery",
    }

    for keyword, skill in keyword_map.items():
        if keyword in query_lower and skill not in skills_to_load:
            skills_to_load.append(skill)

    return skills_to_load

def load_skills_for_query(query: str) -> dict:
    """
    Load all relevant skills for a given query

    Args:
        query: User's query text

    Returns:
        Dict with loaded skills: {skill_name: skill_content}
    """
    skill_names = detect_keywords(query)
    loaded_skills = {}

    for skill_name in skill_names:
        skill_content = load_skill(skill_name)
        if skill_content:
            loaded_skills[skill_name] = skill_content

    return loaded_skills
```

**Integration with RAG Pipeline**:

```python
# apps/api/src/services/rag_pipeline.py
from services.matrix_loader import load_skills_for_query
import logging

logger = logging.getLogger(__name__)

def chat_with_rag(query: str, hardware_context: Optional[str] = None):
    # 1. Load relevant skills
    loaded_skills = load_skills_for_query(query)

    # 2. Log skill loading (FR-009 requirement)
    for skill_name in loaded_skills.keys():
        logger.info(f"Matrix Skill Loaded: {skill_name}")

    # 3. Build system prompt with skills
    system_prompt = "Answer using only provided book content. Always cite chapter sources.\n\n"

    if loaded_skills:
        system_prompt += "LOADED SKILLS:\n"
        for skill_name, skill_content in loaded_skills.items():
            system_prompt += f"\n--- {skill_name} ---\n{skill_content}\n"

    # 4. Search Qdrant for relevant chunks
    search_results = search_qdrant(query, hardware_context)

    # 5. Call Gemini with enriched context
    response = gemini_client.chat.completions.create(
        model="gemini-2.5-flash",
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": f"Context: {search_results}\n\nQuery: {query}"}
        ]
    )

    return response
```

**Audit Trail**:
- Every skill load MUST be logged: `logger.info(f"Matrix Skill Loaded: {skill_name}")`
- Store loaded skills in ChatLog JSONB field `skills_loaded`
- Example: `skills_loaded: ["ros2-mastery", "nvidia-isaac"]`

**Benefits**:
- Context-aware responses with specialized knowledge
- Reduced hallucination (specific knowledge loaded on-demand)
- Audit trail for debugging (which skills were used)
- Extensible (add new skills and keywords easily)

**Keyword Triggers**:
- **Load when implementing**: RAG pipeline, skill loading, keyword detection, chatbot enhancement

**Invocation**:
Use this skill when implementing the Matrix Protocol dynamic skill loading feature.
