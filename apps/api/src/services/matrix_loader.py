"""
Matrix Protocol - Dynamic Skill Loader

Loads specialized knowledge (skills) based on query keywords.
"""

import os
from typing import List, Dict, Optional
import logging

logger = logging.getLogger(__name__)

# Keyword to skill mapping
KEYWORD_SKILL_MAP = {
    "ros 2": "ros2-mastery",
    "ros2": "ros2-mastery",
    "slam": "ros2-mastery",
    "navigation": "ros2-mastery",
    "nav2": "ros2-mastery",
    "jetson": "edge-computing",
    "orin": "edge-computing",
    "edge": "edge-computing",
    "isaac": "nvidia-isaac",
    "isaac sim": "nvidia-isaac",
    "gazebo": "simulation-expert",
    "simulation": "simulation-expert",
    "vla": "vision-language-action",
    "vision-language": "vision-language-action",
    "docusaurus": "docusaurus-guru",
    "mermaid": "docusaurus-guru",
    "better-auth": "better-auth",
    "oauth": "better-auth",
    "authentication": "better-auth",
}

def load_skill(skill_name: str) -> Optional[str]:
    """
    Load skill content from .claude/skills/{skill_name}.md

    Args:
        skill_name: Name of skill file (without .md extension)

    Returns:
        Skill content as string, or None if file not found
    """
    # Navigate up to repo root
    current_dir = os.path.dirname(__file__)
    repo_root = os.path.abspath(os.path.join(current_dir, '../../..'))
    skill_path = os.path.join(repo_root, '.claude', 'skills', f'{skill_name}.md')

    if os.path.exists(skill_path):
        try:
            with open(skill_path, 'r', encoding='utf-8') as f:
                return f.read()
        except Exception as e:
            logger.error(f"Error loading skill {skill_name}: {e}")
            return None
    else:
        logger.warning(f"Skill file not found: {skill_path}")
        return None

def detect_keywords(query: str) -> List[str]:
    """
    Detect keywords in user query and map to skills.

    Args:
        query: User's query text

    Returns:
        List of unique skill names to load
    """
    query_lower = query.lower()
    skills_to_load = []

    for keyword, skill in KEYWORD_SKILL_MAP.items():
        if keyword in query_lower and skill not in skills_to_load:
            skills_to_load.append(skill)

    return skills_to_load

def load_skills_for_query(query: str) -> Dict[str, str]:
    """
    Load all relevant skills for a given query.

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
            # Log skill loading (FR-009 requirement)
            logger.info(f"Matrix Skill Loaded: {skill_name}")

    return loaded_skills
