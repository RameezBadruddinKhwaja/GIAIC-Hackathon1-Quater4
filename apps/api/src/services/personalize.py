"""
Personalization Service - Hardware-Aware Content Rewriting

Personalizes educational content based on user's hardware profile using Gemini 2.5 Flash.
Implements 7-day caching to reduce API calls.
"""

import os
import logging
from datetime import datetime, timedelta
from typing import Optional, Dict, Any
from sqlalchemy.orm import Session

from .gemini_client import get_gemini_client, CHAT_MODEL
from ..models.user import User, HardwareProfile
from ..models.personalized_content import PersonalizedContent

logger = logging.getLogger(__name__)

# Cache TTL: 7 days
CACHE_TTL_DAYS = 7


def get_personalization_prompt(hardware_profile: HardwareProfile) -> str:
    """
    Generate system prompt for content personalization based on hardware profile.

    Args:
        hardware_profile: User's hardware configuration

    Returns:
        System prompt string for Gemini
    """
    if hardware_profile == HardwareProfile.RTX_4090:
        return """You are a content personalization expert for Physical AI education.

TASK: Rewrite code examples and technical instructions for HIGH-PERFORMANCE SIMULATION.

HARDWARE CONTEXT: User has NVIDIA RTX 4090 GPU with Isaac Sim.

REWRITING RULES:
1. Code Examples:
   - Enable GPU acceleration in all simulation code
   - Use Isaac Sim high-fidelity rendering
   - Maximize batch sizes and parallel processing
   - Recommend RTX-specific optimizations (RTX Ray Tracing, DLSS)

2. Technical Instructions:
   - Emphasize simulation-first workflow
   - Reference Isaac Sim documentation
   - Suggest high-resolution sensor simulation
   - Recommend complex physics (PhysX GPU)

3. PRESERVE:
   - All prose text (unchanged)
   - All Mermaid diagrams
   - All markdown formatting
   - All headings and structure
   - Technical terminology (ROS 2, URDF, etc.)

4. DO NOT:
   - Add new sections
   - Remove existing content
   - Change the learning objectives
   - Translate any text

OUTPUT: Return the full markdown with ONLY code blocks and technical instructions modified for RTX 4090."""

    else:  # JETSON_ORIN_NANO
        return """You are a content personalization expert for Physical AI education.

TASK: Rewrite code examples and technical instructions for EDGE DEPLOYMENT.

HARDWARE CONTEXT: User has Jetson Orin Nano for real robot deployment.

REWRITING RULES:
1. Code Examples:
   - Optimize for low power consumption
   - Use CPU-based processing where possible
   - Reduce memory footprint
   - Add power profiling code (tegrastats, nvpmodel)
   - Use Jetson-specific libraries (jetson-utils, TensorRT)

2. Technical Instructions:
   - Emphasize edge optimization techniques
   - Reference Jetson documentation
   - Suggest lightweight models (quantization, pruning)
   - Recommend real-time performance monitoring

3. PRESERVE:
   - All prose text (unchanged)
   - All Mermaid diagrams
   - All markdown formatting
   - All headings and structure
   - Technical terminology (ROS 2, URDF, etc.)

4. DO NOT:
   - Add new sections
   - Remove existing content
   - Change the learning objectives
   - Translate any text

OUTPUT: Return the full markdown with ONLY code blocks and technical instructions modified for Jetson Orin Nano."""


def check_cache(
    db: Session,
    user_id: str,
    chapter_id: str
) -> Optional[PersonalizedContent]:
    """
    Check if personalized content exists in cache (within 7-day TTL).

    Args:
        db: Database session
        user_id: User UUID
        chapter_id: Chapter identifier

    Returns:
        PersonalizedContent if cache hit, None if cache miss
    """
    cache_expiry = datetime.utcnow() - timedelta(days=CACHE_TTL_DAYS)

    cached = db.query(PersonalizedContent).filter(
        PersonalizedContent.user_id == user_id,
        PersonalizedContent.chapter_id == chapter_id,
        PersonalizedContent.created_at > cache_expiry
    ).first()

    if cached:
        logger.info(f"Cache HIT for user={user_id}, chapter={chapter_id}")
        return cached
    else:
        logger.info(f"Cache MISS for user={user_id}, chapter={chapter_id}")
        return None


def fetch_original_content(chapter_id: str) -> str:
    """
    Fetch original markdown content from Docusaurus docs directory.

    Args:
        chapter_id: Chapter identifier (e.g., "intro", "week-01-ros2-basics/index", "module-1/ros2-fundamentals")

    Returns:
        Original markdown content

    Raises:
        FileNotFoundError: If chapter file doesn't exist
    """
    # From apps/api/src/services -> ../../docs/docs
    current_dir = os.path.dirname(__file__)
    src_dir = os.path.dirname(current_dir)  # apps/api/src
    api_dir = os.path.dirname(src_dir)  # apps/api
    apps_dir = os.path.dirname(api_dir)  # apps
    docs_dir = os.path.join(apps_dir, 'docs', 'docs')

    # Try multiple file path patterns
    possible_paths = [
        os.path.join(docs_dir, f"{chapter_id}.md"),  # e.g., intro.md, module-1/ros2-fundamentals.md
        os.path.join(docs_dir, f"{chapter_id}.mdx"),  # e.g., intro.mdx
        os.path.join(docs_dir, chapter_id, "index.md"),  # e.g., week-01-ros2-basics/index.md
        os.path.join(docs_dir, chapter_id, "index.mdx"),  # e.g., week-01-ros2-basics/index.mdx
    ]

    for file_path in possible_paths:
        if os.path.exists(file_path):
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
            logger.info(f"Fetched original content for personalization: {chapter_id} from {file_path}")
            return content

    # If no file found, log all attempts
    logger.error(f"Chapter file not found for '{chapter_id}'. Tried paths:")
    for path in possible_paths:
        logger.error(f"  - {path}")
    raise FileNotFoundError(f"Chapter not found: {chapter_id}. Tried {len(possible_paths)} possible paths.")


def personalize_with_gemini(
    content: str,
    hardware_profile: HardwareProfile
) -> str:
    """
    Personalize content using Gemini 2.5 Flash.

    Args:
        content: Original markdown content
        hardware_profile: User's hardware configuration

    Returns:
        Personalized markdown content
    """
    client = get_gemini_client()
    system_prompt = get_personalization_prompt(hardware_profile)

    user_message = f"""Original Content:

{content}

Please personalize this content according to the rules."""

    try:
        response = client.chat.completions.create(
            model=CHAT_MODEL,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_message}
            ],
            temperature=0.3,  # Lower temperature for consistent rewrites
            max_tokens=4000
        )

        personalized_content = response.choices[0].message.content
        logger.info(f"Successfully personalized content for {hardware_profile.value}")
        return personalized_content

    except Exception as e:
        logger.error(f"Gemini personalization failed: {e}")
        raise


def save_to_cache(
    db: Session,
    user_id: str,
    chapter_id: str,
    hardware_profile: HardwareProfile,
    personalized_mdx: str
) -> PersonalizedContent:
    """
    Save personalized content to database cache.

    Args:
        db: Database session
        user_id: User UUID
        chapter_id: Chapter identifier
        hardware_profile: User's hardware configuration
        personalized_mdx: Personalized markdown content

    Returns:
        Created PersonalizedContent record
    """
    personalized = PersonalizedContent(
        user_id=user_id,
        chapter_id=chapter_id,
        hardware_profile=hardware_profile,
        personalized_mdx=personalized_mdx,
        created_at=datetime.utcnow()
    )

    db.add(personalized)
    db.commit()
    db.refresh(personalized)

    logger.info(f"Saved personalized content to cache: user={user_id}, chapter={chapter_id}")
    return personalized


def personalize_chapter(
    db: Session,
    user: User,
    chapter_id: str
) -> Dict[str, Any]:
    """
    Main personalization function: cache lookup -> fetch -> personalize -> cache store.

    Args:
        db: Database session
        user: Authenticated user
        chapter_id: Chapter identifier

    Returns:
        Dict with personalized_mdx, cache_hit, and hardware_profile

    Raises:
        ValueError: If user has no hardware profile set
        FileNotFoundError: If chapter doesn't exist
    """
    # Validate user has hardware profile
    if not user.hardware_profile:
        raise ValueError("User must complete onboarding to set hardware profile")

    # Step 1: Check cache
    cached = check_cache(db, str(user.id), chapter_id)
    if cached:
        return {
            "personalized_mdx": cached.personalized_mdx,
            "cache_hit": True,
            "hardware_profile": user.hardware_profile.value
        }

    # Step 2: Fetch original content
    original_content = fetch_original_content(chapter_id)

    # Step 3: Personalize with Gemini
    personalized_mdx = personalize_with_gemini(original_content, user.hardware_profile)

    # Step 4: Save to cache
    save_to_cache(db, str(user.id), chapter_id, user.hardware_profile, personalized_mdx)

    return {
        "personalized_mdx": personalized_mdx,
        "cache_hit": False,
        "hardware_profile": user.hardware_profile.value
    }
