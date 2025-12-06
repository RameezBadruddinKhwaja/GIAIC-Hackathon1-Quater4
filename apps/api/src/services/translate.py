"""
Translation Service - Urdu Localization with Code Preservation

Translates educational content to Roman/Formal Urdu using Gemini 2.5 Flash.
Preserves code blocks, Mermaid diagrams, and technical terminology.
Implements 7-day caching.
"""

import os
import re
import logging
from datetime import datetime, timedelta
from typing import Optional, Dict, Any, List, Tuple
from sqlalchemy.orm import Session

from .gemini_client import get_gemini_client, CHAT_MODEL
from ..models.user import User
from ..models.translated_content import TranslatedContent

logger = logging.getLogger(__name__)

# Cache TTL: 7 days
CACHE_TTL_DAYS = 7


def get_translation_prompt(target_language: str) -> str:
    """
    Generate system prompt for Urdu translation.

    Args:
        target_language: 'roman_urdu' or 'formal_urdu'

    Returns:
        System prompt string for Gemini
    """
    if target_language == "roman_urdu":
        return """You are an expert Urdu translator for technical education content.

TASK: Translate technical prose to ROMAN URDU (Urdu written in Latin script).

TRANSLATION RULES:
1. Prose Translation:
   - Translate ALL explanatory text to Roman Urdu
   - Use simple, clear Roman Urdu
   - Maintain technical accuracy

2. DO NOT TRANSLATE:
   - Code blocks (```python, ```cpp, etc.)
   - Variable names, function names, class names
   - Mermaid diagram syntax
   - Technical terms: ROS 2, URDF, Isaac Sim, Gazebo, NavStack, SLAM
   - File paths, URLs, command names
   - Markdown syntax characters (##, **, [], etc.)

3. PRESERVE:
   - All markdown formatting
   - All code blocks EXACTLY as-is
   - All Mermaid diagrams
   - All headings structure
   - All links and images

4. Technical Term Handling:
   - Keep English: "ROS 2", "node", "topic", "publisher", "subscriber"
   - Translate context: "ROS 2 ek middleware framework hai..."

EXAMPLE:
Input: "ROS 2 is a middleware framework for robotics."
Output: "ROS 2 ek middleware framework hai jo robotics ke liye use hota hai."

OUTPUT: Return full markdown with prose translated to Roman Urdu, code/diagrams preserved."""

    else:  # formal_urdu
        return """You are an expert Urdu translator for technical education content.

TASK: Translate technical prose to FORMAL URDU (اردو written in Urdu script).

TRANSLATION RULES:
1. Prose Translation:
   - Translate ALL explanatory text to Formal Urdu (Urdu script)
   - Use academic/formal tone
   - Maintain technical accuracy

2. DO NOT TRANSLATE:
   - Code blocks (```python, ```cpp, etc.)
   - Variable names, function names, class names
   - Mermaid diagram syntax
   - Technical terms: ROS 2, URDF, Isaac Sim, Gazebo, NavStack, SLAM
   - File paths, URLs, command names
   - Markdown syntax characters (##, **, [], etc.)

3. PRESERVE:
   - All markdown formatting
   - All code blocks EXACTLY as-is
   - All Mermaid diagrams
   - All headings structure
   - All links and images

4. Technical Term Handling:
   - Keep English terms in parentheses after Urdu
   - Example: "روبوٹکس (Robotics) کے لیے"

OUTPUT: Return full markdown with prose in Formal Urdu, code/diagrams preserved."""


def extract_code_blocks(content: str) -> Tuple[str, List[str]]:
    """
    Extract code blocks and replace with placeholders.

    Args:
        content: Original markdown content

    Returns:
        Tuple of (content_with_placeholders, code_blocks_list)
    """
    code_blocks = []
    placeholder_pattern = "<<<CODE_BLOCK_{index}>>>"

    # Regex to match fenced code blocks
    code_block_pattern = r'```[\s\S]*?```'

    def replace_with_placeholder(match):
        code_blocks.append(match.group(0))
        return placeholder_pattern.format(index=len(code_blocks) - 1)

    content_with_placeholders = re.sub(
        code_block_pattern,
        replace_with_placeholder,
        content
    )

    logger.info(f"Extracted {len(code_blocks)} code blocks for preservation")
    return content_with_placeholders, code_blocks


def restore_code_blocks(content: str, code_blocks: List[str]) -> str:
    """
    Restore code blocks from placeholders.

    Args:
        content: Translated content with placeholders
        code_blocks: List of original code blocks

    Returns:
        Content with restored code blocks
    """
    for index, code_block in enumerate(code_blocks):
        placeholder = f"<<<CODE_BLOCK_{index}>>>"
        content = content.replace(placeholder, code_block)

    logger.info(f"Restored {len(code_blocks)} code blocks")
    return content


def check_cache(
    db: Session,
    user_id: str,
    chapter_id: str,
    target_language: str
) -> Optional[TranslatedContent]:
    """
    Check if translated content exists in cache (within 7-day TTL).

    Args:
        db: Database session
        user_id: User UUID
        chapter_id: Chapter identifier
        target_language: Target language

    Returns:
        TranslatedContent if cache hit, None if cache miss
    """
    cache_expiry = datetime.utcnow() - timedelta(days=CACHE_TTL_DAYS)

    cached = db.query(TranslatedContent).filter(
        TranslatedContent.user_id == user_id,
        TranslatedContent.chapter_id == chapter_id,
        TranslatedContent.target_language == target_language,
        TranslatedContent.created_at > cache_expiry
    ).first()

    if cached:
        logger.info(f"Translation cache HIT: user={user_id}, chapter={chapter_id}, lang={target_language}")
        return cached
    else:
        logger.info(f"Translation cache MISS: user={user_id}, chapter={chapter_id}, lang={target_language}")
        return None


def fetch_original_content(chapter_id: str) -> str:
    """
    Fetch original markdown content from Docusaurus docs directory.

    Args:
        chapter_id: Chapter identifier

    Returns:
        Original markdown content

    Raises:
        FileNotFoundError: If chapter file doesn't exist
    """
    current_dir = os.path.dirname(__file__)
    docs_dir = os.path.abspath(os.path.join(current_dir, '../../../docs/docs'))
    file_path = os.path.join(docs_dir, f"{chapter_id}.md")

    if not os.path.exists(file_path):
        logger.error(f"Chapter file not found: {file_path}")
        raise FileNotFoundError(f"Chapter not found: {chapter_id}")

    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    logger.info(f"Fetched original content for translation: {chapter_id}")
    return content


def translate_with_gemini(
    content: str,
    target_language: str
) -> str:
    """
    Translate content using Gemini 2.5 Flash.

    Args:
        content: Content with code blocks replaced by placeholders
        target_language: 'roman_urdu' or 'formal_urdu'

    Returns:
        Translated content (with placeholders still in place)
    """
    client = get_gemini_client()
    system_prompt = get_translation_prompt(target_language)

    user_message = f"""Original Content:

{content}

Please translate according to the rules."""

    try:
        response = client.chat.completions.create(
            model=CHAT_MODEL,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_message}
            ],
            temperature=0.4,  # Slightly higher for natural translation
            max_tokens=4000
        )

        translated_content = response.choices[0].message.content
        logger.info(f"Successfully translated content to {target_language}")
        return translated_content

    except Exception as e:
        logger.error(f"Gemini translation failed: {e}")
        raise


def save_to_cache(
    db: Session,
    user_id: str,
    chapter_id: str,
    target_language: str,
    translated_mdx: str
) -> TranslatedContent:
    """
    Save translated content to database cache.

    Args:
        db: Database session
        user_id: User UUID
        chapter_id: Chapter identifier
        target_language: Target language
        translated_mdx: Translated markdown content

    Returns:
        Created TranslatedContent record
    """
    translated = TranslatedContent(
        user_id=user_id,
        chapter_id=chapter_id,
        target_language=target_language,
        translated_mdx=translated_mdx,
        created_at=datetime.utcnow()
    )

    db.add(translated)
    db.commit()
    db.refresh(translated)

    logger.info(f"Saved translation to cache: user={user_id}, chapter={chapter_id}, lang={target_language}")
    return translated


def translate_chapter(
    db: Session,
    user: User,
    chapter_id: str,
    target_language: str
) -> Dict[str, Any]:
    """
    Main translation function: cache lookup -> extract code -> translate -> restore code -> cache.

    Args:
        db: Database session
        user: Authenticated user
        chapter_id: Chapter identifier
        target_language: 'roman_urdu' or 'formal_urdu'

    Returns:
        Dict with translated_mdx, cache_hit, and target_language

    Raises:
        ValueError: If target_language is invalid
        FileNotFoundError: If chapter doesn't exist
    """
    # Validate target language
    valid_languages = ['roman_urdu', 'formal_urdu']
    if target_language not in valid_languages:
        raise ValueError(f"target_language must be one of: {valid_languages}")

    # Step 1: Check cache
    cached = check_cache(db, str(user.id), chapter_id, target_language)
    if cached:
        return {
            "translated_mdx": cached.translated_mdx,
            "cache_hit": True,
            "target_language": target_language
        }

    # Step 2: Fetch original content
    original_content = fetch_original_content(chapter_id)

    # Step 3: Extract code blocks
    content_with_placeholders, code_blocks = extract_code_blocks(original_content)

    # Step 4: Translate with Gemini
    translated_with_placeholders = translate_with_gemini(content_with_placeholders, target_language)

    # Step 5: Restore code blocks
    translated_mdx = restore_code_blocks(translated_with_placeholders, code_blocks)

    # Step 6: Save to cache
    save_to_cache(db, str(user.id), chapter_id, target_language, translated_mdx)

    return {
        "translated_mdx": translated_mdx,
        "cache_hit": False,
        "target_language": target_language
    }
