"""
Translation Router - Urdu Localization

Implements /api/translate endpoint for translating educational content to Urdu
while preserving code blocks and technical terminology.
"""

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from pydantic import BaseModel, validator
from typing import Optional
import logging
from datetime import datetime

from ..models.user import User
from ..models.audit_log import AuditLog
from ..models.database import get_db
from ..utils.auth_middleware import get_current_user
from ..services.translate import translate_chapter

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/api", tags=["localization"])


# Request/Response Models
class TranslateRequest(BaseModel):
    chapter_id: str
    target_language: str

    @validator('target_language')
    def validate_target_language(cls, v):
        valid_languages = ['roman_urdu', 'formal_urdu']
        if v not in valid_languages:
            raise ValueError(f'target_language must be one of: {valid_languages}')
        return v


class TranslateResponse(BaseModel):
    translated_mdx: str
    cache_hit: bool
    target_language: str


@router.post("/translate", response_model=TranslateResponse)
async def translate_content(
    request: TranslateRequest,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Translate chapter content to Urdu (Roman or Formal script).

    Protected endpoint requiring authentication.
    Uses Gemini 2.5 Flash for translation with code block preservation and 7-day caching.

    Args:
        request: TranslateRequest with chapter_id and target_language
        current_user: Authenticated user from JWT
        db: Database session

    Returns:
        TranslateResponse with translated markdown, cache status, and target language

    Raises:
        HTTPException 400: If target_language is invalid
        HTTPException 404: If chapter not found
        HTTPException 500: If translation fails

    Example:
        POST /api/translate
        Headers: Authorization: Bearer <token>
        Body: {
            "chapter_id": "module-1/ros2-fundamentals",
            "target_language": "roman_urdu"
        }

        Response: {
            "translated_mdx": "# ROS 2 Fundamentals\\n\\nROS 2 ek middleware...",
            "cache_hit": false,
            "target_language": "roman_urdu"
        }
    """
    try:
        # Convert target_language to lowercase for ENUM compatibility
        target_language = request.target_language.lower()

        # Call translation service
        result = translate_chapter(
            db=db,
            user=current_user,
            chapter_id=request.chapter_id,
            target_language=target_language
        )

        # Audit logging (FR-024 requirement)
        audit_log = AuditLog(
            user_id=current_user.id,
            event_type="translation",
            event_details={
                "chapter_id": request.chapter_id,
                "target_language": request.target_language,
                "cache_hit": result["cache_hit"],
                "timestamp": str(datetime.utcnow())
            },
            ip_address=None,  # TODO: Extract from request if needed
            created_at=datetime.utcnow()
        )
        db.add(audit_log)
        db.commit()

        logger.info(
            f"Translation successful: user={current_user.email}, "
            f"chapter={request.chapter_id}, lang={request.target_language}, "
            f"cache_hit={result['cache_hit']}"
        )

        return TranslateResponse(
            translated_mdx=result["translated_mdx"],
            cache_hit=result["cache_hit"],
            target_language=result["target_language"]
        )

    except ValueError as e:
        # Validation errors
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )

    except FileNotFoundError as e:
        # Chapter not found
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=str(e)
        )

    except Exception as e:
        # Unexpected errors
        logger.error(f"Translation failed: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to translate content"
        )
