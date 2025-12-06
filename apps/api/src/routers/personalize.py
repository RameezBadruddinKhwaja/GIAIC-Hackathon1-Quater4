"""
Personalization Router - Hardware-Aware Content Personalization

Implements /api/personalize endpoint for personalizing educational content
based on user's hardware profile (RTX 4090 vs Jetson Orin Nano).
"""

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from pydantic import BaseModel
from typing import Optional
import logging

from ..models.user import User
from ..models.audit_log import AuditLog
from ..models.database import get_db
from ..utils.auth_middleware import get_current_user
from ..services.personalize import personalize_chapter

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/api", tags=["personalization"])


# Request/Response Models
class PersonalizeRequest(BaseModel):
    chapter_id: str


class PersonalizeResponse(BaseModel):
    personalized_mdx: str
    cache_hit: bool
    hardware_profile: str


@router.post("/personalize", response_model=PersonalizeResponse)
async def personalize_content(
    request: PersonalizeRequest,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Personalize chapter content based on user's hardware profile.

    Protected endpoint requiring authentication.
    Uses Gemini 2.5 Flash for content rewriting with 7-day caching.

    Args:
        request: PersonalizeRequest with chapter_id
        current_user: Authenticated user from JWT
        db: Database session

    Returns:
        PersonalizeResponse with personalized markdown, cache status, and hardware profile

    Raises:
        HTTPException 400: If user hasn't completed onboarding
        HTTPException 404: If chapter not found
        HTTPException 500: If personalization fails

    Example:
        POST /api/personalize
        Headers: Authorization: Bearer <token>
        Body: {
            "chapter_id": "module-1/ros2-fundamentals"
        }

        Response: {
            "personalized_mdx": "# ROS 2 Fundamentals\\n\\n...",
            "cache_hit": false,
            "hardware_profile": "rtx_4090"
        }
    """
    try:
        # Validate user has completed onboarding
        if not current_user.hardware_profile:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Please complete onboarding to set your hardware profile"
            )

        # Call personalization service
        result = personalize_chapter(
            db=db,
            user=current_user,
            chapter_id=request.chapter_id
        )

        # Audit logging (FR-018 requirement)
        audit_log = AuditLog(
            user_id=current_user.id,
            event_type="personalization",
            event_details={
                "chapter_id": request.chapter_id,
                "hardware_profile": result["hardware_profile"],
                "cache_hit": result["cache_hit"],
                "timestamp": str(datetime.utcnow())
            },
            ip_address=None,  # TODO: Extract from request if needed
            created_at=datetime.utcnow()
        )
        db.add(audit_log)
        db.commit()

        logger.info(
            f"Personalization successful: user={current_user.email}, "
            f"chapter={request.chapter_id}, cache_hit={result['cache_hit']}"
        )

        return PersonalizeResponse(
            personalized_mdx=result["personalized_mdx"],
            cache_hit=result["cache_hit"],
            hardware_profile=result["hardware_profile"]
        )

    except ValueError as e:
        # User validation errors
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
        logger.error(f"Personalization failed: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to personalize content"
        )


# Import datetime for audit logging
from datetime import datetime
