"""
Authentication Router - JWT-based Auth with Email/Password

Implements user authentication endpoints including signup, signin,
user profile retrieval, and hardware onboarding quiz.

Note: Better-Auth Python SDK not available - using standard JWT approach.
"""

from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.responses import RedirectResponse
from sqlalchemy.orm import Session
from pydantic import BaseModel, EmailStr, validator
from typing import Optional
from datetime import datetime
import logging
import uuid
import os
import httpx
import json
import urllib.parse

from ..models.user import User, AuthProvider, HardwareProfile, ProgrammingLanguage
from ..models.database import get_db
from ..utils.auth_utils import create_access_token, verify_password, get_password_hash
from ..utils.auth_middleware import get_current_user

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/api/auth", tags=["authentication"])


# Request/Response Models
class SignupRequest(BaseModel):
    email: EmailStr
    password: str

    @validator('password')
    def validate_password(cls, v):
        if len(v) < 8:
            raise ValueError('Password must be at least 8 characters')
        return v


class SigninRequest(BaseModel):
    email: EmailStr
    password: str


class OnboardingRequest(BaseModel):
    hardware_profile: str
    programming_language: str

    @validator('hardware_profile')
    def validate_hardware_profile(cls, v):
        valid_profiles = ['rtx_4090', 'jetson_orin_nano']
        if v not in valid_profiles:
            raise ValueError(f'hardware_profile must be one of: {valid_profiles}')
        return v

    @validator('programming_language')
    def validate_programming_language(cls, v):
        valid_languages = ['python', 'cpp']
        if v not in valid_languages:
            raise ValueError(f'programming_language must be one of: {valid_languages}')
        return v


class AuthResponse(BaseModel):
    access_token: str
    token_type: str = "bearer"
    user: dict


class UserResponse(BaseModel):
    id: str
    email: str
    auth_provider: str
    hardware_profile: Optional[str]
    programming_language: Optional[str]
    created_at: datetime
    last_login: Optional[datetime]


@router.post("/signup", response_model=AuthResponse)
async def signup(request: SignupRequest, db: Session = Depends(get_db)):
    """
    User signup with email and password.

    Creates a new user account with auth_provider='email' and returns JWT token.

    Args:
        request: SignupRequest with email and password
        db: Database session

    Returns:
        AuthResponse with JWT access_token and user details

    Raises:
        HTTPException 400: If email already exists
    """
    # Check if user already exists
    existing_user = db.query(User).filter(User.email == request.email).first()
    if existing_user:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Email already registered"
        )

    # Hash password
    hashed_password = get_password_hash(request.password)

    # Create new user
    new_user = User(
        id=uuid.uuid4(),
        email=request.email,
        password_hash=hashed_password,
        auth_provider=AuthProvider.EMAIL,
        created_at=datetime.utcnow(),
        last_login=datetime.utcnow()
    )

    db.add(new_user)
    db.commit()
    db.refresh(new_user)

    logger.info(f"New user signed up: {new_user.email}")

    # Generate JWT token
    access_token = create_access_token(data={"sub": str(new_user.id)})

    return AuthResponse(
        access_token=access_token,
        user={
            "id": str(new_user.id),
            "email": new_user.email,
            "auth_provider": new_user.auth_provider.value,
            "hardware_profile": new_user.hardware_profile.value if new_user.hardware_profile else None,
            "programming_language": new_user.programming_language.value if new_user.programming_language else None,
        }
    )


@router.post("/signin", response_model=AuthResponse)
async def signin(request: SigninRequest, db: Session = Depends(get_db)):
    """
    User signin with email and password.

    Validates credentials and returns JWT token.

    Args:
        request: SigninRequest with email and password
        db: Database session

    Returns:
        AuthResponse with JWT access_token and user details

    Raises:
        HTTPException 401: If credentials are invalid
    """
    # Find user by email
    user = db.query(User).filter(User.email == request.email).first()

    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid email or password"
        )

    # Verify password
    if not verify_password(request.password, user.password_hash):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid email or password"
        )

    # Update last_login
    user.last_login = datetime.utcnow()
    db.commit()

    logger.info(f"User signed in: {user.email}")

    # Generate JWT token
    access_token = create_access_token(data={"sub": str(user.id)})

    return AuthResponse(
        access_token=access_token,
        user={
            "id": str(user.id),
            "email": user.email,
            "auth_provider": user.auth_provider.value,
            "hardware_profile": user.hardware_profile.value if user.hardware_profile else None,
            "programming_language": user.programming_language.value if user.programming_language else None,
        }
    )


@router.get("/user", response_model=UserResponse)
async def get_user(current_user: User = Depends(get_current_user)):
    """
    Get current authenticated user profile.

    Protected endpoint requiring valid JWT token.

    Args:
        current_user: User from JWT token (injected by middleware)

    Returns:
        UserResponse with full user profile

    Example:
        GET /api/auth/user
        Headers: Authorization: Bearer <token>
    """
    return UserResponse(
        id=str(current_user.id),
        email=current_user.email,
        auth_provider=current_user.auth_provider.value,
        hardware_profile=current_user.hardware_profile.value if current_user.hardware_profile else None,
        programming_language=current_user.programming_language.value if current_user.programming_language else None,
        created_at=current_user.created_at,
        last_login=current_user.last_login
    )


@router.post("/onboarding")
async def submit_onboarding(
    request: OnboardingRequest,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Submit hardware onboarding quiz.

    Updates user's hardware_profile and programming_language preferences.
    Protected endpoint requiring authentication.

    Args:
        request: OnboardingRequest with hardware and language preferences
        current_user: Authenticated user from JWT
        db: Database session

    Returns:
        Success message with updated profile

    Example:
        POST /api/auth/onboarding
        Headers: Authorization: Bearer <token>
        Body: {
            "hardware_profile": "rtx_4090",
            "programming_language": "python"
        }
    """
    # Update user profile
    current_user.hardware_profile = HardwareProfile(request.hardware_profile)
    current_user.programming_language = ProgrammingLanguage(request.programming_language)
    current_user.last_login = datetime.utcnow()

    db.commit()
    db.refresh(current_user)

    logger.info(
        f"Onboarding completed for user {current_user.email}: "
        f"hardware={request.hardware_profile}, lang={request.programming_language}"
    )

    return {
        "message": "Onboarding completed successfully",
        "user": {
            "id": str(current_user.id),
            "email": current_user.email,
            "hardware_profile": current_user.hardware_profile.value,
            "programming_language": current_user.programming_language.value,
        }
    }


@router.get("/github/login")
async def github_login():
    GITHUB_CLIENT_ID = os.getenv("BETTER_AUTH_GITHUB_CLIENT_ID")
    if not GITHUB_CLIENT_ID:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="GitHub OAuth not configured"
        )

    callback_url = f"{os.getenv('API_URL', 'http://localhost:8000')}/api/auth/github/callback"
    github_auth_url = (
        f"https://github.com/login/oauth/authorize"
        f"?client_id={GITHUB_CLIENT_ID}"
        f"&redirect_uri={callback_url}"
        f"&scope=user:email"
    )

    return RedirectResponse(url=github_auth_url)


@router.get("/github/callback")
async def github_callback(code: str, db: Session = Depends(get_db)):
    GITHUB_CLIENT_ID = os.getenv("BETTER_AUTH_GITHUB_CLIENT_ID")
    GITHUB_CLIENT_SECRET = os.getenv("BETTER_AUTH_GITHUB_CLIENT_SECRET")

    if not GITHUB_CLIENT_ID or not GITHUB_CLIENT_SECRET:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="GitHub OAuth not configured"
        )

    try:
        async with httpx.AsyncClient() as client:
            token_response = await client.post(
                "https://github.com/login/oauth/access_token",
                data={
                    "client_id": GITHUB_CLIENT_ID,
                    "client_secret": GITHUB_CLIENT_SECRET,
                    "code": code,
                },
                headers={"Accept": "application/json"}
            )
            token_data = token_response.json()

            if "error" in token_data:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail=f"GitHub OAuth error: {token_data.get('error_description', 'Unknown error')}"
                )

            access_token = token_data.get("access_token")

            user_response = await client.get(
                "https://api.github.com/user",
                headers={
                    "Authorization": f"Bearer {access_token}",
                    "Accept": "application/json"
                }
            )
            github_user = user_response.json()

            email_response = await client.get(
                "https://api.github.com/user/emails",
                headers={
                    "Authorization": f"Bearer {access_token}",
                    "Accept": "application/json"
                }
            )
            emails = email_response.json()
            primary_email = next((e["email"] for e in emails if e["primary"]), None)

            if not primary_email:
                primary_email = github_user.get("email")

            if not primary_email:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail="No email found in GitHub account"
                )

            existing_user = db.query(User).filter(User.email == primary_email).first()

            if existing_user:
                existing_user.last_login = datetime.utcnow()
                db.commit()
                db.refresh(existing_user)
                user = existing_user
            else:
                new_user = User(
                    id=uuid.uuid4(),
                    email=primary_email,
                    auth_provider=AuthProvider.GITHUB,
                    created_at=datetime.utcnow(),
                    last_login=datetime.utcnow()
                )
                db.add(new_user)
                db.commit()
                db.refresh(new_user)
                user = new_user

            jwt_token = create_access_token(data={"sub": str(user.id)})

            user_data = {
                "id": str(user.id),
                "email": user.email,
                "auth_provider": user.auth_provider.value,
                "hardware_profile": user.hardware_profile.value if user.hardware_profile else None,
                "programming_language": user.programming_language.value if user.programming_language else None,
            }

            user_json = urllib.parse.quote(json.dumps(user_data))

            frontend_url = os.getenv("FRONTEND_URL", "http://localhost:3000")
            redirect_url = f"{frontend_url}/auth/callback?token={jwt_token}&user={user_json}"

            return RedirectResponse(url=redirect_url)

    except Exception as e:
        logger.error(f"GitHub OAuth error: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"GitHub authentication failed: {str(e)}"
        )
