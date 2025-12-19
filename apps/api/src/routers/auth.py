"""
Authentication Router - Clean auth API endpoints

Handles signup, signin, signout, and user profile endpoints.
"""

import os
import httpx
from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.responses import RedirectResponse
from sqlalchemy.orm import Session
from sqlalchemy.exc import IntegrityError
from typing import Annotated

from src.core.database import get_db
from src.core.security import hash_password, verify_password, create_access_token
from src.core.deps import get_current_user
from src.models.user import User, ExperienceLevel
from src.schemas.auth import SignupRequest, SigninRequest, UserResponse, TokenResponse

router = APIRouter(prefix="/api/auth", tags=["auth"])


@router.post("/signup", response_model=TokenResponse, status_code=status.HTTP_201_CREATED)
async def signup(
    request: SignupRequest,
    db: Annotated[Session, Depends(get_db)]
):
    """
    Create a new user account.

    Args:
        request: Signup request with email, password, and experience levels
        db: Database session

    Returns:
        JWT access token

    Raises:
        409 Conflict: Email already registered
        400 Bad Request: Invalid input data
    """
    # Hash password
    password_hash = hash_password(request.password)

    # Create user
    new_user = User(
        email=request.email,
        password_hash=password_hash,
        software_level=request.software_level,
        hardware_level=request.hardware_level
    )

    try:
        db.add(new_user)
        db.commit()
        db.refresh(new_user)
    except IntegrityError:
        db.rollback()
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail="Email already registered"
        )

    # Create access token
    access_token = create_access_token(new_user.id)

    return TokenResponse(access_token=access_token)


@router.post("/signin", response_model=TokenResponse)
async def signin(
    request: SigninRequest,
    db: Annotated[Session, Depends(get_db)]
):
    """
    Sign in an existing user.

    Args:
        request: Signin request with email and password
        db: Database session

    Returns:
        JWT access token

    Raises:
        401 Unauthorized: Invalid credentials
    """
    # Find user by email
    user = db.query(User).filter(User.email == request.email).first()

    # Verify user exists and password is correct
    if not user or not verify_password(request.password, user.password_hash):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid email or password"
        )

    # Create access token
    access_token = create_access_token(user.id)

    return TokenResponse(access_token=access_token)


@router.post("/signout", status_code=status.HTTP_200_OK)
async def signout():
    """
    Sign out the current user.

    Note: Since we're using JWT tokens (stateless), signout is handled
    client-side by removing the token from storage. This endpoint exists
    for API consistency and future enhancements (e.g., token blacklisting).

    Returns:
        Success message
    """
    return {"message": "Successfully signed out"}


@router.get("/me", response_model=UserResponse)
async def get_current_user_profile(
    current_user: Annotated[User, Depends(get_current_user)]
):
    """
    Get current authenticated user's profile.

    Args:
        current_user: Current user from auth dependency

    Returns:
        User profile data (without password)
    """
    return current_user


# GitHub OAuth Configuration
GITHUB_CLIENT_ID = os.getenv("GITHUB_CLIENT_ID")
GITHUB_CLIENT_SECRET = os.getenv("GITHUB_CLIENT_SECRET")
GITHUB_REDIRECT_URI = os.getenv("GITHUB_REDIRECT_URI", "https://giaic-hackathon1-quater4.vercel.app/api/auth/github/callback")
FRONTEND_URL = os.getenv("FRONTEND_URL", "https://giaic-hackathon1-quater4.vercel.app")


@router.get("/github/login")
async def github_login():
    """
    Initiate GitHub OAuth flow.

    Redirects user to GitHub's OAuth authorization page.
    """
    if not GITHUB_CLIENT_ID:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="GitHub OAuth not configured"
        )

    github_auth_url = (
        f"https://github.com/login/oauth/authorize"
        f"?client_id={GITHUB_CLIENT_ID}"
        f"&redirect_uri={GITHUB_REDIRECT_URI}"
        f"&scope=read:user user:email"
    )

    return RedirectResponse(url=github_auth_url)


@router.get("/github/callback")
async def github_callback(
    code: str,
    db: Annotated[Session, Depends(get_db)]
):
    """
    Handle GitHub OAuth callback.

    Exchanges authorization code for access token, fetches user data,
    creates or finds user in database, and redirects to frontend with JWT.

    Args:
        code: Authorization code from GitHub
        db: Database session

    Returns:
        Redirect to frontend with JWT token in URL
    """
    if not GITHUB_CLIENT_ID or not GITHUB_CLIENT_SECRET:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="GitHub OAuth not configured"
        )

    async with httpx.AsyncClient() as client:
        # Exchange code for access token
        token_response = await client.post(
            "https://github.com/login/oauth/access_token",
            headers={"Accept": "application/json"},
            data={
                "client_id": GITHUB_CLIENT_ID,
                "client_secret": GITHUB_CLIENT_SECRET,
                "code": code,
                "redirect_uri": GITHUB_REDIRECT_URI,
            }
        )

        token_data = token_response.json()
        access_token = token_data.get("access_token")

        if not access_token:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Failed to get access token from GitHub"
            )

        # Get user data from GitHub
        user_response = await client.get(
            "https://api.github.com/user",
            headers={
                "Authorization": f"Bearer {access_token}",
                "Accept": "application/json"
            }
        )

        github_user = user_response.json()

        # Get user email (may not be public)
        email = github_user.get("email")
        if not email:
            emails_response = await client.get(
                "https://api.github.com/user/emails",
                headers={
                    "Authorization": f"Bearer {access_token}",
                    "Accept": "application/json"
                }
            )
            emails = emails_response.json()
            # Find primary verified email
            for email_data in emails:
                if email_data.get("primary") and email_data.get("verified"):
                    email = email_data.get("email")
                    break

        if not email:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="No verified email found in GitHub account"
            )

        # Find or create user
        user = db.query(User).filter(User.email == email).first()

        if not user:
            # Create new user with default experience levels (beginner)
            user = User(
                email=email,
                password_hash=hash_password(os.urandom(32).hex()),  # Random password for OAuth users
                software_level=ExperienceLevel.BEGINNER,
                hardware_level=ExperienceLevel.BEGINNER
            )
            db.add(user)
            db.commit()
            db.refresh(user)

        # Generate JWT token
        jwt_token = create_access_token(user.id)

        # Redirect to frontend with token
        return RedirectResponse(
            url=f"{FRONTEND_URL}?token={jwt_token}",
            status_code=status.HTTP_303_SEE_OTHER
        )
