"""
Authentication Router - Clean auth API endpoints

Handles signup, signin, signout, and user profile endpoints.
"""

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from sqlalchemy.exc import IntegrityError
from typing import Annotated

from src.core.database import get_db
from src.core.security import hash_password, verify_password, create_access_token
from src.core.deps import get_current_user
from src.models.user import User
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
