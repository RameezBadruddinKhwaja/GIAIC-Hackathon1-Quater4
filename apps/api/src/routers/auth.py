"""
Authentication Router

Endpoints for user signup, login, logout, and profile management.
"""

import os
import httpx
from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from fastapi.responses import RedirectResponse
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from pydantic import BaseModel, EmailStr
from typing import Optional

from src.core.database import get_db
from src.core.security import (
    verify_password,
    get_password_hash,
    create_access_token,
    decode_access_token,
)
from src.models.user import User

router = APIRouter(prefix="/api/auth", tags=["auth"])

# OAuth2 scheme for token authentication
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="/api/auth/login")


# Pydantic schemas
class UserSignUp(BaseModel):
    email: EmailStr
    username: str
    password: str
    full_name: Optional[str] = None


class UserLogin(BaseModel):
    username: str
    password: str


class UserResponse(BaseModel):
    id: int
    email: str
    username: str
    full_name: Optional[str]

    class Config:
        from_attributes = True


class Token(BaseModel):
    access_token: str
    token_type: str


class TokenData(BaseModel):
    user_id: Optional[int] = None


# Dependency: Get current user from token
async def get_current_user(
    token: str = Depends(oauth2_scheme),
    db: AsyncSession = Depends(get_db)
) -> User:
    """
    Get current authenticated user from JWT token.
    
    Raises:
        HTTPException: If token is invalid or user not found
    """
    payload = decode_access_token(token)
    user_id: int = payload.get("sub")
    
    if user_id is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials"
        )
    
    result = await db.execute(select(User).filter(User.id == user_id))
    user = result.scalar_one_or_none()
    
    if user is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="User not found"
        )
    
    return user


@router.post("/signup", response_model=UserResponse, status_code=status.HTTP_201_CREATED)
async def signup(user_data: UserSignUp, db: AsyncSession = Depends(get_db)):
    """
    Create a new user account.

    Args:
        user_data: User registration data
        db: Database session

    Returns:
        Created user object

    Raises:
        HTTPException: If email or username already exists or validation fails
    """
    try:
        # Validate input
        if len(user_data.password) < 8:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Password must be at least 8 characters long"
            )

        if len(user_data.username) < 3:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Username must be at least 3 characters long"
            )

        # Check if email exists
        result = await db.execute(select(User).filter(User.email == user_data.email))
        if result.scalar_one_or_none():
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Email already registered. Please use a different email or try logging in."
            )

        # Check if username exists
        result = await db.execute(select(User).filter(User.username == user_data.username))
        if result.scalar_one_or_none():
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Username already taken. Please choose a different username."
            )

        # Create new user
        hashed_password = get_password_hash(user_data.password)
        new_user = User(
            email=user_data.email,
            username=user_data.username,
            hashed_password=hashed_password,
            full_name=user_data.full_name,
        )

        db.add(new_user)
        await db.commit()
        await db.refresh(new_user)

        return new_user

    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        # Log unexpected errors and return generic message
        await db.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while creating your account. Please try again."
        )


@router.post("/login", response_model=Token)
async def login(user_data: UserLogin, db: AsyncSession = Depends(get_db)):
    """
    Login with username and password.
    
    Args:
        user_data: Login credentials
        db: Database session
    
    Returns:
        JWT access token
    
    Raises:
        HTTPException: If credentials are invalid
    """
    # Find user
    result = await db.execute(select(User).filter(User.username == user_data.username))
    user = result.scalar_one_or_none()
    
    if not user or not verify_password(user_data.password, user.hashed_password):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect username or password",
            headers={"WWW-Authenticate": "Bearer"},
        )
    
    # Create access token
    access_token = create_access_token(data={"sub": user.id})
    
    return {"access_token": access_token, "token_type": "bearer"}


@router.get("/me", response_model=UserResponse)
async def get_me(current_user: User = Depends(get_current_user)):
    """
    Get current user profile.
    
    Args:
        current_user: Authenticated user from token
    
    Returns:
        User profile data
    """
    return current_user


@router.post("/logout")
async def logout(current_user: User = Depends(get_current_user)):
    """
    Logout current user.
    
    Note: JWT tokens are stateless, so logout is handled client-side
    by removing the token. This endpoint validates the token is valid.
    
    Returns:
        Success message
    """
    return {"message": "Successfully logged out"}

# GitHub OAuth Configuration
GITHUB_CLIENT_ID = os.getenv("GITHUB_CLIENT_ID")
GITHUB_CLIENT_SECRET = os.getenv("GITHUB_CLIENT_SECRET")
GITHUB_REDIRECT_URI = os.getenv("GITHUB_REDIRECT_URI", "https://giaic-hackathon1-quater4.vercel.app/api/auth/github/callback")


@router.get("/github/login")
async def github_login():
    """
    Initiate GitHub OAuth flow.
    
    Redirects user to GitHub authorization page.
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
async def github_callback(code: str, db: AsyncSession = Depends(get_db)):
    """
    Handle GitHub OAuth callback.
    
    Exchange authorization code for access token, fetch user data,
    and create/login user.
    
    Args:
        code: Authorization code from GitHub
        db: Database session
    
    Returns:
        Redirect to frontend with JWT token
    """
    if not GITHUB_CLIENT_ID or not GITHUB_CLIENT_SECRET:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="GitHub OAuth not configured"
        )
    
    # Exchange code for access token
    async with httpx.AsyncClient() as client:
        token_response = await client.post(
            "https://github.com/login/oauth/access_token",
            data={
                "client_id": GITHUB_CLIENT_ID,
                "client_secret": GITHUB_CLIENT_SECRET,
                "code": code,
            },
            headers={"Accept": "application/json"},
        )
        
        if token_response.status_code != 200:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Failed to exchange code for token"
            )
        
        token_data = token_response.json()
        access_token = token_data.get("access_token")
        
        if not access_token:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="No access token in response"
            )
        
        # Fetch user data from GitHub
        user_response = await client.get(
            "https://api.github.com/user",
            headers={"Authorization": f"Bearer {access_token}"},
        )
        
        if user_response.status_code != 200:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Failed to fetch user data"
            )
        
        github_user = user_response.json()
        
        # Fetch user email if not public
        email = github_user.get("email")
        if not email:
            email_response = await client.get(
                "https://api.github.com/user/emails",
                headers={"Authorization": f"Bearer {access_token}"},
            )
            if email_response.status_code == 200:
                emails = email_response.json()
                primary_email = next(
                    (e["email"] for e in emails if e["primary"]), None
                )
                email = primary_email or emails[0]["email"] if emails else None
        
        if not email:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Could not fetch email from GitHub"
            )
        
        # Find or create user
        username = github_user.get("login")
        name = github_user.get("name") or username
        
        result = await db.execute(select(User).filter(User.email == email))
        user = result.scalar_one_or_none()
        
        if not user:
            # Check if username exists
            result = await db.execute(select(User).filter(User.username == username))
            if result.scalar_one_or_none():
                # Append random suffix to username if taken
                import random
                username = f"{username}{random.randint(1000, 9999)}"
            
            # Create new user with GitHub data
            # Use a random password since they'll login via GitHub
            import secrets
            random_password = secrets.token_urlsafe(32)
            
            user = User(
                email=email,
                username=username,
                hashed_password=get_password_hash(random_password),
                full_name=name,
            )
            db.add(user)
            await db.commit()
            await db.refresh(user)
        
        # Create JWT token
        jwt_token = create_access_token(data={"sub": user.id})
        
        # Redirect to frontend with token
        frontend_url = os.getenv("FRONTEND_URL", "https://giaic-hackathon1-quater4-frontend.vercel.app")
        return RedirectResponse(
            url=f"{frontend_url}/?token={jwt_token}"
        )
