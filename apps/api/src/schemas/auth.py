"""
Authentication Schemas - Pydantic models for validation

Request and response schemas for authentication endpoints.
"""

from pydantic import BaseModel, EmailStr, field_validator
from datetime import datetime
from enum import Enum
import re


class ExperienceLevel(str, Enum):
    """Experience level enum"""
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


class SignupRequest(BaseModel):
    """Request schema for user signup"""
    email: EmailStr
    password: str
    software_level: ExperienceLevel
    hardware_level: ExperienceLevel

    @field_validator('password')
    @classmethod
    def validate_password(cls, v: str) -> str:
        """Validate password strength"""
        if len(v) < 8:
            raise ValueError('Password must be at least 8 characters long')

        if not re.search(r'[A-Za-z]', v):
            raise ValueError('Password must contain at least one letter')

        if not re.search(r'\d', v):
            raise ValueError('Password must contain at least one number')

        return v


class SigninRequest(BaseModel):
    """Request schema for user signin"""
    email: EmailStr
    password: str


class UserResponse(BaseModel):
    """Response schema for user data (no password)"""
    id: int
    email: str
    software_level: ExperienceLevel
    hardware_level: ExperienceLevel
    created_at: datetime

    class Config:
        from_attributes = True  # Enable ORM mode for SQLAlchemy


class TokenResponse(BaseModel):
    """Response schema for authentication tokens"""
    access_token: str
    token_type: str = "bearer"
