"""
User Model - Clean authentication rebuild

SQLAlchemy model for user authentication with software/hardware experience levels.
"""

from sqlalchemy import Column, String, DateTime, Integer, Enum as SQLEnum
from datetime import datetime
from src.core.database import Base
import enum


class ExperienceLevel(str, enum.Enum):
    """Experience level enum for software and hardware"""
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


class User(Base):
    """User model for authentication with experience profiles"""

    __tablename__ = "users"

    id = Column(Integer, primary_key=True, index=True)
    email = Column(String, unique=True, index=True, nullable=False)
    password_hash = Column(String, nullable=False)
    software_level = Column(SQLEnum(ExperienceLevel), nullable=False)
    hardware_level = Column(SQLEnum(ExperienceLevel), nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow, nullable=False)

    def __repr__(self):
        return f"<User(id={self.id}, email={self.email}, software={self.software_level}, hardware={self.hardware_level})>"
