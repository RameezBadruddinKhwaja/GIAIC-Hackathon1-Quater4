# User Model - SQLAlchemy

from sqlalchemy import Column, String, DateTime, Enum
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.ext.declarative import declarative_base
from datetime import datetime
import uuid
import enum

Base = declarative_base()

class AuthProvider(str, enum.Enum):
    EMAIL = "email"
    GITHUB = "github"

class HardwareProfile(str, enum.Enum):
    RTX_4090 = "rtx_4090"
    JETSON_ORIN_NANO = "jetson_orin_nano"

class ProgrammingLanguage(str, enum.Enum):
    PYTHON = "python"
    CPP = "cpp"

class User(Base):
    __tablename__ = "users"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    email = Column(String, unique=True, nullable=False, index=True)
    password_hash = Column(String, nullable=True)  # Null for OAuth users
    auth_provider = Column(Enum(AuthProvider), nullable=False)
    hardware_profile = Column(Enum(HardwareProfile), nullable=True)
    programming_language = Column(Enum(ProgrammingLanguage), nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    last_login = Column(DateTime, nullable=True)

    def __repr__(self):
        return f"<User(id={self.id}, email={self.email}, provider={self.auth_provider})>"
