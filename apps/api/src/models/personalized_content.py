# PersonalizedContent Model - SQLAlchemy

from sqlalchemy import Column, String, Text, DateTime, ForeignKey, Enum
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.ext.declarative import declarative_base
from datetime import datetime
import uuid
import enum

Base = declarative_base()

class HardwareProfile(str, enum.Enum):
    RTX_4090 = "rtx_4090"
    JETSON_ORIN_NANO = "jetson_orin_nano"

class PersonalizedContent(Base):
    __tablename__ = "personalized_content"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id"), nullable=False, index=True)
    chapter_id = Column(String, nullable=False, index=True)
    hardware_profile = Column(Enum(HardwareProfile), nullable=False)
    personalized_mdx = Column(Text, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False, index=True)

    def __repr__(self):
        return f"<PersonalizedContent(id={self.id}, user_id={self.user_id}, chapter={self.chapter_id})>"
