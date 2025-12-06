# TranslatedContent Model - SQLAlchemy

from sqlalchemy import Column, String, Text, DateTime, ForeignKey, Enum
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.ext.declarative import declarative_base
from datetime import datetime
import uuid
import enum

Base = declarative_base()

class TargetLanguage(str, enum.Enum):
    ROMAN_URDU = "roman_urdu"
    FORMAL_URDU = "formal_urdu"

class TranslatedContent(Base):
    __tablename__ = "translated_content"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id"), nullable=False, index=True)
    chapter_id = Column(String, nullable=False, index=True)
    target_language = Column(Enum(TargetLanguage), nullable=False)
    translated_mdx = Column(Text, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False, index=True)

    def __repr__(self):
        return f"<TranslatedContent(id={self.id}, user_id={self.user_id}, chapter={self.chapter_id}, lang={self.target_language})>"
