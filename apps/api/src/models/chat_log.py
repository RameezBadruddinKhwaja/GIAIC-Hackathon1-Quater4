# ChatLog Model - SQLAlchemy

from sqlalchemy import Column, String, Text, DateTime, ForeignKey
from sqlalchemy.dialects.postgresql import UUID, JSONB
from sqlalchemy.ext.declarative import declarative_base
from datetime import datetime
import uuid

Base = declarative_base()

class ChatLog(Base):
    __tablename__ = "chat_logs"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id"), nullable=False, index=True)
    query_text = Column(Text, nullable=False)
    response_text = Column(Text, nullable=False)
    cited_chapters = Column(JSONB, default=list, nullable=False)  # List of chapter IDs
    skills_loaded = Column(JSONB, default=list, nullable=False)  # List of skill names
    sanitized_input = Column(Text, nullable=False)  # Sanitized version of query_text
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False, index=True)

    def __repr__(self):
        return f"<ChatLog(id={self.id}, user_id={self.user_id}, query={self.query_text[:50]}...)>"
