# AuditLog Model - SQLAlchemy

from sqlalchemy import Column, String, Text, DateTime, ForeignKey
from sqlalchemy.dialects.postgresql import UUID, JSONB
from datetime import datetime
import uuid
from ..core.database import Base

class AuditLog(Base):
    __tablename__ = "audit_logs"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id"), nullable=True, index=True)  # Nullable for anonymous events
    event_type = Column(String, nullable=False, index=True)  # e.g., 'auth_failed', 'sanitization', 'rate_limit'
    event_details = Column(JSONB, default=dict, nullable=False)  # Additional context
    ip_address = Column(String, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False, index=True)

    def __repr__(self):
        return f"<AuditLog(id={self.id}, event_type={self.event_type}, user_id={self.user_id})>"
