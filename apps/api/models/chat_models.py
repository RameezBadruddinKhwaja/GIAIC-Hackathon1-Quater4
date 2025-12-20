from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime
from uuid import UUID

# Chat message models
class Citation(BaseModel):
    chapter: str
    section: str
    url: str

class ChatMessage(BaseModel):
    role: str  # 'user' or 'assistant'
    content: str
    citations: Optional[List[Citation]] = None

class ChatQuery(BaseModel):
    question: str
    session_id: Optional[str] = None

class ChatSelectionQuery(BaseModel):
    selected_text: str
    question: str
    session_id: Optional[str] = None

class ChatResponse(BaseModel):
    answer: str
    citations: List[Citation]
    session_id: str

class ChatHistoryResponse(BaseModel):
    messages: List[ChatMessage]
    session_id: str