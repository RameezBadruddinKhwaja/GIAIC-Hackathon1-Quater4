"""
Chat Router - RAG Chatbot Endpoint
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional
import logging

from ..services.rag_pipeline import chat_with_rag, ChatResponse

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/api")

class ChatRequest(BaseModel):
    query: str
    hardware_context: Optional[str] = None

@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    RAG Chatbot endpoint with Matrix Protocol skill loading.

    Args:
        request: ChatRequest with user query

    Returns:
        ChatResponse with answer, citations, and loaded skills
    """
    try:
        # Input validation and sanitization
        query = request.query.strip()

        if not query:
            raise HTTPException(status_code=400, detail="Query cannot be empty")

        if len(query) > 500:
            raise HTTPException(status_code=400, detail="Query too long (max 500 chars)")

        # Basic sanitization - remove HTML tags
        import re
        query = re.sub(r'<[^>]+>', '', query)

        # Log request (no PII)
        logger.info(f"Chat request received (length: {len(query)})")

        # Call RAG pipeline
        response = chat_with_rag(query)

        return response

    except Exception as e:
        logger.error(f"Error in chat endpoint: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")
