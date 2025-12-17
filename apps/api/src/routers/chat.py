"""
Chat Router - RAG Chatbot API Endpoint

Provides endpoints for the RAG-powered chatbot.
"""

from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel, Field
from typing import Optional, List, Dict, Any
import logging
import re

from ..services.rag_generator import generate_rag_response

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/chat", tags=["chat"])

# Request/Response models
class ChatMessage(BaseModel):
    """Single chat message"""
    role: str = Field(..., pattern="^(user|assistant)$")
    content: str

class ChatRequest(BaseModel):
    """Chat request from frontend"""
    query: str = Field(..., min_length=1, max_length=500)
    chapter_id: Optional[str] = None
    conversation_history: Optional[List[ChatMessage]] = None

class Citation(BaseModel):
    """Citation/source reference"""
    title: str
    module: str
    week: str
    chapter_url: str
    snippet: str

class ChatResponse(BaseModel):
    """Chat response to frontend"""
    answer: str
    citations: List[Citation]
    model: str
    context_used: int


@router.post("/query", response_model=ChatResponse)
async def chat_query(request: ChatRequest):
    """
    Main RAG chatbot endpoint.

    Receives a user query, retrieves relevant content, and generates
    a response with citations.

    Args:
        request: ChatRequest with query and optional context

    Returns:
        ChatResponse with answer and citations

    Raises:
        HTTPException: If query is invalid or service error occurs
    """
    try:
        # Validate and sanitize query
        query = request.query.strip()

        if not query:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Query cannot be empty"
            )

        # Basic XSS prevention - remove HTML tags
        query = re.sub(r'<[^>]+>', '', query)

        # Convert conversation history to dict format if provided
        conversation_history = None
        if request.conversation_history:
            conversation_history = [
                {"role": msg.role, "content": msg.content}
                for msg in request.conversation_history
            ]

        # Log request (no PII)
        logger.info(f"Chat query received (length: {len(query)}, chapter: {request.chapter_id})")

        # Generate RAG response
        result = generate_rag_response(
            query=query,
            chapter_id=request.chapter_id,
            conversation_history=conversation_history
        )

        # Format response
        response = ChatResponse(
            answer=result['answer'],
            citations=[Citation(**citation) for citation in result['citations']],
            model=result['model'],
            context_used=result['context_used']
        )

        logger.info(f"Response generated with {len(response.citations)} citations")
        return response

    except ValueError as e:
        logger.error(f"Validation error: {e}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except Exception as e:
        logger.error(f"Error in chat endpoint: {e}", exc_info=True)

        # Provide more specific error messages
        error_msg = str(e)
        if "OPENAI_API_KEY" in error_msg or "OpenAI" in error_msg:
            detail = "OpenAI API key not configured. Please set OPENAI_API_KEY environment variable."
        elif "QDRANT" in error_msg or "Qdrant" in error_msg:
            detail = "Qdrant vector database not configured. Please set QDRANT_URL and QDRANT_API_KEY environment variables."
        elif "collection" in error_msg.lower():
            detail = "Textbook content not yet ingested. Please run the content ingestion script first."
        else:
            detail = f"Service error: {error_msg[:100]}"  # First 100 chars of error

        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=detail
        )


@router.get("/health")
async def chat_health():
    """
    Health check for chat service.

    Returns:
        Service status and configuration
    """
    import os

    openai_configured = bool(os.getenv("OPENAI_API_KEY"))
    qdrant_configured = bool(os.getenv("QDRANT_URL") and os.getenv("QDRANT_API_KEY"))

    return {
        "status": "healthy" if (openai_configured and qdrant_configured) else "degraded",
        "services": {
            "openai": "configured" if openai_configured else "not configured",
            "qdrant": "configured" if qdrant_configured else "not configured"
        }
    }
