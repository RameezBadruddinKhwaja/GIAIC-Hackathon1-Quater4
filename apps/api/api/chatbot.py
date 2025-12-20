from fastapi import APIRouter, HTTPException, Depends
from typing import List
from ..models.chat_models import ChatQuery, ChatSelectionQuery, ChatResponse, ChatHistoryResponse
from ..services.rag_service import rag_service
from ..db.neon import get_db
from sqlalchemy.ext.asyncio import AsyncSession

router = APIRouter(prefix="/chatbot", tags=["chatbot"])

@router.post("/query", response_model=ChatResponse)
async def query_endpoint(chat_query: ChatQuery):
    """
    Accept a question and return an answer with citations
    """
    try:
        response = rag_service.process_query(chat_query.question)

        # If we have a session_id in the request, we'd normally associate it with the response
        # For now, we'll return an empty session_id
        response.session_id = chat_query.session_id or ""

        return response
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")


@router.post("/query-selection", response_model=ChatResponse)
async def query_selection_endpoint(selection_query: ChatSelectionQuery):
    """
    Accept selected text + question and return a contextualized answer
    """
    try:
        # Combine the selected text with the question
        combined_query = f"Based on the following text: '{selection_query.selected_text}', {selection_query.question}"

        response = rag_service.process_query(combined_query)
        response.session_id = selection_query.session_id or ""

        return response
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing selection query: {str(e)}")


@router.get("/history/{session_id}", response_model=ChatHistoryResponse)
async def get_history_endpoint(session_id: str):
    """
    Retrieve chat history for a specific session
    """
    try:
        # In a real implementation, this would fetch from the database
        # For now, return an empty history
        return ChatHistoryResponse(
            messages=[],
            session_id=session_id
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving chat history: {str(e)}")