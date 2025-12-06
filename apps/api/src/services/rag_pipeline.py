"""
RAG Pipeline - Retrieval-Augmented Generation

Searches Qdrant and generates answers with Gemini 2.5 Flash.
"""

import os
import logging
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from pydantic import BaseModel

from .gemini_client import get_gemini_client, CHAT_MODEL
from .matrix_loader import load_skills_for_query
from ..utils.embeddings import generate_single_embedding

logger = logging.getLogger(__name__)

class Citation(BaseModel):
    chapter_id: str
    title: str
    content_snippet: str
    chapter_url: str

class ChatResponse(BaseModel):
    response_text: str
    citations: List[Citation]
    skills_loaded: List[str]

def search_qdrant(query: str, top_k: int = 3, score_threshold: float = 0.7) -> List[Dict[str, Any]]:
    """
    Search Qdrant for relevant chunks.

    Args:
        query: User's query
        top_k: Number of results to return
        score_threshold: Minimum similarity score

    Returns:
        List of search results with payload and score
    """
    # Load env vars
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if not qdrant_url or not qdrant_api_key:
        raise ValueError("Qdrant credentials not configured")

    # Initialize client
    client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)

    # Generate query embedding
    query_embedding = generate_single_embedding(query)

    # Search
    search_results = client.search(
        collection_name="book_knowledge",
        query_vector=query_embedding,
        limit=top_k,
        score_threshold=score_threshold
    )

    results = []
    for result in search_results:
        results.append({
            'payload': result.payload,
            'score': result.score
        })

    logger.info(f"Found {len(results)} relevant chunks for query")
    return results

def generate_answer(query: str, context_chunks: List[Dict[str, Any]], loaded_skills: Dict[str, str]) -> str:
    """
    Generate answer using Gemini with context and skills.

    Args:
        query: User's question
        context_chunks: Retrieved chunks from Qdrant
        loaded_skills: Loaded Matrix skills

    Returns:
        Generated answer
    """
    client = get_gemini_client()

    # Build system prompt
    system_prompt = "You are an expert AI tutor for Physical AI and Robotics. "
    system_prompt += "Answer questions using ONLY the provided book content. "
    system_prompt += "Always cite chapter sources in your answer.\n\n"

    # Add loaded skills to prompt
    if loaded_skills:
        system_prompt += "LOADED SKILLS:\n"
        for skill_name, skill_content in loaded_skills.items():
            system_prompt += f"\n--- {skill_name} ---\n{skill_content}\n"

    # Build context from chunks
    context = "\n\n".join([
        f"[{chunk['payload']['title']}]\n{chunk['payload']['content']}"
        for chunk in context_chunks
    ])

    user_message = f"Context from textbook:\n\n{context}\n\nQuestion: {query}"

    # Generate response
    response = client.chat.completions.create(
        model=CHAT_MODEL,
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_message}
        ],
        temperature=0.7,
        max_tokens=500
    )

    return response.choices[0].message.content

def chat_with_rag(query: str) -> ChatResponse:
    """
    Main RAG pipeline: Search -> Load Skills -> Generate Answer.

    Args:
        query: User's question

    Returns:
        ChatResponse with answer, citations, and loaded skills
    """
    # Step 1: Load relevant skills
    loaded_skills = load_skills_for_query(query)

    # Step 2: Search Qdrant
    search_results = search_qdrant(query, top_k=3)

    if not search_results:
        return ChatResponse(
            response_text="I couldn't find relevant information in the textbook for your question.",
            citations=[],
            skills_loaded=list(loaded_skills.keys())
        )

    # Step 3: Generate answer
    answer = generate_answer(query, search_results, loaded_skills)

    # Step 4: Extract citations
    citations = []
    seen_chapters = set()

    for result in search_results:
        payload = result['payload']
        chapter_id = payload['chapter_id']

        if chapter_id not in seen_chapters:
            citations.append(Citation(
                chapter_id=chapter_id,
                title=payload.get('title', 'Untitled'),
                content_snippet=payload['content'][:150] + "...",
                chapter_url=f"/modules/{chapter_id}"
            ))
            seen_chapters.add(chapter_id)

    return ChatResponse(
        response_text=answer,
        citations=citations,
        skills_loaded=list(loaded_skills.keys())
    )
