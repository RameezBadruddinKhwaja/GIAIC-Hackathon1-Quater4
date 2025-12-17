"""
RAG Response Generator

Uses OpenAI Chat Completions API to generate responses with retrieved context.
Following the specification for OpenAI Agents SDK integration.
"""

import os
from typing import List, Dict, Any, Optional
from openai import OpenAI
import logging
import re

from .retriever import search_content, search_chapter

logger = logging.getLogger(__name__)

# Model configuration
CHAT_MODEL = "gpt-4o-mini"  # Using GPT-4o-mini for cost efficiency
MAX_TOKENS = 800
TEMPERATURE = 0.7

class RAGGenerator:
    """Generates responses using OpenAI with RAG context"""

    _client: Optional[OpenAI] = None

    @classmethod
    def get_client(cls) -> OpenAI:
        """Get or create OpenAI client"""
        if cls._client is None:
            api_key = os.getenv("OPENAI_API_KEY")
            if not api_key:
                raise ValueError("OPENAI_API_KEY must be set in environment variables")

            cls._client = OpenAI(api_key=api_key)
            logger.info("OpenAI client initialized for chat completions")

        return cls._client

    @classmethod
    def build_system_prompt(cls) -> str:
        """Build system prompt for the AI tutor"""
        return """You are an expert AI tutor for the Physical AI & Humanoid Robotics textbook.

Your responsibilities:
1. Answer questions using ONLY the provided textbook content
2. Always cite specific chapters when referencing information
3. If the provided context doesn't contain the answer, say so clearly
4. Be concise but thorough in explanations
5. Use technical terms appropriately for the student level

Citation format: Reference chapters like "Week 3: ROS2 Packages" or "Module 2: Digital Twin"

Remember: You cannot access information outside the provided textbook context."""

    @classmethod
    def build_context_prompt(cls, chunks: List[Dict[str, Any]]) -> str:
        """
        Build context section from retrieved chunks.

        Args:
            chunks: Retrieved chunks with content and metadata

        Returns:
            Formatted context string
        """
        if not chunks:
            return "No relevant content found in the textbook."

        context_parts = []
        for i, chunk in enumerate(chunks, 1):
            metadata = chunk['metadata']
            title = metadata.get('title', 'Unknown')
            module = metadata.get('module', 'unknown')
            week = metadata.get('week', 'unknown')

            context_parts.append(
                f"[Source {i}: {title} ({module}, {week})]\n{chunk['content']}\n"
            )

        return "\n".join(context_parts)

    @classmethod
    def extract_citations(cls, chunks: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Extract citation information from chunks.

        Args:
            chunks: Retrieved chunks

        Returns:
            List of citation dictionaries
        """
        citations = []
        seen_chapters = set()

        for chunk in chunks:
            metadata = chunk['metadata']
            chapter_url = metadata.get('chapter_url', '')

            # Avoid duplicate citations
            if chapter_url and chapter_url not in seen_chapters:
                citations.append({
                    'title': metadata.get('title', 'Untitled'),
                    'module': metadata.get('module', 'unknown'),
                    'week': metadata.get('week', 'unknown'),
                    'chapter_url': chapter_url,
                    'snippet': chunk['content'][:150] + '...'
                })
                seen_chapters.add(chapter_url)

        return citations

    @classmethod
    def generate_response(
        cls,
        query: str,
        context_chunks: List[Dict[str, Any]],
        conversation_history: Optional[List[Dict[str, str]]] = None
    ) -> Dict[str, Any]:
        """
        Generate RAG response.

        Args:
            query: User's question
            context_chunks: Retrieved context chunks
            conversation_history: Previous messages (optional)

        Returns:
            Response dictionary with text and citations
        """
        client = cls.get_client()

        # Build prompts
        system_prompt = cls.build_system_prompt()
        context_prompt = cls.build_context_prompt(context_chunks)

        # Build messages
        messages = [
            {"role": "system", "content": system_prompt}
        ]

        # Add conversation history if provided
        if conversation_history:
            messages.extend(conversation_history[-5:])  # Keep last 5 messages

        # Add current query with context
        user_message = f"""Textbook Content:

{context_prompt}

Student Question: {query}

Please answer the question using ONLY the provided textbook content above. Cite the relevant chapters."""

        messages.append({"role": "user", "content": user_message})

        try:
            # Generate response
            response = client.chat.completions.create(
                model=CHAT_MODEL,
                messages=messages,
                temperature=TEMPERATURE,
                max_tokens=MAX_TOKENS
            )

            answer = response.choices[0].message.content

            # Extract citations
            citations = cls.extract_citations(context_chunks)

            return {
                'answer': answer,
                'citations': citations,
                'model': CHAT_MODEL,
                'context_used': len(context_chunks)
            }

        except Exception as e:
            logger.error(f"Error generating response: {e}")
            raise

    @classmethod
    def chat(
        cls,
        query: str,
        chapter_id: Optional[str] = None,
        top_k: int = 5,
        conversation_history: Optional[List[Dict[str, str]]] = None
    ) -> Dict[str, Any]:
        """
        Complete RAG pipeline: retrieve + generate.

        Args:
            query: User's question
            chapter_id: Optional chapter ID to search within
            top_k: Number of chunks to retrieve
            conversation_history: Previous conversation messages

        Returns:
            Complete response with answer and citations
        """
        try:
            # Retrieve relevant chunks
            if chapter_id:
                chunks = search_chapter(query, chapter_id, top_k=top_k)
            else:
                chunks = search_content(query, top_k=top_k, score_threshold=0.7)

            # Check if we found relevant content
            if not chunks:
                return {
                    'answer': "I couldn't find relevant information in the textbook to answer your question. Could you try rephrasing or asking about a different topic from the Physical AI & Humanoid Robotics curriculum?",
                    'citations': [],
                    'model': CHAT_MODEL,
                    'context_used': 0
                }

            # Generate response
            response = cls.generate_response(query, chunks, conversation_history)

            logger.info(f"Generated response for query using {response['context_used']} chunks")
            return response

        except Exception as e:
            logger.error(f"Error in RAG chat: {e}")
            raise


# Convenience functions
def generate_rag_response(
    query: str,
    chapter_id: Optional[str] = None,
    conversation_history: Optional[List[Dict[str, str]]] = None
) -> Dict[str, Any]:
    """Generate RAG response (convenience function)"""
    return RAGGenerator.chat(query, chapter_id, conversation_history=conversation_history)
