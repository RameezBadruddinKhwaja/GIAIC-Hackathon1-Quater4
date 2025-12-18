"""
Embeddings Service

Generates embeddings using OpenAI's text-embedding-3-small model.
"""

import os
from typing import List
from openai import OpenAI
import logging

logger = logging.getLogger(__name__)

# Embedding model
EMBEDDING_MODEL = "text-embedding-3-small"

class EmbeddingService:
    """Service for generating text embeddings"""
    _client: OpenAI = None

    @classmethod
    def get_client(cls) -> OpenAI:
        """Get or create OpenAI client (supports Gemini via OpenAI-compatible API)"""
        if cls._client is None:
            # Support both OpenAI and Gemini API keys
            api_key = os.getenv("OPENAI_API_KEY") or os.getenv("GEMINI_API_KEY")
            if not api_key:
                raise ValueError("OPENAI_API_KEY or GEMINI_API_KEY must be set in environment variables")

            cls._client = OpenAI(api_key=api_key)
            logger.info("OpenAI-compatible client initialized for embeddings")

        return cls._client

    @classmethod
    def generate_embedding(cls, text: str) -> List[float]:
        """
        Generate embedding for a single text.

        Args:
            text: Text to embed

        Returns:
            Embedding vector (1536 dimensions)
        """
        client = cls.get_client()

        try:
            response = client.embeddings.create(
                model=EMBEDDING_MODEL,
                input=text
            )
            return response.data[0].embedding
        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            raise

    @classmethod
    def generate_embeddings_batch(cls, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts in a batch.

        Args:
            texts: List of texts to embed

        Returns:
            List of embedding vectors
        """
        client = cls.get_client()

        try:
            response = client.embeddings.create(
                model=EMBEDDING_MODEL,
                input=texts
            )
            return [item.embedding for item in response.data]
        except Exception as e:
            logger.error(f"Error generating batch embeddings: {e}")
            raise

def generate_embedding(text: str) -> List[float]:
    """Convenience function to generate single embedding"""
    return EmbeddingService.generate_embedding(text)

def generate_embeddings_batch(texts: List[str]) -> List[List[float]]:
    """Convenience function to generate batch embeddings"""
    return EmbeddingService.generate_embeddings_batch(texts)
