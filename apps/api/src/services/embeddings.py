"""
Embeddings Service

Generates embeddings using sentence transformers (free alternative to OpenAI embeddings).
"""

import os
from typing import List
import logging
from sentence_transformers import SentenceTransformer

logger = logging.getLogger(__name__)

class EmbeddingService:
    """Service for generating text embeddings"""
    _model: SentenceTransformer = None

    @classmethod
    def get_model(cls) -> SentenceTransformer:
        """Get or create sentence transformer model"""
        if cls._model is None:
            # Use a pre-trained sentence transformer model for embeddings
            # This is a free alternative to OpenAI embeddings and works well for RAG
            cls._model = SentenceTransformer('all-MiniLM-L6-v2')
            logger.info("Sentence transformer model initialized for embeddings")

        return cls._model

    @classmethod
    def generate_embedding(cls, text: str) -> List[float]:
        """
        Generate embedding for a single text.

        Args:
            text: Text to embed

        Returns:
            Embedding vector
        """
        model = cls.get_model()

        try:
            # Generate embedding using the sentence transformer model
            embedding = model.encode([text])
            # Convert to list of floats
            embedding_list = embedding[0].tolist()
            return embedding_list
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
        model = cls.get_model()

        try:
            # Generate embeddings for all texts at once for efficiency
            embeddings = model.encode(texts)
            # Convert to list of lists of floats
            embeddings_list = [embedding.tolist() for embedding in embeddings]
            return embeddings_list
        except Exception as e:
            logger.error(f"Error generating batch embeddings: {e}")
            raise

def generate_embedding(text: str) -> List[float]:
    """Convenience function to generate single embedding"""
    return EmbeddingService.generate_embedding(text)

def generate_embeddings_batch(texts: List[str]) -> List[List[float]]:
    """Convenience function to generate batch embeddings"""
    return EmbeddingService.generate_embeddings_batch(texts)
