"""
Embedding Generation Utilities

Generate 768-dimensional embeddings using Gemini text-embedding-004 model
via OpenAI SDK (drop-in replacement strategy).

IMPORTANT:
- Embedding dimensions: 768 (NOT 1536 like OpenAI)
- Model: text-embedding-004 (Gemini's embedding model)
- Base URL: Configured in gemini_client.py

Example Usage:
    embeddings = generate_embeddings(["Hello world", "ROS 2 topics"])
    # Returns: [[0.123, 0.456, ...], [0.789, 0.012, ...]]
"""

from typing import List
from ..services.gemini_client import get_gemini_client, EMBEDDING_MODEL, EMBEDDING_DIMENSIONS

def generate_embeddings(texts: List[str]) -> List[List[float]]:
    """
    Generate 768-dimensional embeddings for a list of texts.

    Args:
        texts: List of text strings to embed

    Returns:
        List of embedding vectors (each vector has 768 dimensions)

    Raises:
        ValueError: If texts list is empty
        Exception: If API call fails

    Example:
        >>> embeddings = generate_embeddings(["ROS 2 Topics", "URDF models"])
        >>> len(embeddings)
        2
        >>> len(embeddings[0])
        768
    """
    if not texts:
        raise ValueError("texts list cannot be empty")

    client = get_gemini_client()

    response = client.embeddings.create(
        model=EMBEDDING_MODEL,
        input=texts
    )

    # Extract embedding vectors from response
    embeddings = [item.embedding for item in response.data]

    # Verify dimensions
    for embedding in embeddings:
        if len(embedding) != EMBEDDING_DIMENSIONS:
            raise ValueError(
                f"Expected {EMBEDDING_DIMENSIONS}-dimensional embeddings, "
                f"got {len(embedding)}"
            )

    return embeddings

def generate_single_embedding(text: str) -> List[float]:
    """
    Generate 768-dimensional embedding for a single text.

    Args:
        text: Text string to embed

    Returns:
        Embedding vector (768 dimensions)

    Example:
        >>> embedding = generate_single_embedding("ROS 2 Topics")
        >>> len(embedding)
        768
    """
    embeddings = generate_embeddings([text])
    return embeddings[0]
