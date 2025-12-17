"""
Qdrant Vector Store Configuration

Manages connection to Qdrant Cloud for vector embeddings storage and retrieval.
"""

import os
from typing import Optional
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams
import logging

logger = logging.getLogger(__name__)

# Collection name
COLLECTION_NAME = "textbook_content"

# Embedding dimensions (OpenAI text-embedding-3-small uses 1536 dimensions)
VECTOR_SIZE = 1536

class VectorStore:
    """Singleton wrapper for Qdrant client"""
    _instance: Optional[QdrantClient] = None

    @classmethod
    def get_client(cls) -> QdrantClient:
        """Get or create Qdrant client instance"""
        if cls._instance is None:
            qdrant_url = os.getenv("QDRANT_URL")
            qdrant_api_key = os.getenv("QDRANT_API_KEY")

            if not qdrant_url or not qdrant_api_key:
                raise ValueError("QDRANT_URL and QDRANT_API_KEY must be set in environment variables")

            cls._instance = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key,
            )
            logger.info(f"Qdrant client initialized: {qdrant_url}")

        return cls._instance

    @classmethod
    def ensure_collection(cls) -> None:
        """Create collection if it doesn't exist"""
        client = cls.get_client()

        try:
            # Check if collection exists
            collections = client.get_collections().collections
            collection_names = [c.name for c in collections]

            if COLLECTION_NAME not in collection_names:
                # Create collection
                client.create_collection(
                    collection_name=COLLECTION_NAME,
                    vectors_config=VectorParams(
                        size=VECTOR_SIZE,
                        distance=Distance.COSINE
                    )
                )
                logger.info(f"Created collection: {COLLECTION_NAME}")
            else:
                logger.info(f"Collection {COLLECTION_NAME} already exists")

        except Exception as e:
            logger.error(f"Error ensuring collection: {e}")
            raise

def get_vector_store() -> QdrantClient:
    """Get Qdrant client instance (convenience function)"""
    return VectorStore.get_client()
