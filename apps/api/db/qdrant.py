import os
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.models import Distance, VectorParams
from dotenv import load_dotenv
from typing import List, Dict, Any

load_dotenv()

class QdrantManager:
    def __init__(self):
        # Get Qdrant configuration from environment variables
        self.qdrant_url = os.getenv("QDRANT_URL", "")
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY", "")

        if self.qdrant_url and self.qdrant_api_key:
            self.client = QdrantClient(url=self.qdrant_url, api_key=self.qdrant_api_key)
        elif self.qdrant_url:
            self.client = QdrantClient(url=self.qdrant_url)
        else:
            # Use local instance if no URL provided
            self.client = QdrantClient(":memory:")  # For development/testing

        self.collection_name = "physical-ai-textbook"

    def create_collection(self):
        """Initialize the Qdrant collection for storing textbook content"""
        try:
            # Check if collection already exists
            collections = self.client.get_collections().collections
            collection_names = [col.name for col in collections]

            if self.collection_name not in collection_names:
                # Create collection with 384 dimensions (for sentence transformer embeddings)
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(size=384, distance=Distance.COSINE),
                )

                print(f"Created Qdrant collection: {self.collection_name}")
            else:
                print(f"Qdrant collection {self.collection_name} already exists")
        except Exception as e:
            print(f"Error creating Qdrant collection: {e}")

    def add_vectors(self, vectors: List[Dict[str, Any]], texts: List[str]):
        """Add vectors with their associated text and metadata to the collection"""
        try:
            points = []
            for idx, (vector, text) in enumerate(zip(vectors, texts)):
                points.append(models.PointStruct(
                    id=idx,
                    vector=vector,
                    payload={
                        "text": text,
                        "chunk_id": idx
                    }
                ))

            self.client.upsert(collection_name=self.collection_name, points=points)
            print(f"Added {len(points)} vectors to collection {self.collection_name}")
        except Exception as e:
            print(f"Error adding vectors to Qdrant: {e}")

    def search_similar(self, query_vector: List[float], top_k: int = 5) -> List[Dict[str, Any]]:
        """Search for similar vectors in the collection"""
        try:
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=top_k
            )

            return [{
                "id": result.id,
                "score": result.score,
                "payload": result.payload
            } for result in results]
        except Exception as e:
            print(f"Error searching in Qdrant: {e}")
            return []

# Global instance
qdrant_manager = QdrantManager()