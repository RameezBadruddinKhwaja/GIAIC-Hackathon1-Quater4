import os
from typing import List
from dotenv import load_dotenv
from sentence_transformers import SentenceTransformer
import numpy as np

load_dotenv()

class EmbeddingService:
    def __init__(self):
        # Use a pre-trained sentence transformer model for embeddings
        # This is a free alternative to OpenAI embeddings and works well for RAG
        self.model = SentenceTransformer('all-MiniLM-L6-v2')

    def generate_embedding(self, text: str) -> List[float]:
        """Generate embedding for a single text using sentence transformer model"""
        try:
            # Generate embedding using the sentence transformer model
            embedding = self.model.encode([text])
            # Convert to list of floats
            embedding_list = embedding[0].tolist()
            return embedding_list
        except Exception as e:
            print(f"Error generating embedding: {e}")
            return []

    def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for a batch of texts"""
        try:
            # Generate embeddings for all texts at once for efficiency
            embeddings = self.model.encode(texts)
            # Convert to list of lists of floats
            embeddings_list = [embedding.tolist() for embedding in embeddings]
            return embeddings_list
        except Exception as e:
            print(f"Error generating embeddings batch: {e}")
            return [[] for _ in texts]

# Global instance
embedding_service = EmbeddingService()