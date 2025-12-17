"""
Vector Retrieval Service

Searches Qdrant for relevant content chunks based on query embeddings.
"""

from typing import List, Dict, Any
import logging

from ..db.vector_store import get_vector_store, COLLECTION_NAME
from .embeddings import generate_embedding

logger = logging.getLogger(__name__)

class RetrievalService:
    """Service for retrieving relevant content from vector store"""

    @staticmethod
    def search(
        query: str,
        top_k: int = 5,
        score_threshold: float = 0.7
    ) -> List[Dict[str, Any]]:
        """
        Search for relevant content chunks.

        Args:
            query: User's query text
            top_k: Number of results to return
            score_threshold: Minimum similarity score (0-1)

        Returns:
            List of matching chunks with metadata and scores
        """
        try:
            # Generate query embedding
            query_embedding = generate_embedding(query)

            # Search Qdrant
            client = get_vector_store()
            search_results = client.search(
                collection_name=COLLECTION_NAME,
                query_vector=query_embedding,
                limit=top_k,
                score_threshold=score_threshold
            )

            # Format results
            results = []
            for result in search_results:
                results.append({
                    'content': result.payload.get('content', ''),
                    'score': result.score,
                    'metadata': {
                        'module': result.payload.get('module', 'unknown'),
                        'week': result.payload.get('week', 'unknown'),
                        'title': result.payload.get('title', 'Untitled'),
                        'chapter_url': result.payload.get('chapter_url', ''),
                        'chunk_index': result.payload.get('chunk_index', 0),
                        'file_path': result.payload.get('file_path', '')
                    }
                })

            logger.info(f"Retrieved {len(results)} chunks for query (scores: {[r['score'] for r in results]})")
            return results

        except Exception as e:
            logger.error(f"Error in retrieval: {e}")
            raise

    @staticmethod
    def search_by_chapter(
        query: str,
        chapter_id: str,
        top_k: int = 3
    ) -> List[Dict[str, Any]]:
        """
        Search within a specific chapter.

        Args:
            query: User's query text
            chapter_id: Chapter identifier (e.g., "module-1/week-1")
            top_k: Number of results to return

        Returns:
            List of matching chunks from the specified chapter
        """
        try:
            # Generate query embedding
            query_embedding = generate_embedding(query)

            # Search with filter
            client = get_vector_store()
            search_results = client.search(
                collection_name=COLLECTION_NAME,
                query_vector=query_embedding,
                limit=top_k,
                query_filter={
                    "must": [
                        {
                            "key": "file_path",
                            "match": {"text": chapter_id}
                        }
                    ]
                }
            )

            # Format results
            results = []
            for result in search_results:
                results.append({
                    'content': result.payload.get('content', ''),
                    'score': result.score,
                    'metadata': {
                        'module': result.payload.get('module', 'unknown'),
                        'week': result.payload.get('week', 'unknown'),
                        'title': result.payload.get('title', 'Untitled'),
                        'chapter_url': result.payload.get('chapter_url', ''),
                        'chunk_index': result.payload.get('chunk_index', 0),
                        'file_path': result.payload.get('file_path', '')
                    }
                })

            logger.info(f"Retrieved {len(results)} chunks from chapter {chapter_id}")
            return results

        except Exception as e:
            logger.error(f"Error in chapter-specific retrieval: {e}")
            raise


def search_content(query: str, top_k: int = 5, score_threshold: float = 0.7) -> List[Dict[str, Any]]:
    """Convenience function for content search"""
    return RetrievalService.search(query, top_k, score_threshold)


def search_chapter(query: str, chapter_id: str, top_k: int = 3) -> List[Dict[str, Any]]:
    """Convenience function for chapter-specific search"""
    return RetrievalService.search_by_chapter(query, chapter_id, top_k)
