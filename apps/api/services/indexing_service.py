from typing import List, Dict, Any
from .vector_chunking_service import vector_chunking_service
from .embedding_service import embedding_service
from ..db.qdrant import qdrant_manager
from .rag_service import rag_service

class IndexingService:
    def __init__(self):
        # Initialize dependencies
        self.chunking_service = vector_chunking_service
        self.embedding_service = embedding_service
        self.qdrant_manager = qdrant_manager

    def index_book_content(self, chapters: List[Dict[str, Any]]):
        """
        Index book content by:
        1. Chunking the text
        2. Generating embeddings for each chunk
        3. Storing in Qdrant vector database
        """
        try:
            # Chunk the book content
            print("Chunking book content...")
            chunks = self.chunking_service.chunk_book_content(chapters)
            print(f"Created {len(chunks)} chunks from book content")

            # Generate embeddings for all chunks
            print("Generating embeddings for chunks...")
            texts = [chunk["text"] for chunk in chunks]
            embeddings = self.embedding_service.generate_embeddings_batch(texts)
            print(f"Generated {len(embeddings)} embeddings")

            # Add to Qdrant
            print("Adding vectors to Qdrant...")
            self.qdrant_manager.create_collection()  # Ensure collection exists
            self.qdrant_manager.add_vectors(embeddings, texts)
            print("Successfully indexed book content to Qdrant")

            return {"status": "success", "chunks_processed": len(chunks)}
        except Exception as e:
            print(f"Error indexing book content: {e}")
            return {"status": "error", "message": str(e)}

    def index_single_chapter(self, chapter: Dict[str, Any]):
        """
        Index a single chapter
        """
        try:
            # Chunk the chapter
            chunks = self.chunking_service.chunk_text(
                chapter.get("content", ""),
                {
                    "chapter": chapter.get("title", ""),
                    "module": chapter.get("module", ""),
                    "section": chapter.get("section", ""),
                    "url": chapter.get("url", "")
                }
            )

            # Generate embeddings
            texts = [chunk["text"] for chunk in chunks]
            embeddings = self.embedding_service.generate_embeddings_batch(texts)

            # Add to Qdrant
            self.qdrant_manager.create_collection()  # Ensure collection exists
            self.qdrant_manager.add_vectors(embeddings, texts)

            return {"status": "success", "chunks_processed": len(chunks)}
        except Exception as e:
            print(f"Error indexing single chapter: {e}")
            return {"status": "error", "message": str(e)}

# Global instance
indexing_service = IndexingService()