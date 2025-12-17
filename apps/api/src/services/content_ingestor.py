"""
Content Ingestor

Reads MDX files, chunks them, generates embeddings, and stores in Qdrant.
"""

import os
import re
from pathlib import Path
from typing import List, Dict, Any
import uuid
import logging
from qdrant_client.models import PointStruct

from ..db.vector_store import get_vector_store, COLLECTION_NAME, VectorStore
from .embeddings import generate_embeddings_batch

logger = logging.getLogger(__name__)

# Chunking configuration
CHUNK_SIZE = 800  # characters per chunk
CHUNK_OVERLAP = 200  # overlap between chunks

class ContentIngestor:
    """Handles MDX content ingestion into Qdrant"""

    def __init__(self, docs_dir: str):
        """
        Initialize ingestor.

        Args:
            docs_dir: Path to the docs directory containing MDX files
        """
        self.docs_dir = Path(docs_dir)
        if not self.docs_dir.exists():
            raise ValueError(f"Docs directory not found: {docs_dir}")

    def extract_text_from_mdx(self, content: str) -> str:
        """
        Extract clean text from MDX content.

        Removes:
        - JSX/React components
        - Code blocks (preserves inline code)
        - Front matter
        - HTML tags

        Args:
            content: Raw MDX content

        Returns:
            Cleaned text
        """
        # Remove front matter
        content = re.sub(r'^---\n.*?\n---\n', '', content, flags=re.DOTALL)

        # Remove code blocks but keep language identifier for context
        content = re.sub(r'```(\w+)?\n(.*?)\n```', r'[Code: \1]', content, flags=re.DOTALL)

        # Remove JSX/React components
        content = re.sub(r'<[^>]+>', '', content)

        # Remove markdown image syntax
        content = re.sub(r'!\[.*?\]\(.*?\)', '', content)

        # Clean up multiple newlines
        content = re.sub(r'\n\s*\n', '\n\n', content)

        return content.strip()

    def chunk_text(self, text: str, metadata: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Chunk text into semantic units.

        Args:
            text: Text to chunk
            metadata: Base metadata for chunks

        Returns:
            List of chunks with metadata
        """
        chunks = []
        words = text.split()
        current_chunk = []
        current_size = 0

        for word in words:
            word_size = len(word) + 1  # +1 for space
            if current_size + word_size > CHUNK_SIZE and current_chunk:
                # Create chunk
                chunk_text = ' '.join(current_chunk)
                chunks.append({
                    'text': chunk_text,
                    'metadata': {
                        **metadata,
                        'chunk_index': len(chunks),
                        'chunk_size': len(chunk_text)
                    }
                })

                # Start new chunk with overlap
                overlap_words = current_chunk[-CHUNK_OVERLAP // 10:]  # rough word overlap
                current_chunk = overlap_words
                current_size = sum(len(w) + 1 for w in overlap_words)

            current_chunk.append(word)
            current_size += word_size

        # Add final chunk
        if current_chunk:
            chunk_text = ' '.join(current_chunk)
            chunks.append({
                'text': chunk_text,
                'metadata': {
                    **metadata,
                    'chunk_index': len(chunks),
                    'chunk_size': len(chunk_text)
                }
            })

        return chunks

    def extract_chapter_metadata(self, file_path: Path) -> Dict[str, Any]:
        """
        Extract metadata from file path.

        Expected structure: docs/module-X/week-Y-title.mdx

        Args:
            file_path: Path to MDX file

        Returns:
            Metadata dictionary
        """
        relative_path = file_path.relative_to(self.docs_dir)
        parts = relative_path.parts

        metadata = {
            'file_path': str(relative_path),
            'module': 'unknown',
            'week': 'unknown',
            'title': file_path.stem.replace('-', ' ').title()
        }

        # Extract module
        for part in parts:
            if part.startswith('module-'):
                metadata['module'] = part
                break

        # Extract week from filename
        filename = file_path.stem
        week_match = re.match(r'week-(\d+)-(.+)', filename)
        if week_match:
            metadata['week'] = f"week-{week_match.group(1)}"
            metadata['title'] = week_match.group(2).replace('-', ' ').title()

        # Generate chapter URL
        url_path = str(relative_path).replace('.mdx', '').replace('.md', '')
        metadata['chapter_url'] = f"/docs/{url_path}"

        return metadata

    def ingest_file(self, file_path: Path) -> int:
        """
        Ingest a single MDX file.

        Args:
            file_path: Path to MDX file

        Returns:
            Number of chunks created
        """
        logger.info(f"Ingesting file: {file_path}")

        try:
            # Read file
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract text
            text = self.extract_text_from_mdx(content)

            if len(text) < 100:  # Skip very small files
                logger.warning(f"Skipping small file: {file_path}")
                return 0

            # Extract metadata
            metadata = self.extract_chapter_metadata(file_path)

            # Chunk text
            chunks = self.chunk_text(text, metadata)

            if not chunks:
                logger.warning(f"No chunks created for: {file_path}")
                return 0

            # Generate embeddings
            chunk_texts = [chunk['text'] for chunk in chunks]
            embeddings = generate_embeddings_batch(chunk_texts)

            # Create points for Qdrant
            points = []
            for chunk, embedding in zip(chunks, embeddings):
                point = PointStruct(
                    id=str(uuid.uuid4()),
                    vector=embedding,
                    payload={
                        'content': chunk['text'],
                        **chunk['metadata']
                    }
                )
                points.append(point)

            # Upsert to Qdrant
            client = get_vector_store()
            client.upsert(
                collection_name=COLLECTION_NAME,
                points=points
            )

            logger.info(f"Ingested {len(chunks)} chunks from {file_path}")
            return len(chunks)

        except Exception as e:
            logger.error(f"Error ingesting file {file_path}: {e}")
            return 0

    def ingest_all(self) -> Dict[str, int]:
        """
        Ingest all MDX files from docs directory.

        Returns:
            Statistics dictionary
        """
        logger.info(f"Starting ingestion from: {self.docs_dir}")

        # Ensure collection exists
        VectorStore.ensure_collection()

        # Find all MDX files
        mdx_files = list(self.docs_dir.rglob("*.mdx")) + list(self.docs_dir.rglob("*.md"))
        logger.info(f"Found {len(mdx_files)} MDX/MD files")

        # Ingest each file
        total_chunks = 0
        successful_files = 0

        for file_path in mdx_files:
            chunks_count = self.ingest_file(file_path)
            if chunks_count > 0:
                total_chunks += chunks_count
                successful_files += 1

        stats = {
            'total_files': len(mdx_files),
            'successful_files': successful_files,
            'total_chunks': total_chunks
        }

        logger.info(f"Ingestion complete: {stats}")
        return stats


def run_ingestion(docs_dir: str) -> Dict[str, int]:
    """
    Convenience function to run full ingestion.

    Args:
        docs_dir: Path to docs directory

    Returns:
        Ingestion statistics
    """
    ingestor = ContentIngestor(docs_dir)
    return ingestor.ingest_all()
