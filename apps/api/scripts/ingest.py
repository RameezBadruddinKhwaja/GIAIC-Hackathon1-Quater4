"""
Content Ingestion Script

Parses Docusaurus MDX files and indexes them into Qdrant for RAG retrieval.

Usage:
    python scripts/ingest.py
"""

import os
import sys
import re
from pathlib import Path
from typing import List, Dict, Any
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from src.utils.embeddings import generate_embeddings

def extract_frontmatter(content: str) -> Dict[str, str]:
    """Extract YAML frontmatter from markdown file."""
    frontmatter_pattern = r'^---\s*\n(.*?)\n---\s*\n'
    match = re.match(frontmatter_pattern, content, re.DOTALL)

    if not match:
        return {}

    frontmatter_text = match.group(1)
    metadata = {}

    for line in frontmatter_text.split('\n'):
        if ':' in line:
            key, value = line.split(':', 1)
            metadata[key.strip()] = value.strip().strip('"').strip("'")

    return metadata

def chunk_text(text: str, chunk_size: int = 500, overlap: int = 50) -> List[str]:
    """Split text into overlapping chunks."""
    words = text.split()
    chunks = []

    for i in range(0, len(words), chunk_size - overlap):
        chunk = ' '.join(words[i:i + chunk_size])
        if chunk:
            chunks.append(chunk)

    return chunks

def parse_markdown_files(docs_dir: str) -> List[Dict[str, Any]]:
    """Parse all markdown files in docs directory."""
    docs_path = Path(docs_dir)
    documents = []

    for md_file in docs_path.rglob('*.md'):
        try:
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract frontmatter
            metadata = extract_frontmatter(content)

            # Remove frontmatter from content
            content_without_frontmatter = re.sub(
                r'^---\s*\n.*?\n---\s*\n',
                '',
                content,
                count=1,
                flags=re.DOTALL
            )

            # Get relative path for chapter_id
            relative_path = md_file.relative_to(docs_path)
            chapter_id = str(relative_path.with_suffix('')).replace('\\', '/')

            # Determine module/part number
            part_number = 0
            if 'module-1' in str(relative_path):
                part_number = 1
            elif 'module-2' in str(relative_path):
                part_number = 2
            elif 'module-3' in str(relative_path):
                part_number = 3
            elif 'module-4' in str(relative_path):
                part_number = 4
            elif 'hardware-lab' in str(relative_path):
                part_number = 0

            # Chunk the content
            chunks = chunk_text(content_without_frontmatter)

            for idx, chunk in enumerate(chunks):
                documents.append({
                    'chapter_id': chapter_id,
                    'section_id': f"{chapter_id}_chunk_{idx}",
                    'part_number': part_number,
                    'title': metadata.get('title', ''),
                    'description': metadata.get('description', ''),
                    'content': chunk,
                    'chunk_index': idx
                })

            print(f"✓ Parsed {chapter_id}: {len(chunks)} chunks")

        except Exception as e:
            print(f"✗ Error parsing {md_file}: {e}")

    return documents

def ingest_to_qdrant(documents: List[Dict[str, Any]]):
    """Ingest documents into Qdrant collection."""
    # Load environment variables
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if not qdrant_url or not qdrant_api_key:
        raise ValueError("QDRANT_URL and QDRANT_API_KEY must be set")

    # Initialize Qdrant client
    print(f"\nConnecting to Qdrant at {qdrant_url}...")
    client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)

    # Prepare texts for embedding
    texts = [doc['content'] for doc in documents]

    print(f"\nGenerating embeddings for {len(texts)} chunks...")
    embeddings = generate_embeddings(texts)

    print(f"✓ Generated {len(embeddings)} embeddings (768-dim)")

    # Prepare points for Qdrant
    points = []
    for idx, (doc, embedding) in enumerate(zip(documents, embeddings)):
        point = PointStruct(
            id=idx,
            vector=embedding,
            payload={
                'chapter_id': doc['chapter_id'],
                'section_id': doc['section_id'],
                'part_number': doc['part_number'],
                'title': doc['title'],
                'description': doc['description'],
                'content': doc['content'],
                'chunk_index': doc['chunk_index']
            }
        )
        points.append(point)

    # Upsert to Qdrant
    print(f"\nUpserting {len(points)} points to Qdrant...")
    client.upsert(
        collection_name="book_knowledge",
        points=points
    )

    print(f"✓ Successfully indexed {len(points)} chunks into Qdrant!")

def main():
    """Main ingestion workflow."""
    print("=" * 60)
    print("Content Ingestion Script")
    print("=" * 60)

    # Determine docs directory - fixed path for monorepo structure
    # From apps/api/scripts/ingest.py -> ../../docs/docs
    api_dir = os.path.dirname(os.path.dirname(__file__))  # apps/api
    apps_dir = os.path.dirname(api_dir)  # apps
    docs_dir = os.path.join(apps_dir, 'docs', 'docs')

    if not os.path.exists(docs_dir):
        print(f"✗ Docs directory not found: {docs_dir}")
        print(f"  Current script location: {__file__}")
        print(f"  Calculated docs path: {docs_dir}")
        return

    print(f"\nDocs directory: {docs_dir}")

    # Parse markdown files
    print("\n" + "=" * 60)
    print("Step 1: Parsing Markdown Files")
    print("=" * 60)
    documents = parse_markdown_files(docs_dir)

    if not documents:
        print("✗ No documents found to ingest")
        return

    print(f"\n✓ Parsed {len(documents)} total chunks from markdown files")

    # Ingest to Qdrant
    print("\n" + "=" * 60)
    print("Step 2: Ingesting to Qdrant")
    print("=" * 60)
    ingest_to_qdrant(documents)

    print("\n" + "=" * 60)
    print("Ingestion Complete!")
    print("=" * 60)

if __name__ == "__main__":
    main()
