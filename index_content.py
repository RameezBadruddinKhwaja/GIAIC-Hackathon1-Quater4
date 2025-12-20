#!/usr/bin/env python3
"""
Content Indexing Script for Physical AI & Humanoid Robotics Textbook

This script reads all MDX files from the docs directory, chunks them,
generates embeddings, and stores them in Qdrant vector database.
"""
import os
import re
import sys
import json
import uuid
from pathlib import Path
from typing import List, Dict, Any
from dotenv import load_dotenv

# Add the api directory to path to import services
sys.path.insert(0, str(Path(__file__).parent))

from apps.api.services.indexing_service import indexing_service

load_dotenv()

def extract_text_from_mdx(content: str) -> str:
    """
    Extract clean text from MDX content.

    Removes:
    - JSX/React components
    - Code blocks (preserves inline code)
    - Front matter
    - HTML tags
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

def extract_chapter_metadata(file_path: Path, docs_dir: Path) -> Dict[str, Any]:
    """
    Extract metadata from file path and content.

    Expected structure: docs/docs/module-X/week-Y-title.mdx
    """
    relative_path = file_path.relative_to(docs_dir)
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
    metadata['url'] = f"/docs/{url_path}"

    return metadata

def read_all_mdx_files(docs_dir: str) -> List[Dict[str, Any]]:
    """
    Read all MDX files from the docs directory and extract content with metadata.
    """
    docs_path = Path(docs_dir)
    mdx_files = list(docs_path.rglob("*.mdx")) + list(docs_path.rglob("*.md"))

    chapters = []
    for file_path in mdx_files:
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract text
            text = extract_text_from_mdx(content)

            if len(text) < 100:  # Skip very small files
                print(f"Skipping small file: {file_path}")
                continue

            # Extract metadata
            metadata = extract_chapter_metadata(file_path, docs_path)

            chapter = {
                'title': metadata['title'],
                'content': text,
                'module': metadata['module'],
                'section': metadata['week'],
                'url': metadata['url'],
                'file_path': str(file_path)
            }

            chapters.append(chapter)
            print(f"Processed: {chapter['title']} ({len(text)} chars)")

        except Exception as e:
            print(f"Error processing {file_path}: {e}")
            continue

    return chapters

def main():
    """
    Main function to index all book content into Qdrant.
    """
    print("Starting content indexing process...")

    # Set up paths
    docs_dir = Path(__file__).parent / "apps" / "docs" / "docs"

    if not docs_dir.exists():
        print(f"Docs directory not found: {docs_dir}")
        return 1

    print(f"Reading content from: {docs_dir}")

    # Read all MDX files
    print("Reading MDX files...")
    chapters = read_all_mdx_files(str(docs_dir))

    if not chapters:
        print("No chapters found to index")
        return 1

    print(f"Found {len(chapters)} chapters to index")

    # Index the content
    print("Indexing content to Qdrant...")
    result = indexing_service.index_book_content(chapters)

    if result["status"] == "success":
        print(f"[SUCCESS] Successfully processed {result.get('chunks_processed', 0)} chunks from book content")
        print("[INFO] Content processing completed!")
        return 0
    else:
        print(f"[ERROR] Error indexing content: {result['message']}")
        return 1

if __name__ == "__main__":
    sys.exit(main())