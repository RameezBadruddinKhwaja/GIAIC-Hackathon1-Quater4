#!/usr/bin/env python3
"""
Content Ingestion Script

Ingests all MDX files from the docs directory into Qdrant.
Run this script to populate the vector database with textbook content.

Usage:
    python scripts/ingest_content.py
"""

import sys
import os
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.services.content_ingestor import run_ingestion
from dotenv import load_dotenv
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

logger = logging.getLogger(__name__)

def main():
    """Run content ingestion"""
    # Load environment variables
    load_dotenv()

    # Verify environment variables
    required_vars = ['GEMINI_API_KEY', 'QDRANT_URL', 'QDRANT_API_KEY']  # Changed from OPENAI_API_KEY to GEMINI_API_KEY
    missing_vars = [var for var in required_vars if not os.getenv(var)]

    if missing_vars:
        logger.error(f"Missing required environment variables: {', '.join(missing_vars)}")
        logger.error("Please set these in your .env file")
        return 1

    # Determine docs directory
    # Assuming structure: apps/api/scripts/ingest_content.py
    # Docs are at: apps/docs/docs/
    api_dir = Path(__file__).parent.parent
    docs_dir = api_dir.parent / "docs" / "docs"

    if not docs_dir.exists():
        logger.error(f"Docs directory not found: {docs_dir}")
        logger.error("Please ensure the docs directory exists with MDX files")
        return 1

    logger.info(f"Starting content ingestion from: {docs_dir}")
    logger.info("This may take several minutes depending on the number of files...")

    try:
        # Run ingestion
        stats = run_ingestion(str(docs_dir))

        # Print results
        logger.info("=" * 60)
        logger.info("INGESTION COMPLETE")
        logger.info("=" * 60)
        logger.info(f"Total MDX files found: {stats['total_files']}")
        logger.info(f"Successfully ingested: {stats['successful_files']}")
        logger.info(f"Total chunks created: {stats['total_chunks']}")
        logger.info("=" * 60)

        if stats['successful_files'] == 0:
            logger.warning("No files were successfully ingested. Check the logs for errors.")
            return 1

        logger.info("✅ Content is now available for RAG queries!")
        return 0

    except Exception as e:
        logger.error(f"Ingestion failed: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
