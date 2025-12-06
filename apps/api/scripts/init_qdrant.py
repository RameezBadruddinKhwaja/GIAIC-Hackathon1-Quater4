"""
Qdrant Collection Initialization Script

Create and configure the book_knowledge collection in Qdrant Cloud.

Collection Configuration:
- Name: book_knowledge
- Vector Dimensions: 768 (for Gemini text-embedding-004)
- Distance Metric: Cosine
- Payload Schema:
  - chapter_id: str (index)
  - section_id: str
  - part_number: int
  - week_number: int
  - content_type: str (e.g., "prose", "code", "diagram")
  - hardware_context: str (e.g., "simulation", "edge", "cloud")

Usage:
    python apps/api/scripts/init_qdrant.py

Environment Variables:
    QDRANT_URL: Qdrant Cloud cluster URL
    QDRANT_API_KEY: Qdrant Cloud API key
"""

import os
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PayloadSchemaType
import sys

def init_qdrant_collection():
    """
    Initialize Qdrant book_knowledge collection with proper configuration.

    Returns:
        bool: True if successful, False otherwise
    """
    # Load environment variables
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if not qdrant_url or not qdrant_api_key:
        print("ERROR: QDRANT_URL and QDRANT_API_KEY environment variables must be set")
        print("See apps/api/.env.example for reference")
        return False

    try:
        # Initialize Qdrant client
        print(f"Connecting to Qdrant Cloud at {qdrant_url}...")
        client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)

        # Check if collection already exists
        collection_name = "book_knowledge"
        collections = client.get_collections().collections
        collection_exists = any(col.name == collection_name for col in collections)

        if collection_exists:
            print(f"Collection '{collection_name}' already exists.")
            response = input("Do you want to recreate it? (yes/no): ")
            if response.lower() in ['yes', 'y']:
                print(f"Deleting existing collection '{collection_name}'...")
                client.delete_collection(collection_name=collection_name)
            else:
                print("Skipping collection creation.")
                return True

        # Create collection with 768-dimensional vectors
        print(f"Creating collection '{collection_name}' with 768-dimensional vectors...")
        client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(
                size=768,  # Gemini text-embedding-004 dimensions
                distance=Distance.COSINE
            )
        )

        # Create payload indexes for efficient filtering
        print("Creating payload indexes...")
        client.create_payload_index(
            collection_name=collection_name,
            field_name="chapter_id",
            field_schema=PayloadSchemaType.KEYWORD
        )
        client.create_payload_index(
            collection_name=collection_name,
            field_name="part_number",
            field_schema=PayloadSchemaType.INTEGER
        )
        client.create_payload_index(
            collection_name=collection_name,
            field_name="week_number",
            field_schema=PayloadSchemaType.INTEGER
        )

        print(f"✓ Collection '{collection_name}' created successfully!")
        print(f"  - Vector dimensions: 768")
        print(f"  - Distance metric: Cosine")
        print(f"  - Payload indexes: chapter_id, part_number, week_number")

        return True

    except Exception as e:
        print(f"ERROR: Failed to initialize Qdrant collection: {e}")
        return False

if __name__ == "__main__":
    success = init_qdrant_collection()
    sys.exit(0 if success else 1)
