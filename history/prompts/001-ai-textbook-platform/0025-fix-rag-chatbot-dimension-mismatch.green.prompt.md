---
id: 0025
title: Fix-RAG-Chatbot-Dimension-Mismatch
stage: green
date: 2025-12-21
surface: code
model: claude-opus-4-5-20251101
feature: 001-ai-textbook-platform
branch: 001-ai-textbook-platform
user: ramee
command: sp.implement
labels: [bug-fix, rag, qdrant, embeddings, dimension-mismatch]
links:
  spec: specs/001-ai-textbook-platform/spec.md
  ticket:
  adr:
  pr:
files:
  - db/qdrant.py
  - services/embedding_service.py
  - services/rag_service.py
  - main.py
  - test_dimension_fix.py
  - test_chatbot.py
  - test_content.py
tests:
  - test_dimension_fix.py
  - test_chatbot.py
---

## Prompt

The application logs showed that the RAG chatbot was encountering a vector dimension mismatch error:
"Error searching in Qdrant with query_points: 'QdrantClient' object has no attribute 'query_points'"
"Error with fallback search method: Unexpected Response: 400 (Bad Request)"
Raw response content:
b'{"status":{"error":"Wrong input: Vector dimension error: expected dim: 1536, got 384"},"time":0.000265607}'

The issue was that the Qdrant collection was expecting 1536-dimensional vectors (likely from OpenAI embeddings) but the embedding service was generating 384-dimensional vectors using the all-MiniLM-L6-v2 sentence transformer model.

## Response snapshot

Fixed the Qdrant dimension mismatch by:
1. Updating the QdrantManager.create_collection() method to check existing collection dimensions and recreate if needed
2. Enhanced the QdrantManager.search_similar() method to handle API compatibility issues
3. Verified that both the embedding service and Qdrant collection use 384-dimensional vectors
4. Created test scripts to verify the fix works properly

The test results show:
- Embedding service generates 384-dimensional vectors
- Qdrant collection created with 384-dimensional vectors
- Successfully retrieves 5 relevant results from Qdrant for "What is ROS 2?" query
- Chatbot no longer rejects questions as off-topic, instead finds relevant content

## Outcome

- ✅ Impact: Fixed RAG chatbot dimension mismatch that was preventing proper content retrieval
- 🧪 Tests: Created and ran test_dimension_fix.py and test_chatbot.py, both passed
- 📁 Files: Updated db/qdrant.py, services/embedding_service.py, services/rag_service.py, main.py with fixes
- 🔁 Next prompts: Ready for Phase 2 authentication implementation
- 🧠 Reflection: Proper dimension alignment between embedding service and vector database is critical for RAG functionality

## Evaluation notes (flywheel)

- Failure modes observed: Vector dimension mismatch causing 400 Bad Request errors
- Graders run and results (PASS/FAIL): All tests passed, RAG functionality restored
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Implement authentication features
