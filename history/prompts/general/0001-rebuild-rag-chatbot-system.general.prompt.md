---
id: 0001
title: Rebuild RAG Chatbot System
stage: general
date: 2025-12-17
surface: agent
model: claude-sonnet-4.5
feature: 001-ai-textbook-platform
branch: 001-ai-native-textbook-platform
user: system
command: /sp.implement
labels: ["rag", "chatbot", "openai", "qdrant", "rebuild", "vectordb"]
links:
  spec: .specify/specs/001-ai-textbook-platform/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - apps/api/src/db/vector_store.py
  - apps/api/src/services/embeddings.py
  - apps/api/src/services/content_ingestor.py
  - apps/api/src/services/retriever.py
  - apps/api/src/services/rag_generator.py
  - apps/api/src/routers/chat.py
  - apps/api/scripts/ingest_content.py
  - apps/docs/src/components/ChatWidget/index.tsx
  - apps/docs/src/components/ChatWidget/styles.module.css
  - apps/api/README_CHATBOT.md
tests:
  - Manual API testing via /docs endpoint
  - Frontend ChatWidget UI testing
  - End-to-end RAG pipeline validation
---

## Prompt

remove all the chatbot code and make it from scratch because it is not working whatever we do

## Response snapshot

Successfully rebuilt the entire RAG chatbot system from scratch with a clean architecture:

**Backend (FastAPI):**
1. **Vector Store** (`vector_store.py`): Qdrant Cloud integration with singleton pattern, collection management, 1536-dimension vectors for OpenAI embeddings
2. **Embeddings Service** (`embeddings.py`): OpenAI text-embedding-3-small integration for generating embeddings
3. **Content Ingestor** (`content_ingestor.py`): MDX file processing, text extraction, chunking (800 chars with 200 overlap), batch embedding generation, Qdrant upsert
4. **Retriever** (`retriever.py`): Vector similarity search, chapter-specific filtering, score thresholding (0.7), metadata extraction
5. **RAG Generator** (`rag_generator.py`): OpenAI GPT-4o-mini integration, context-aware prompting, citation extraction, conversation history support
6. **Chat Router** (`chat.py`): FastAPI endpoint `/api/chat/query`, request validation, XSS prevention, comprehensive error handling

**Frontend (React/Docusaurus):**
7. **ChatWidget** (`index.tsx`): Floating chat button, expandable window, message history, citation display, loading states, authentication-gated
8. **Styles** (`styles.module.css`): Modern gradient UI, responsive design, dark mode support, smooth animations

**Infrastructure:**
9. **Ingestion Script** (`ingest_content.py`): Standalone script to populate Qdrant with all textbook content
10. **Documentation** (`README_CHATBOT.md`): Complete setup guide, API configuration, troubleshooting, testing instructions

**Key Features:**
- Clean separation of concerns (embeddings, retrieval, generation)
- Proper error handling and logging throughout
- Conversation history support for context
- Citation extraction with chapter metadata
- Authentication-gated UI (only shown to logged-in users)
- Health check endpoints
- Comprehensive documentation

**Removed Old Code:**
- Deleted all previous chatbot files (rag_pipeline.py, matrix_loader.py, gemini_client.py, old ChatWidget)
- Updated main.py to use new router structure

## Outcome

- ✅ Impact: Complete RAG chatbot system rebuilt with clean architecture, proper error handling, and comprehensive documentation
- 🧪 Tests: Manual API testing required, frontend UI validation needed, content ingestion script ready to run
- 📁 Files: 10 new files created, 6 old files removed, main.py updated
- 🔁 Next prompts: Run content ingestion script, test RAG pipeline end-to-end, verify citations are clickable, test with various queries
- 🧠 Reflection: Rebuilding from scratch allowed for better architecture design. Separated concerns clearly (embeddings → retrieval → generation). Used OpenAI's recommended patterns. Need to test with actual textbook content to validate chunking strategy and retrieval quality.

## Evaluation notes (flywheel)

- Failure modes observed: Previous implementation had tightly coupled services, unclear error handling, Matrix Protocol complexity. New implementation avoids these by using simple, focused services.
- Graders run and results (PASS/FAIL): N/A - awaiting manual testing after content ingestion
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Run ingestion script with actual textbook content, measure retrieval quality (precision/recall), adjust CHUNK_SIZE and score_threshold based on results
