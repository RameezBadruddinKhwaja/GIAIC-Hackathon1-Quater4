# RAG Chatbot System - Setup and Usage

This document explains how to set up and use the newly rebuilt RAG (Retrieval-Augmented Generation) chatbot system.

## 🏗️ Architecture Overview

The chatbot system consists of:

1. **Vector Database (Qdrant Cloud)**: Stores embeddings of textbook content
2. **Content Ingestor**: Processes MDX files and generates embeddings
3. **Retrieval Service**: Searches for relevant content based on queries
4. **RAG Generator**: Uses OpenAI GPT-4o-mini to generate responses with context
5. **Chat API**: FastAPI endpoint for frontend communication
6. **ChatWidget**: React component for user interaction

## 📋 Prerequisites

### Required Environment Variables

Create or update your `.env` file in `apps/api/` with:

```bash
# OpenAI API Key (for embeddings and chat)
OPENAI_API_KEY=sk-...

# Qdrant Cloud Configuration
QDRANT_URL=https://your-cluster.cloud.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# Optional: Neon PostgreSQL (for chat history - future feature)
NEON_CONNECTION_STRING=postgresql://...
```

### Get Your API Keys

1. **OpenAI API Key**:
   - Go to https://platform.openai.com/api-keys
   - Create a new API key
   - Add credits to your account

2. **Qdrant Cloud**:
   - Sign up at https://cloud.qdrant.io/
   - Create a free cluster
   - Copy the cluster URL and API key

## 🚀 Setup Steps

### Step 1: Install Dependencies

```bash
cd apps/api
pip install -r requirements.txt
```

### Step 2: Ingest Content into Qdrant

Before using the chatbot, you need to populate the vector database with textbook content:

```bash
cd apps/api
python scripts/ingest_content.py
```

This will:
- Read all MDX files from `apps/docs/docs/`
- Extract and clean text content
- Generate embeddings using OpenAI
- Store chunks in Qdrant with metadata

**Expected Output:**
```
INFO - Starting ingestion from: /path/to/apps/docs/docs
INFO - Found X MDX/MD files
INFO - Ingesting file: ...
...
INFO - Ingestion complete: {'total_files': X, 'successful_files': Y, 'total_chunks': Z}
```

### Step 3: Start the API Server

```bash
cd apps/api
python -m uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

### Step 4: Test the Chatbot API

Open your browser and go to:
- API Docs: http://localhost:8000/docs
- Health Check: http://localhost:8000/api/health
- Chat Health: http://localhost:8000/api/chat/health

### Step 5: Start the Frontend

```bash
cd apps/docs
npm start
```

The ChatWidget should now appear in the bottom-right corner (only for authenticated users).

## 🧪 Testing the System

### Test via API Docs

1. Go to http://localhost:8000/docs
2. Find the `/api/chat/query` endpoint
3. Click "Try it out"
4. Enter a test query:

```json
{
  "query": "What is VSLAM?",
  "chapter_id": null,
  "conversation_history": null
}
```

5. Expected response:

```json
{
  "answer": "VSLAM (Visual Simultaneous Localization and Mapping)...",
  "citations": [
    {
      "title": "Isaac ROS",
      "module": "module-3",
      "week": "week-9",
      "chapter_url": "/docs/module-3/week-9-isaac-ros",
      "snippet": "..."
    }
  ],
  "model": "gpt-4o-mini",
  "context_used": 3
}
```

### Test via Frontend

1. Open http://localhost:3000
2. Log in (authentication required)
3. Click the floating chat button (bottom-right)
4. Ask a question: "Explain ROS 2 nodes"
5. Verify:
   - Response is relevant
   - Citations are displayed
   - Citations are clickable and navigate to correct chapters

## 📁 File Structure

```
apps/api/
├── src/
│   ├── db/
│   │   └── vector_store.py          # Qdrant client configuration
│   ├── services/
│   │   ├── embeddings.py            # OpenAI embeddings generation
│   │   ├── content_ingestor.py      # MDX content ingestion
│   │   ├── retriever.py             # Vector search service
│   │   └── rag_generator.py         # RAG response generation
│   ├── routers/
│   │   └── chat.py                  # Chat API endpoints
│   └── main.py                      # FastAPI app
└── scripts/
    └── ingest_content.py            # Content ingestion script

apps/docs/
└── src/
    └── components/
        └── ChatWidget/
            ├── index.tsx            # Chat UI component
            └── styles.module.css    # Styles
```

## 🔧 Configuration

### Chunking Parameters

Edit `apps/api/src/services/content_ingestor.py`:

```python
CHUNK_SIZE = 800        # Characters per chunk
CHUNK_OVERLAP = 200     # Overlap between chunks
```

### Retrieval Parameters

Edit `apps/api/src/services/retriever.py`:

```python
top_k = 5                    # Number of chunks to retrieve
score_threshold = 0.7        # Minimum similarity score
```

### Generation Parameters

Edit `apps/api/src/services/rag_generator.py`:

```python
CHAT_MODEL = "gpt-4o-mini"   # OpenAI model
MAX_TOKENS = 800             # Max response length
TEMPERATURE = 0.7            # Creativity (0-1)
```

## 🐛 Troubleshooting

### Issue: "Qdrant credentials not configured"

**Solution:** Ensure `QDRANT_URL` and `QDRANT_API_KEY` are set in `.env`

### Issue: "Collection 'textbook_content' not found"

**Solution:** Run the ingestion script first:
```bash
python scripts/ingest_content.py
```

### Issue: No relevant results found

**Possible causes:**
1. Content not ingested yet
2. Query is too different from textbook content
3. Score threshold too high (try lowering in `retriever.py`)

### Issue: ChatWidget not appearing

**Possible causes:**
1. User not authenticated
2. API URL not configured correctly in Docusaurus
3. Check browser console for errors

## 📊 Monitoring

### Check Qdrant Collection Stats

```python
from src.db.vector_store import get_vector_store, COLLECTION_NAME

client = get_vector_store()
collection_info = client.get_collection(COLLECTION_NAME)
print(f"Total vectors: {collection_info.points_count}")
```

### Check API Logs

The API logs will show:
- Query received
- Number of chunks retrieved
- Similarity scores
- Response generation time

## 🔄 Re-ingesting Content

If you update the textbook content, re-run the ingestion:

```bash
python scripts/ingest_content.py
```

This will upsert new content (existing chunks with same IDs will be updated).

## 🎯 Next Steps

1. **Add Chat History**: Store conversations in Neon PostgreSQL
2. **Add Text Selection**: Allow users to select text and ask questions about it
3. **Add Typing Indicators**: Stream responses from OpenAI
4. **Add Feedback**: Allow users to rate responses
5. **Add Analytics**: Track popular questions and citation usage

## 📚 Additional Resources

- [OpenAI Embeddings Guide](https://platform.openai.com/docs/guides/embeddings)
- [Qdrant Documentation](https://qdrant.tech/documentation/)
- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [RAG Best Practices](https://www.pinecone.io/learn/retrieval-augmented-generation/)
