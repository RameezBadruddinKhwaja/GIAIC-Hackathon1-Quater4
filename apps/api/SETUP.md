# FastAPI Backend Setup Guide

Physical AI & Humanoid Robotics Textbook Platform - Backend API

---

## 📋 Prerequisites

- Python 3.12+
- PostgreSQL database (Neon recommended)
- Qdrant vector database (Cloud or local)
- OpenAI API key OR Gemini API key
- GitHub OAuth app (for authentication)

---

## 🚀 Quick Start (Local Development)

### 1. Install Dependencies

```bash
cd apps/api

# Create virtual environment
python -m venv .venv

# Activate virtual environment
# On Windows:
.venv\Scripts\activate
# On Linux/Mac:
source .venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

### 2. Configure Environment Variables

Copy `.env.example` to `.env`:

```bash
cp .env.example .env
```

Edit `.env` and set your actual values:

```env
# AI API (choose one)
GEMINI_API_KEY=your_gemini_api_key_here
# OR
OPENAI_API_KEY=your_openai_api_key_here

# Database
NEON_CONNECTION_STRING=postgresql://user:pass@host/db?sslmode=require

# Vector Store
QDRANT_URL=https://your-cluster.cloud.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key

# Authentication
BETTER_AUTH_GITHUB_CLIENT_ID=your_github_client_id
BETTER_AUTH_GITHUB_CLIENT_SECRET=your_github_client_secret
JWT_SECRET=your_random_secret_min_32_chars

# Environment
ENVIRONMENT=development

# URLs
API_URL=http://localhost:8000
FRONTEND_URL=http://localhost:3000
```

### 3. Initialize Database

```bash
# Run migrations
alembic upgrade head
```

### 4. (Optional) Ingest Chapter Embeddings

```bash
# Generate embeddings for all chapters
python scripts/ingest_content.py
```

### 5. Start Development Server

```bash
# Run with auto-reload
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

Server will be running at: http://localhost:8000

API Documentation: http://localhost:8000/docs

---

## 📁 Project Structure

```
apps/api/
├── src/
│   ├── main.py              # FastAPI app entry point
│   ├── core/
│   │   ├── database.py      # Neon PostgreSQL connection
│   │   └── config.py        # Configuration management
│   ├── models/              # SQLAlchemy models
│   │   ├── user.py
│   │   └── chat_log.py
│   ├── routers/             # API endpoints
│   │   ├── auth.py          # Authentication
│   │   ├── chat.py          # RAG chatbot
│   │   ├── personalize.py   # Content personalization
│   │   └── translate.py     # Urdu translation
│   ├── services/            # Business logic
│   │   ├── rag_generator.py     # RAG response generation
│   │   ├── retriever.py         # Vector search
│   │   ├── embeddings.py        # Embedding generation
│   │   └── content_ingestor.py  # Content ingestion
│   └── db/
│       └── vector_store.py  # Qdrant client
├── migrations/              # Alembic migrations
├── scripts/                 # Utility scripts
│   └── ingest_content.py    # Embed and store chapters
├── requirements.txt         # Python dependencies
├── .env                     # Environment variables (DO NOT COMMIT)
├── .env.example             # Environment template
├── vercel.json              # Vercel deployment config
└── SETUP.md                 # This file
```

---

## 🔌 API Endpoints

### Health Check
```bash
GET /api/health
```

### RAG Chatbot
```bash
POST /api/chat/query
Content-Type: application/json

{
  "query": "What is ROS 2?",
  "chapter_id": null,
  "conversation_history": []
}
```

### Authentication
```bash
POST /api/auth/signup
POST /api/auth/login
GET  /api/auth/github
GET  /api/auth/github/callback
POST /api/auth/logout
GET  /api/auth/me
```

### Personalization
```bash
POST /api/personalize/apply
```

### Translation
```bash
POST /api/translate/to-urdu
```

---

## 🧪 Testing

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=src tests/

# Run specific test file
pytest tests/test_chat.py
```

---

## 🐳 Docker (Optional)

```bash
# Build image
docker build -t textbook-api .

# Run container
docker run -p 8000:8000 --env-file .env textbook-api
```

---

## 📊 Database Migrations

### Create a new migration

```bash
alembic revision -m "description of changes"
```

### Apply migrations

```bash
alembic upgrade head
```

### Rollback migration

```bash
alembic downgrade -1
```

---

## 🔍 Debugging

### Check logs

```bash
# View uvicorn logs
tail -f logs/api.log
```

### Test database connection

```python
from src.core.database import engine
import asyncio

async def test_db():
    async with engine.begin() as conn:
        result = await conn.execute("SELECT 1")
        print("Database connection OK:", result.scalar())

asyncio.run(test_db())
```

### Test Qdrant connection

```python
from src.db.vector_store import get_qdrant_client

client = get_qdrant_client()
collections = client.get_collections()
print("Qdrant collections:", collections)
```

---

## 🚢 Deployment

### Vercel (Recommended)

See `VERCEL_DEPLOYMENT_CHECKLIST.md` for detailed instructions.

**Quick steps**:
1. Set all environment variables in Vercel Dashboard
2. Push to `main` branch (auto-deploys)
3. Verify deployment at your Vercel URL

### Manual Server Deployment

```bash
# Install dependencies
pip install -r requirements.txt

# Run with gunicorn
gunicorn src.main:app -w 4 -k uvicorn.workers.UvicornWorker -b 0.0.0.0:8000
```

---

## 🔐 Security

### Best Practices

1. **Never commit `.env` file**
   - Always use `.env.example` as template
   - Use environment variables in production

2. **Rotate API keys regularly**
   - Gemini/OpenAI keys
   - Database credentials
   - JWT secrets

3. **Enable CORS only for trusted origins**
   - Update `allow_origins` in `main.py`

4. **Use HTTPS in production**
   - Never use HTTP for API in production

5. **Validate all inputs**
   - Use Pydantic models
   - Sanitize user queries (already implemented)

---

## 📚 Additional Resources

- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [SQLAlchemy Async](https://docs.sqlalchemy.org/en/20/orm/extensions/asyncio.html)
- [Qdrant Documentation](https://qdrant.tech/documentation/)
- [Neon PostgreSQL](https://neon.tech/docs)
- [OpenAI API](https://platform.openai.com/docs)

---

## 🐛 Troubleshooting

### Import Errors

**Error**: `ModuleNotFoundError: No module named 'fastapi'`

**Solution**:
```bash
pip install -r requirements.txt
```

### Database Connection Errors

**Error**: `asyncpg.exceptions.InvalidPasswordError`

**Solution**:
- Check `NEON_CONNECTION_STRING` is correct
- Verify database is accessible
- Ensure SSL mode is set correctly

### Qdrant Connection Errors

**Error**: `qdrant_client.http.exceptions.UnexpectedResponse: 403`

**Solution**:
- Verify `QDRANT_API_KEY` is correct
- Check Qdrant cluster is active
- Ensure URL includes protocol (`https://`)

### Environment Variable Errors

**Error**: `ValueError: GEMINI_API_KEY must be set`

**Solution**:
- Verify `.env` file exists in `apps/api/`
- Check variable names match exactly
- Restart server after changing `.env`

---

## 💡 Tips

- Use `--reload` flag during development for auto-restart on code changes
- Check `/docs` endpoint for interactive API documentation
- Use Pydantic models for request/response validation
- Leverage FastAPI's dependency injection for database sessions
- Use async functions for all database and external API calls

---

**Last Updated**: 2025-12-18
**Maintainer**: GIAIC Hackathon Team
