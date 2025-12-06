# FastAPI Backend - AI Textbook Platform

Backend API for the Physical AI & Humanoid Robotics Textbook Platform.

## Environment Setup

### Gemini API Key

This project uses **Google Gemini 2.5 Flash** via the OpenAI SDK (drop-in replacement strategy).

**To obtain your API key:**

1. Visit [Google AI Studio](https://makersuite.google.com/app/apikey)
2. Sign in with your Google account
3. Click "Create API Key"
4. Copy the key and add it to `.env` as `GEMINI_API_KEY`

**IMPORTANT**: Use `GEMINI_API_KEY` (NOT `OPENAI_API_KEY`) in environment variables.

### Configuration

The OpenAI SDK is configured with a custom base URL to route requests to Gemini:

```python
from openai import OpenAI

client = OpenAI(
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
    api_key=os.getenv("GEMINI_API_KEY")
)
```

**Model Names:**
- Chat completion: `gemini-2.5-flash`
- Embeddings: `text-embedding-004` (768-dimensional)

## API Endpoints

- `POST /api/auth/signup` - Create user account (email/password)
- `POST /api/auth/signin` - Authenticate user (email/GitHub OAuth)
- `POST /api/auth/onboarding` - Save user hardware profile
- `POST /api/chat` - RAG chatbot with cited responses
- `POST /api/personalize` - Hardware-aware content personalization
- `POST /api/translate` - Urdu translation with code preservation
- `GET /api/health` - Health check

## Local Development

1. Create `.env` from `.env.example`
2. Install dependencies: `pip install -r requirements.txt`
3. Run migrations: `python scripts/migrate_db.py`
4. Start server: `uvicorn src.main:app --reload`

See [quickstart.md](../../specs/001-ai-textbook-platform/quickstart.md) for detailed setup instructions.

## Tech Stack

- **Framework**: FastAPI
- **Database**: Neon (PostgreSQL) via SQLAlchemy
- **Vector Store**: Qdrant Cloud
- **AI**: Google Gemini 2.5 Flash via OpenAI SDK
- **Auth**: Better-Auth
