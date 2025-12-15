# Quickstart Guide: Physical AI & Humanoid Robotics Textbook Platform

**Feature**: 001-ai-textbook-platform
**Date**: 2025-12-05
**Phase**: Phase 1 - Local Development Setup

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Environment Setup](#environment-setup)
3. [Database Setup (Neon PostgreSQL)](#database-setup-neon-postgresql)
4. [Vector Store Setup (Qdrant Cloud)](#vector-store-setup-qdrant-cloud)
5. [Backend Setup (FastAPI)](#backend-setup-fastapi)
6. [Frontend Setup (Docusaurus)](#frontend-setup-docusaurus)
7. [Agentic Infrastructure Setup (.claude/)](#agentic-infrastructure-setup-claude)
8. [Running the Application](#running-the-application)
9. [Testing](#testing)
10. [Troubleshooting](#troubleshooting)

---

## Prerequisites

Ensure you have the following installed on your system:

| Tool | Version | Installation |
|------|---------|--------------|
| **Node.js** | 18+ | [Download](https://nodejs.org/) |
| **Python** | 3.11+ | [Download](https://www.python.org/) |
| **npm** | 9+ | Included with Node.js |
| **Git** | Latest | [Download](https://git-scm.com/) |
| **Docker** | Latest (Optional for local Qdrant) | [Download](https://www.docker.com/) |

**Operating Systems**: Linux (Ubuntu 20.04+), macOS (Ventura+), Windows 11 with WSL2

---

## Environment Setup

### 1. Clone the Repository

```bash
git clone https://github.com/RameezBadruddinKhwaja/GIAIC-Hackathon1-Quater4.git
cd GIAIC-Hackathon1-Quater4
```

### 2. Checkout Feature Branch

```bash
git checkout 001-ai-textbook-platform
```

### 3. Install Dependencies

**Frontend (Docusaurus)**:
```bash
cd apps/docs
npm install
cd ../..
```

**Backend (FastAPI)**:
```bash
cd apps/api
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
cd ../..
```

---

## Database Setup (Neon PostgreSQL)

### 1. Create Neon Account

1. Visit [Neon Console](https://console.neon.tech/)
2. Sign up with GitHub OAuth
3. Create a new project: **"physical-ai-textbook-platform"**
4. Select region: **US East (Ohio)**
5. Copy the connection string (format: `postgres://user:pass@host/dbname?sslmode=require`)

### 2. Configure Environment Variables

Create `.env` file in `apps/api/`:

```bash
# apps/api/.env
NEON_CONNECTION_STRING=postgresql://user:pass@ep-example.us-east-2.aws.neon.tech/main?sslmode=require
DATABASE_URL=${NEON_CONNECTION_STRING}
```

### 3. Run Database Migrations

```bash
cd apps/api
source venv/bin/activate
alembic upgrade head
cd ../..
```

**Expected Output**:
```
INFO  [alembic.runtime.migration] Context impl PostgresqlImpl.
INFO  [alembic.runtime.migration] Will assume transactional DDL.
INFO  [alembic.runtime.migration] Running upgrade  -> 001_create_users, Create users table
INFO  [alembic.runtime.migration] Running upgrade 001_create_users -> 002_create_chat_logs, Create chat_logs table
...
```

### 4. Verify Tables

```bash
psql $NEON_CONNECTION_STRING -c "\dt"
```

**Expected Tables**: `users`, `chat_logs`, `personalized_content`, `translated_content`, `audit_logs`

---

## Vector Store Setup (Qdrant Cloud)

### 1. Create Qdrant Cloud Account

1. Visit [Qdrant Cloud](https://cloud.qdrant.io/)
2. Sign up with email
3. Create a new cluster: **"book-knowledge-cluster"**
4. Select plan: **Free Tier** (1GB storage, 1M vectors)
5. Copy API key and cluster URL

### 2. Configure Environment Variables

Add to `apps/api/.env`:

```bash
# apps/api/.env (continued)
QDRANT_URL=https://xyz123-example.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-api-key-here
```

### 3. Initialize Qdrant Collection

```bash
cd apps/api
source venv/bin/activate
python scripts/init_qdrant.py
cd ../..
```

**Expected Output**:
```
[INFO] Connecting to Qdrant Cloud at https://xyz123-example.qdrant.io:6333
[INFO] Creating collection: book_knowledge
[INFO] Collection created successfully with 768-dimensional vectors (Cosine distance) for text-embedding-004
[INFO] Payload schema configured: chapter_id, section_id, part_number, week_number, content_type, hardware_context
```

---

## Backend Setup (FastAPI)

### 1. Configure Additional Environment Variables

Complete `apps/api/.env`:

```bash
# apps/api/.env (continued)
GEMINI_API_KEY=your-gemini-api-key-here
BETTER_AUTH_GITHUB_CLIENT_ID=your-github-oauth-client-id
BETTER_AUTH_GITHUB_CLIENT_SECRET=your-github-oauth-client-secret
JWT_SECRET=generate-a-strong-random-secret-here
ENVIRONMENT=development
```

**Generate JWT Secret**:
```bash
python -c "import secrets; print(secrets.token_urlsafe(32))"
```

### 2. Start FastAPI Development Server

```bash
cd apps/api
source venv/bin/activate
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

**Expected Output**:
```
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [12345] using StatReload
INFO:     Started server process [12346]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

### 3. Verify API Health

Open browser: [http://localhost:8000/api/health](http://localhost:8000/api/health)

**Expected JSON Response**:
```json
{
  "status": "healthy",
  "services": {
    "neon_db": "connected",
    "qdrant_cloud": "connected",
    "gemini_api": "available",
    "better_auth": "operational"
  }
}
```

---

## Frontend Setup (Docusaurus)

### 1. Configure Environment Variables

Create `.env` file in `apps/docs/`:

```bash
# apps/docs/.env
REACT_APP_API_URL=http://localhost:8000
REACT_APP_BETTER_AUTH_GITHUB_CLIENT_ID=your-github-oauth-client-id
```

### 2. Start Docusaurus Development Server

```bash
cd apps/docs
npm start
```

**Expected Output**:
```
[INFO] Starting the development server...
[SUCCESS] Docusaurus website is running at: http://localhost:3000/

✔ Client
  Compiled successfully in 3.21s

ℹ ｢wds｣: Project is running at http://localhost:3000/
```

### 3. Verify Docusaurus Site

Open browser: [http://localhost:3000](http://localhost:3000)

**Expected Page**: Homepage with 4 course modules (The Nervous System, The Digital Twin, The Brain, VLA & Humanoids)

---

## Agentic Infrastructure Setup (.claude/)

### 1. Create Specialized Agents

**Directory Structure**:
```
.claude/
├── agents/
│   ├── author.md       # Physics & Robotics Professor (Content Creator)
│   ├── coder.md        # Full-Stack Engineer (RAG/FastAPI/React)
│   ├── architect.md    # System Designer (Folder Structure/ADRs)
│   ├── qa.md           # SOC Analyst & Tester (Security/Sanitization)
│   └── translator.md   # Urdu Localization Expert (Bonus Feature)
└── skills/
    ├── ros2-mastery.md        # Deep knowledge of Nodes/Topics
    ├── docusaurus-guru.md     # Mastery of Admonitions, Tabs, Swizzling
    ├── matrix-loader.md       # Logic to dynamically load other skills
    └── better-auth.md         # Implementation guide for Auth bonus
```

### 2. Create Agent Definitions

**Example: `.claude/agents/author.md`**:
```markdown
# Author Agent: Physics & Robotics Professor

**Role**: Content Creator for Physical AI Textbook

**Expertise**:
- ROS 2 architecture (Nodes, Topics, Services, URDF)
- NVIDIA Isaac Sim and Isaac ROS
- Gazebo and Unity simulation
- Vision-Language-Action (VLA) systems
- Jetson Orin Nano edge deployment

**Responsibilities**:
- Write technical chapters with Mermaid diagrams
- Create code examples with "Simulated" vs "Real Robot" tabs
- Verify technical accuracy using Context7 (Constitution Article III)
- Use Docusaurus admonitions (:::tip, :::warning) for hardware alerts

**Language Protocol**: Professional US English for all content
```

**Create all 5 agents** (`author.md`, `coder.md`, `architect.md`, `qa.md`, `translator.md`) based on `research.md` section 12.

### 3. Create Skill Definitions

**Example: `.claude/skills/ros2-mastery.md`**:
```markdown
# Skill: ROS 2 Mastery

**Domain**: Robot Operating System 2 (ROS 2)

**Knowledge Base**:
- ROS 2 Humble distribution
- Nodes: Independent processes that perform computation
- Topics: Named buses for message passing (pub/sub pattern)
- Services: Request/response pattern for synchronous communication
- URDF: XML format for robot description (links, joints, sensors)

**When to Load**: Detect keywords in chatbot queries: "ROS 2", "Nodes", "Topics", "Services", "URDF", "pub/sub"

**Usage in Matrix Protocol**:
```python
if "ROS 2" in query or "Topics" in query:
    load_skill("ros2-mastery")
    print("Matrix Skill Loaded: ros2-mastery")
```

**Citations**: Always cite ROS 2 official docs (verified via Context7)
```

**Create all 4 skills** (`ros2-mastery.md`, `docusaurus-guru.md`, `matrix-loader.md`, `better-auth.md`) based on `research.md` section 12.

---

## Running the Application

### Option 1: Parallel Terminals (Recommended for Development)

**Terminal 1 - Backend**:
```bash
cd apps/api
source venv/bin/activate
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

**Terminal 2 - Frontend**:
```bash
cd apps/docs
npm start
```

### Option 2: Docker Compose (Production-like)

```bash
docker-compose up --build
```

**Services**:
- Docusaurus: `http://localhost:3000`
- FastAPI: `http://localhost:8000`
- FastAPI Docs: `http://localhost:8000/docs`

---

## Testing

### Backend Tests (Pytest)

```bash
cd apps/api
source venv/bin/activate
pytest tests/ -v
```

**Expected Output**:
```
============================= test session starts ==============================
collected 24 items

tests/test_auth.py::test_signup_success PASSED                           [  4%]
tests/test_auth.py::test_signin_success PASSED                           [  8%]
tests/test_chat.py::test_chat_query_success PASSED                       [ 12%]
tests/test_chat.py::test_chat_sanitization PASSED                        [ 16%]
tests/test_personalize.py::test_personalize_rtx4090 PASSED               [ 20%]
tests/test_personalize.py::test_personalize_jetson PASSED                [ 24%]
tests/test_translate.py::test_translate_roman_urdu PASSED                [ 28%]
...

========================= 24 passed in 12.34s ==============================
```

### Frontend E2E Tests (Playwright)

```bash
cd apps/docs
npx playwright test
```

**Expected Output**:
```
Running 8 tests using 4 workers

  ✓  [chromium] › test_chat_widget.spec.ts:5:1 › Chat Widget › can send message (3s)
  ✓  [chromium] › test_chat_widget.spec.ts:12:1 › Chat Widget › displays citations (2s)
  ✓  [chromium] › test_personalize_button.spec.ts:5:1 › Personalize Button › shows for authenticated users (1s)
  ✓  [chromium] › test_urdu_translation.spec.ts:5:1 › Urdu Translation › translates content (4s)
  ...

  8 passed (18s)
```

### Contract Testing

Verify API matches OpenAPI schema:

```bash
cd apps/api
pytest tests/test_contract.py -v
```

---

## Troubleshooting

### Issue: Neon Connection Timeout

**Symptoms**: `psycopg2.OperationalError: could not connect to server`

**Solutions**:
1. Verify Neon cluster is active (check Neon Console)
2. Check firewall/VPN blocking port 5432
3. Ensure `sslmode=require` in connection string

### Issue: Qdrant Collection Not Found

**Symptoms**: `QdrantException: Collection 'book_knowledge' not found`

**Solutions**:
1. Re-run `python scripts/init_qdrant.py`
2. Verify Qdrant API key is correct
3. Check Qdrant Cloud cluster status

### Issue: Gemini API Rate Limit

**Symptoms**: `openai.error.RateLimitError: Rate limit exceeded` (error raised by OpenAI SDK when using Gemini backend)

**Solutions**:
1. Implement exponential backoff in `apps/api/services/gemini_client.py`
2. Use caching aggressively (`personalized_content`, `translated_content` tables) to minimize API calls
3. Verify GEMINI_API_KEY is valid and has quota available
4. Note: Gemini free tier has generous limits but may throttle during high usage

### Issue: Docusaurus Build Errors

**Symptoms**: `ERROR in Entry module not found`

**Solutions**:
1. Clear cache: `cd apps/docs && npm run clear && npm start`
2. Delete `node_modules` and reinstall: `rm -rf node_modules && npm install`
3. Check for syntax errors in MDX files

### Issue: Better-Auth GitHub OAuth Fails

**Symptoms**: `Error: Invalid client_id`

**Solutions**:
1. Verify GitHub OAuth App credentials at [GitHub Developer Settings](https://github.com/settings/developers)
2. Ensure callback URL is `http://localhost:3000/api/auth/callback/github`
3. Check `BETTER_AUTH_GITHUB_CLIENT_ID` and `BETTER_AUTH_GITHUB_CLIENT_SECRET` in `.env`

---

## Next Steps

1. **Content Ingestion**: Run `python apps/api/scripts/ingest.py` to index Docusaurus chapters into Qdrant
2. **Agent Context Update**: Run `.specify/scripts/bash/update-agent-context.sh claude` to register technologies in agent context
3. **Task Generation**: Run `/sp.tasks` to create implementation tasks based on this plan
4. **Implementation**: Execute tasks in `specs/001-ai-textbook-platform/tasks.md`

---

## Additional Resources

- **Docusaurus Docs**: https://docusaurus.io/docs
- **FastAPI Docs**: https://fastapi.tiangolo.com/
- **Neon Docs**: https://neon.tech/docs
- **Qdrant Docs**: https://qdrant.tech/documentation/
- **OpenAI Agents SDK**: https://github.com/openai/openai-agents-sdk
- **Better-Auth Docs**: https://better-auth.com/docs

---

**Quickstart Complete**: You are now ready to develop the Physical AI Textbook Platform locally! 🚀

For issues, consult `specs/001-ai-textbook-platform/research.md` or open a GitHub issue.
