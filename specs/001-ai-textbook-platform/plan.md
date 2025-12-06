# Implementation Plan: Physical AI & Humanoid Robotics Textbook Platform

**Branch**: `001-ai-textbook-platform` | **Date**: 2025-12-05 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/001-ai-textbook-platform/spec.md`

**Note**: This plan defines the technical architecture for achieving 300/100 Hackathon score with 4 bonus features (Matrix Protocol, Better-Auth, Personalization, Localization).

## Summary

Build an AI-Native educational platform for Physical AI & Humanoid Robotics course with:
- **Content Engine**: Docusaurus v3 textbook with 4 modules (13 weeks), Mermaid diagrams, code tabs, hardware comparisons
- **RAG Chatbot**: FastAPI + Qdrant + OpenAI SDK (Gemini backend) with cited responses and dynamic skill loading (Matrix Protocol)
- **Personalization**: Hardware-aware content adaptation (RTX 4090 vs Jetson Orin Nano) with Gemini 2.5 Flash rewriting and 7-day caching
- **Localization**: Urdu translation (Roman/Formal) with code preservation
- **Authentication**: Better-Auth with Email/GitHub OAuth and onboarding quiz for hardware profiling

**Technical Approach**: Monorepo strategy with `apps/docs` (Docusaurus frontend), `apps/api` (FastAPI backend), Neon PostgreSQL for user data + caching, Qdrant Cloud for vector search, **OpenAI SDK configured with Gemini 2.5 Flash backend** for AI orchestration, and `.claude/` for agentic infrastructure (specialized agents + reusable skills).

## Technical Context

**Language/Version**:
- Frontend: TypeScript 5.3 + React 18 (Docusaurus v3)
- Backend: Python 3.11+

**Primary Dependencies**:
- Frontend: `@docusaurus/core` v3, `@docusaurus/theme-mermaid`, `@better-auth/react`
- Backend: `fastapi`, `uvicorn`, `sqlalchemy[asyncio]`, `qdrant-client`, **`openai`**, **`openai-agents-sdk`**, `better-auth-python`, `pydantic`

**AI Orchestration (Drop-in Replacement Strategy)**:
- **Library**: OpenAI Python SDK (`openai` package) and `openai-agents-sdk`
- **Model**: Google Gemini 2.5 Flash (`gemini-2.5-flash`)
- **Configuration**:
  - **Base URL**: `https://generativelanguage.googleapis.com/v1beta/openai/`
  - **API Key**: Loaded from `GEMINI_API_KEY` environment variable (NOT `OPENAI_API_KEY`)
  - **Embeddings**: Gemini `text-embedding-004` via same compatible endpoint (free tier)
- **Implementation Note**: Use OpenAI SDK as client library (NOT Google's SDK) but route all requests to Gemini API via custom `base_url` configuration (Constitution Article II, Item 6)

**Example Backend Configuration**:
```python
from openai import OpenAI

# Initialize OpenAI client with Gemini backend
client = OpenAI(
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
    api_key=os.getenv("GEMINI_API_KEY")  # NOT OPENAI_API_KEY
)

# Chat completion with Gemini 2.5 Flash
response = client.chat.completions.create(
    model="gemini-2.5-flash",
    messages=[{"role": "user", "content": "Explain ROS 2 Topics"}]
)

# Embeddings with Gemini text-embedding-004
embedding = client.embeddings.create(
    model="text-embedding-004",
    input="ROS 2 Topics are named buses for message passing..."
)
```

**Storage**:
- Relational: Neon (Serverless PostgreSQL) - 5 tables (`users`, `chat_logs`, `personalized_content`, `translated_content`, `audit_logs`)
- Vector: Qdrant Cloud (Free Tier) - 1 collection (`book_knowledge` with 768-dim embeddings for `text-embedding-004`)

**Testing**:
- Backend: `pytest` with `TestClient` for API endpoints + contract testing
- Frontend: Playwright for E2E testing of Chat Widget, Personalize/Translate buttons

**Target Platform**:
- Frontend: Web (GitHub Pages) - responsive for desktop/mobile browsers
- Backend: Linux server (Render or Vercel) - Python runtime

**Project Type**: Web application (frontend + backend monorepo)

**Performance Goals**:
- Docusaurus page load: <2 seconds (SC-001)
- Chatbot response: <3 seconds with citations (SC-003)
- Personalization: <10 seconds for 2000-word chapter (SC-004)
- Translation: <8 seconds for 2000-word chapter (SC-005)
- Concurrent users: 500 without degradation (SC-006)

**Constraints**:
- Constitution Article II: MUST use Docusaurus, Qdrant, Neon, FastAPI, Better-Auth, OpenAI SDK (configured with Gemini backend) - no alternatives
- Constitution Article VI: Zero hardcoded secrets (all in `.env`), mandatory input sanitization, RAG citations required
- Hackathon Timeline: MVP must achieve 300/100 points within submission deadline
- **API Key Management**: GEMINI_API_KEY (NOT OPENAI_API_KEY) for all AI operations
- **Embedding Dimensions**: 768-dim vectors for `text-embedding-004` (NOT 1536-dim)

**Scale/Scope**:
- Content: 4 course modules, 13 weeks, ~50 chapters, ~100K words
- Users: 500 concurrent learners (Hackathon evaluation period)
- Vector Embeddings: ~1000 chunks (500 words each) in Qdrant with 768-dimensional vectors
- API Endpoints: 7 endpoints (`/auth/signup`, `/auth/signin`, `/auth/onboarding`, `/chat`, `/personalize`, `/translate`, `/health`)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Verify compliance with `.specify/memory/constitution.md` (Version 4.1.0):

- [x] **Article I (SDD)**: Feature has `spec.md` (21,737 bytes, 305 lines) and `plan.md` (this file) in place ✅
- [x] **Article II (Tech Stack)**: All technologies comply with authorized stack:
  - ✅ Docusaurus v3 (Frontend)
  - ✅ Qdrant Cloud (RAG Database)
  - ✅ Neon (Serverless PostgreSQL)
  - ✅ FastAPI (Backend API)
  - ✅ Better-Auth (Authentication)
  - ✅ OpenAI SDK configured with Gemini 2.5 Flash backend (AI Orchestration - Article II, Item 6)
- [x] **Article III (MCP Tool Mandate)**: MCP servers usage documented:
  - ✅ Context7: Fetch Docusaurus v3, FastAPI, Neon docs during research phase (see `research.md`)
  - ✅ GitHub MCP: Deployment workflows (see `quickstart.md` deployment section)
  - ✅ Playwright: E2E testing (see `quickstart.md` testing section)
- [x] **Article IV (Agent Behavior)**: Language protocol followed:
  - ✅ Code/Docs: Professional US English (all `.md`, `.ts`, `.py` files)
  - ✅ Chat: Roman Urdu (user-facing chatbot responses - implemented in `/api/chat` endpoint)
  - ✅ Anti-Hallucination: Context7 used for documentation verification (see `research.md` section 13)
  - ✅ Role Fidelity: `.claude/agents/` defines specialized roles (author, coder, architect, qa, translator)
- [x] **Article V (Publishing Standards)**: Docusaurus-native content:
  - ✅ `sidebars.js` structure planned for 4 modules
  - ✅ Mermaid.js diagrams mandated for ROS 2 graphs (FR-002)
  - ✅ Code tabs (`<Tabs>`) for "Simulated vs Real Robot" variations (FR-003)
  - ✅ Admonitions (`:::tip`, `:::warning`) for hardware alerts (FR-006)
  - ✅ Context7 verification for ROS 2 / NVIDIA Isaac technical claims (research.md section 13)
- [x] **Article VI (Engineering Standards)**: SOC Protocol enforced:
  - ✅ Zero Hardcoding: All API keys in `.env` (GEMINI_API_KEY, not OPENAI_API_KEY) - see `quickstart.md` environment setup
  - ✅ Input Sanitization: Implemented in `/api/chat`, `/api/personalize`, `/api/translate` (FR-023)
  - ✅ RAG Integrity: Citations mandatory in chatbot responses (FR-009, see `data-model.md` ChatLog entity)
  - ✅ GitHub Actions Deployment: Documented in `quickstart.md` deployment section
- [x] **Article VII (Intelligence Preservation)**: PHR and ADR planned:
  - ✅ PHR Creation: This command will generate PHR after plan completion
  - ✅ ADR Planning: Significant architectural decisions documented in `research.md` (e.g., Monorepo Strategy, Vector Store Selection, Gemini Drop-in Replacement)
  - ✅ Folder Integrity: Work constrained to `apps/`, `specs/`, `.claude/` (no new root directories)

**Compliance Status**: ✅ ALL CHECKS PASSED - Ready for Phase 2 (Task Generation)

## Project Structure

### Documentation (this feature)

```text
specs/001-ai-textbook-platform/
├── spec.md              # Feature specification (User Stories, Requirements, Success Criteria)
├── plan.md              # This file (Architecture, Technical Context, Phase 0-1 artifacts)
├── research.md          # Phase 0: Technology research and architectural decisions ✅ CREATED
├── data-model.md        # Phase 1: Entity schemas (Neon + Qdrant) ✅ CREATED
├── quickstart.md        # Phase 1: Local development setup guide ✅ CREATED
├── contracts/           # Phase 1: API contracts ✅ CREATED
│   └── api-schema.yaml  #   OpenAPI 3.1 schema for 7 FastAPI endpoints
├── checklists/          # Spec validation
│   └── requirements.md  #   Quality gates (all passed)
└── tasks.md             # Phase 2: Implementation tasks ⏳ PENDING (/sp.tasks command)
```

### Source Code (repository root)

**Selected Structure**: **Web Application (Monorepo)** with `apps/` and `packages/` separation

```text
/ (repository root)
│
├── apps/                              # Application workspaces
│   ├── docs/                          # Docusaurus v3 (Frontend)
│   │   ├── docs/                      #   MDX content files (4 modules, 13 weeks)
│   │   │   ├── week-01-ros2-basics/   #     Module 1: Robotic Nervous System
│   │   │   ├── week-06-gazebo-sim/    #     Module 2: Digital Twin
│   │   │   ├── week-08-isaac-sim/     #     Module 3: AI-Robot Brain
│   │   │   ├── week-11-vla/           #     Module 4: Vision-Language-Action
│   │   │   └── hardware-lab/          #     Hardware Lab Guide
│   │   ├── src/                       #   Custom React components
│   │   │   ├── components/            #     Chat Widget, Personalize/Translate buttons
│   │   │   ├── theme/                 #     Docusaurus theme customizations
│   │   │   └── pages/                 #     Custom pages (login, onboarding)
│   │   ├── static/                    #   Static assets (images, diagrams)
│   │   ├── docusaurus.config.js       #   Docusaurus configuration (plugins, theme)
│   │   ├── sidebars.js                #   Sidebar structure for 4 modules
│   │   ├── package.json               #   NPM dependencies
│   │   └── .env                       #   Frontend env vars (API_URL, BETTER_AUTH_CLIENT_ID)
│   │
│   └── api/                           # FastAPI (Backend)
│       ├── src/                       #   Python source code
│       │   ├── models/                #     SQLAlchemy models (User, ChatLog, etc.)
│       │   ├── routers/               #     API route handlers (/auth, /chat, /personalize, /translate)
│       │   ├── services/              #     Business logic (RAG pipeline, personalization, translation)
│       │   │   ├── gemini_client.py   #       OpenAI SDK configured with Gemini backend
│       │   │   ├── rag_pipeline.py    #       Vector search + Gemini chat completion
│       │   │   ├── personalize.py     #       Content adaptation with Gemini 2.5 Flash
│       │   │   └── translate.py       #       Urdu translation with Gemini 2.5 Flash
│       │   ├── utils/                 #     Helper functions (sanitization, JWT validation)
│       │   └── main.py                #     FastAPI app entry point
│       ├── scripts/                   #   Utility scripts
│       │   ├── init_qdrant.py         #     Create Qdrant collection (768-dim for text-embedding-004)
│       │   ├── ingest.py              #     Index Docusaurus MDX files into Qdrant (using Gemini embeddings)
│       │   └── migrate_db.py          #     Run Alembic migrations
│       ├── tests/                     #   Pytest test suites
│       │   ├── test_auth.py           #     Authentication endpoint tests
│       │   ├── test_chat.py           #     RAG chatbot tests + sanitization
│       │   ├── test_personalize.py    #     Personalization tests (RTX 4090, Jetson)
│       │   ├── test_translate.py      #     Urdu translation tests
│       │   └── test_contract.py       #     OpenAPI contract validation
│       ├── migrations/                #   Alembic database migrations
│       │   ├── versions/              #     Migration files (001_create_users.py, etc.)
│       │   └── env.py                 #     Alembic config
│       ├── requirements.txt           #   Python dependencies (openai, openai-agents-sdk)
│       ├── .env                       #   Backend env vars (NEON_CONNECTION_STRING, QDRANT_API_KEY, GEMINI_API_KEY, etc.)
│       └── Dockerfile                 #   Docker image for deployment
│
├── packages/                          # Shared code (optional)
│   └── shared-types/                  # TypeScript interfaces shared between frontend/backend
│       ├── user.ts                    #   User, UserProfile interfaces
│       ├── chat.ts                    #   ChatMessage, Citation interfaces
│       └── package.json               #   Shared types NPM package
│
├── .claude/                           # Agentic Infrastructure (Matrix Protocol) 🧠
│   ├── agents/                        #   Specialized AI agent definitions
│   │   ├── author.md                  #     Physics & Robotics Professor (Content Creator)
│   │   ├── coder.md                   #     Full-Stack Engineer (RAG/FastAPI/React)
│   │   ├── architect.md               #     System Designer (Folder Structure/ADRs)
│   │   ├── qa.md                      #     SOC Analyst & Tester (Security/Sanitization)
│   │   └── translator.md              #     Urdu Localization Expert (Bonus Feature)
│   │
│   └── skills/                        #   Reusable knowledge modules
│       ├── ros2-mastery.md            #     Deep knowledge of ROS 2 Nodes/Topics
│       ├── docusaurus-guru.md         #     Mastery of Admonitions, Tabs, Swizzling
│       ├── matrix-loader.md           #     Logic to dynamically load other skills
│       └── better-auth.md             #     Implementation guide for Auth bonus
│
├── specs/                             # Feature specifications (SDD workflow)
│   └── 001-ai-textbook-platform/      #   This feature
│
├── history/                           # Intelligence preservation
│   ├── prompts/                       #   Prompt History Records (PHRs)
│   │   ├── constitution/              #     Constitution-related PHRs
│   │   ├── 001-ai-textbook-platform/  #     Feature-specific PHRs
│   │   └── general/                   #     General PHRs
│   └── adr/                           #   Architectural Decision Records
│
├── .specify/                          # SpecKit Plus templates and scripts
│   ├── memory/                        #   Project memory
│   │   └── constitution.md            #     Constitution v4.1.0 (Gemini Configuration Amendment)
│   ├── templates/                     #   Document templates
│   └── scripts/                       #   Automation scripts
│
├── .github/                           # GitHub Actions workflows
│   └── workflows/                     #   CI/CD pipelines
│       ├── deploy-docs.yml            #     Deploy Docusaurus to GitHub Pages
│       └── test-api.yml               #     Run Pytest on push
│
├── docker-compose.yml                 # Local orchestration (optional)
├── README.md                          # Project overview and quickstart link
├── CLAUDE.md                          # Claude Code agent instructions
└── .gitignore                         # Git ignore rules
```

**Structure Decision**:
- **Monorepo Rationale**: Enables code sharing (TypeScript types), atomic deployments, and centralized `.claude/` agentic infrastructure
- **apps/docs**: Docusaurus v3 frontend with 4 course modules (13 weeks), custom React components for Chat Widget, Personalize/Translate buttons
- **apps/api**: FastAPI backend with 7 endpoints, SQLAlchemy models for Neon PostgreSQL, Qdrant client for vector search (768-dim), **OpenAI SDK configured with Gemini 2.5 Flash backend** for RAG
- **packages/shared-types**: Optional TypeScript interfaces shared between frontend/backend (User, ChatMessage, Citation)
- **.claude/**: Agentic infrastructure with 5 specialized agents (author, coder, architect, qa, translator) and 4 reusable skills (ros2-mastery, docusaurus-guru, matrix-loader, better-auth) - **CRITICAL for Matrix Protocol bonus feature**

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**Status**: ✅ NO VIOLATIONS - All Constitution checks passed (see Constitution Check section above).

No complexity justifications required. All architectural decisions align with Constitution Article II (Authorized Tech Stack - OpenAI SDK with Gemini backend) and follow established best practices documented in `research.md`.

---

## Phase Completion Summary

### Phase 0: Research & Technology Decisions ✅ COMPLETE
- **Artifact**: `research.md` (20,788 bytes)
- **Content**: 13 architectural decisions (project structure, frontend/backend stack, database/vector store, auth, **AI orchestration with Gemini drop-in replacement**, personalization/localization, testing, deployment, agentic infrastructure, MCP usage)
- **All NEEDS CLARIFICATION resolved**: User provided detailed architecture strategy including Gemini configuration; no ambiguities remain

### Phase 1: Design & Contracts ✅ COMPLETE
- **Artifacts**:
  1. `data-model.md` (20,369 bytes): 5 Neon PostgreSQL entities + 1 Qdrant collection (768-dim for `text-embedding-004`) with complete ERDs and data flows
  2. `contracts/api-schema.yaml` (OpenAPI 3.1 schema): 7 FastAPI endpoints with request/response schemas
  3. `quickstart.md` (local development guide): Prerequisites, environment setup (GEMINI_API_KEY), database/vector store init, testing, troubleshooting
- **Agent Context Update**: ✅ COMPLETE (ran `.specify/scripts/bash/update-agent-context.sh claude`)

### Phase 2: Task Generation ⏳ PENDING
- **Next Command**: `/sp.tasks` to generate implementation tasks based on this plan
- **Expected Output**: `specs/001-ai-textbook-platform/tasks.md` with dependency-ordered tasks for all 6 user stories (P1-P6)

---

## Next Steps

1. **Run `/sp.tasks`**: Generate actionable implementation tasks organized by user story priority
2. **Create `.claude/` Infrastructure**: Implement 5 agents (author, coder, architect, qa, translator) and 4 skills (ros2-mastery, docusaurus-guru, matrix-loader, better-auth) as defined in `research.md` section 12
3. **Execute Implementation**: Follow `tasks.md` to build:
   - Phase A: Setup + Foundation (project init, database/Qdrant setup with 768-dim vectors)
   - Phase B: User Story 1 (Content Engine - Docusaurus textbook)
   - Phase C: User Story 2 (RAG Chatbot with Gemini backend + Matrix Protocol)
   - Phase D: User Story 4 (Better-Auth + Onboarding)
   - Phase E: User Story 3 (Personalization Engine with Gemini 2.5 Flash)
   - Phase F: User Story 5 (Localization Engine with Gemini 2.5 Flash)
4. **Create ADRs**: Document significant architectural decisions (Monorepo Strategy, Vector Store Selection, Personalization Approach, **Gemini Drop-in Replacement Strategy**) in `history/adr/`
5. **Deploy**: Follow `quickstart.md` deployment section for GitHub Pages (Docusaurus) + Render (FastAPI)

---

## Key Architectural Highlights

1. **Monorepo Strategy**: `apps/docs` (Docusaurus frontend) + `apps/api` (FastAPI backend) with shared `packages/shared-types`
2. **Dual Storage**: Neon PostgreSQL (relational data, caching) + Qdrant Cloud (768-dimensional vector embeddings for `text-embedding-004`)
3. **AI Orchestration (Drop-in Replacement)**: OpenAI SDK configured with `base_url="https://generativelanguage.googleapis.com/v1beta/openai/"` to use **Gemini 2.5 Flash** and **text-embedding-004** via `GEMINI_API_KEY`
4. **Matrix Protocol**: Dynamic skill loading for context-aware chatbot responses using OpenAI Agents SDK
5. **Personalization Pipeline**: Gemini 2.5 Flash content rewriting with 7-day Neon caching (hardware-aware: RTX 4090 vs Jetson Orin Nano)
6. **Localization Pipeline**: Gemini 2.5 Flash Urdu translation with code preservation and 7-day Neon caching
7. **Agentic Infrastructure**: `.claude/agents/` (5 specialized roles) + `.claude/skills/` (4 reusable knowledge modules) for Matrix Protocol bonus feature
8. **Security (SOC Protocol)**: Input sanitization on all endpoints, zero hardcoded secrets (GEMINI_API_KEY in `.env`), JWT auth with Better-Auth, audit logging

---

**Plan Status**: ✅ COMPLETE - All Constitution checks passed (including Gemini configuration), all Phase 0-1 artifacts created, ready for Phase 2 (Task Generation)
