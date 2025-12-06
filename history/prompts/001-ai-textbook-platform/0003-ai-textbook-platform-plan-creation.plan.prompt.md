---
id: 0003
title: AI Textbook Platform Plan Creation
stage: plan
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-ai-textbook-platform
branch: 001-ai-textbook-platform
user: system
command: /sp.plan
labels: ["architecture", "monorepo", "docusaurus", "fastapi", "rag", "personalization", "localization", "matrix-protocol"]
links:
  spec: specs/001-ai-textbook-platform/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-ai-textbook-platform/plan.md (created, 282 lines)
  - specs/001-ai-textbook-platform/research.md (created, 20,788 bytes, 13 architectural decisions)
  - specs/001-ai-textbook-platform/data-model.md (created, 20,369 bytes, 5 Neon entities + 1 Qdrant collection)
  - specs/001-ai-textbook-platform/quickstart.md (created, local development guide)
  - specs/001-ai-textbook-platform/contracts/api-schema.yaml (created, OpenAPI 3.1 schema for 7 endpoints)
  - CLAUDE.md (updated, agent context registered)
tests:
  - Constitution Check: ALL 7 ARTICLES PASSED ✅
---

## Prompt

**Implementation Context & Architecture Strategy**

**1. Project Structure (Monorepo Strategy):**
   - Root `/`:
     - `apps/docs/`: The Docusaurus v3 project (Frontend).
     - `apps/api/`: The FastAPI Python project (Backend/RAG).
     - `packages/`: Shared logic (if any).
     - `docker-compose.yml`: For local orchestrating (optional).

**2. Technology Stack (Strict Adherence to Constitution):**
   - **Frontend:** Docusaurus v3 (React/TypeScript).
     - *Styling:* CSS Modules / Custom CSS (Matrix Theme).
     - *Auth:* **Better-Auth** client integration.
   - **Backend:** Python 3.11+, FastAPI.
     - *Server:* Uvicorn.
     - *Database:* **Neon** (Serverless Postgres) via `sqlalchemy` (Async).
     - *Vector Store:* **Qdrant Cloud** via `qdrant-client`.
     - *AI:* OpenAI Agents SDK + ChatKit.

**3. Data Architecture (Phase 1 Design):**
   - **Postgres (Neon):**
     - Table `users`: Stores ID, email, hardware_profile (Sim Rig vs Edge Kit).
     - Table `chat_logs`: Stores query, response, citation, user_id.
   - **Qdrant:**
     - Collection `book_knowledge`: Stores chunked MDX content with metadata (chapter_id, section_id).

**4. Integration Points (The Matrix Protocol):**
   - **Ingestion Script (`apps/api/scripts/ingest.py`):** Must parse Docusaurus Frontmatter to preserve hierarchy.
   - **Chat Widget:** A React component in Docusaurus that calls `POST /api/chat`.
   - **Personalization:** React Context in Docusaurus that reads `user.hardware_profile` and conditionally renders content blocks.

**5. Testing Strategy:**
   - **Backend:** `pytest` for API endpoints.
   - **Frontend:** Playwright for E2E testing of the Chat Widget.

**6. Deployment Targets:**
   - **Docs:** GitHub Pages (via GitHub Actions).
   - **API:** Render or Vercel (Python Runtime).

**7. Agentic Infrastructure (The Matrix Protocol Strategy):**
   *Reference: Panaversity AI-Native Repo Structure*

   We need to architect the "Brain" of the project alongside the code. The Plan must include the creation of the following **Smart Contexts** inside `.claude/`:

   **A. Specialized Agents (`.claude/agents/`):**
   - `author.md`: The Physics & Robotics Professor (Content Creator).
   - `coder.md`: The Full-Stack Engineer (RAG/FastAPI/React).
   - `architect.md`: The System Designer (Folder Structure/ADRs).
   - `qa.md`: The SOC Analyst & Tester (Security/Sanitization).
   - `translator.md`: The Urdu Localization Expert (Bonus Feature).

   **B. Reusable Skills (`.claude/skills/`):**
   - `ros2-mastery.md`: Deep knowledge of Nodes/Topics for accuracy.
   - `docusaurus-guru.md`: Mastery of Admonitions, Tabs, and Swizzling.
   - `matrix-loader.md`: (Bonus) The logic to dynamic load other skills.
   - `better-auth.md`: Implementation guide for the Auth bonus.

   **Directive for Plan:**
   Please explicitly list these files in the **"Project Structure"** section of the `plan.md`. Treat these Agent Files as critical source code that needs to be implemented.

## Response snapshot

Implementation plan successfully created for "Physical AI & Humanoid Robotics Textbook Platform" targeting 300/100 Hackathon score.

**Planning Phases Completed**:

**Phase 0: Research & Technology Decisions ✅**
- Created `research.md` (20,788 bytes) with 13 architectural decisions:
  1. Monorepo Strategy (apps/ + packages/ + .claude/)
  2. Frontend: Docusaurus v3 (React/TypeScript) with Mermaid, Tabs, Better-Auth
  3. Backend: FastAPI (Python 3.11+) with Uvicorn, async SQLAlchemy
  4. Database: Neon (Serverless PostgreSQL) with 5 tables (users, chat_logs, personalized_content, translated_content, audit_logs)
  5. Vector Store: Qdrant Cloud with 1536-dim embeddings (OpenAI ada-002)
  6. Authentication: Better-Auth with Email/GitHub OAuth + onboarding quiz
  7. AI Orchestration: OpenAI Agents SDK + ChatKit for RAG
  8. Personalization: GPT-4 content rewriting with 7-day Neon caching
  9. Localization: GPT-4 Urdu translation with code preservation
  10. Testing: Pytest (backend) + Playwright (frontend E2E)
  11. Deployment: GitHub Pages (docs) + Render (API)
  12. Agentic Infrastructure: 5 specialized agents + 4 reusable skills in `.claude/`
  13. MCP Tool Usage: Context7 (docs), GitHub MCP (deployment), Playwright (verification)

**Phase 1: Design & Contracts ✅**
- Created `data-model.md` (20,369 bytes):
  - 5 Neon PostgreSQL entities with ERDs (User, ChatLog, PersonalizedContent, TranslatedContent, AuditLog)
  - 1 Qdrant collection (book_knowledge with payload schema)
  - 3 data flow diagrams (onboarding, RAG query, personalization)
- Created `contracts/api-schema.yaml` (OpenAPI 3.1 schema):
  - 7 FastAPI endpoints: /auth/signup, /auth/signin, /auth/onboarding, /chat, /personalize, /translate, /health
  - Request/response schemas with validation rules
  - Security schemes (JWT Bearer auth)
- Created `quickstart.md` (local development guide):
  - Prerequisites (Node.js 18+, Python 3.11+)
  - Environment setup (.env configuration for Neon, Qdrant, OpenAI, Better-Auth)
  - Database migrations (Alembic)
  - Qdrant collection initialization
  - Frontend/backend parallel execution
  - Testing commands (pytest, playwright)
  - Troubleshooting guide
- Updated agent context: Ran `.specify/scripts/bash/update-agent-context.sh claude`

**Constitution Compliance Validation**:
- ✅ Article I (SDD): spec.md and plan.md in place
- ✅ Article II (Tech Stack): All technologies match authorized stack (Docusaurus, Qdrant, Neon, FastAPI, Better-Auth, OpenAI Agents SDK)
- ✅ Article III (MCP Mandate): Context7, GitHub MCP, Playwright usage documented
- ✅ Article IV (Agent Behavior): Language protocol (Roman Urdu chat, English code/docs), role fidelity (.claude/agents/)
- ✅ Article V (Publishing): Docusaurus conventions (Mermaid, Tabs, Admonitions), Context7 verification
- ✅ Article VI (Engineering): Zero hardcoding, input sanitization, RAG citations, GitHub Actions deployment
- ✅ Article VII (Intelligence): PHR creation, ADR planning, folder integrity

**Key Architectural Decisions**:
1. **Monorepo**: Enables code sharing (TypeScript types), atomic deployments, centralized .claude/ infrastructure
2. **Dual Storage**: Neon (relational + caching) + Qdrant (vector search)
3. **Matrix Protocol**: Dynamic skill loading (ros2-mastery, edge-computing) based on chatbot query keywords
4. **Personalization Pipeline**: GPT-4 rewriting → 7-day Neon cache → hardware-aware content (RTX 4090 vs Jetson)
5. **Localization Pipeline**: GPT-4 translation → 7-day Neon cache → code preservation in English
6. **Agentic Infrastructure**: 5 agents (author, coder, architect, qa, translator) + 4 skills - CRITICAL for Matrix Protocol bonus

**Project Structure Highlights**:
- `apps/docs/`: Docusaurus frontend with 4 modules (13 weeks), Chat Widget, Personalize/Translate buttons
- `apps/api/`: FastAPI backend with 7 endpoints, RAG pipeline, personalization/translation services
- `.claude/`: Agentic infrastructure (5 agents + 4 skills) for Matrix Protocol bonus
- `packages/shared-types/`: Optional TypeScript interfaces (User, ChatMessage, Citation)

## Outcome

- ✅ Impact: Comprehensive implementation plan created for hackathon project targeting 300/100 score (Base + 4 bonuses: Matrix Protocol, Better-Auth, Personalization, Localization); all Constitution checks passed
- 🧪 Tests: Constitution compliance validation completed (7 articles, all passed); no code tests at planning stage
- 📁 Files: 5 artifacts created (plan.md: 282 lines, research.md: 20,788 bytes, data-model.md: 20,369 bytes, quickstart.md, contracts/api-schema.yaml: OpenAPI 3.1 with 7 endpoints)
- 🔁 Next prompts: Run `/sp.tasks` to generate implementation tasks organized by user story priority (P1-P6); Create `.claude/` infrastructure (5 agents + 4 skills); Execute implementation following tasks.md
- 🧠 Reflection: User-provided architecture strategy (monorepo, agentic infrastructure) was comprehensive and aligned perfectly with Constitution; no ambiguities or NEEDS CLARIFICATION markers; plan emphasizes Matrix Protocol (.claude/ infrastructure) as critical for bonus feature

## Evaluation notes (flywheel)

- Failure modes observed: None - user provided detailed architecture strategy eliminating all ambiguities
- Graders run and results (PASS/FAIL): PASS - All Constitution checks passed (7/7 articles), all Phase 0-1 artifacts created, no violations, no complexity justifications needed
- Prompt variant (if applicable): plan-with-agentic-infrastructure-v1 (user explicitly requested .claude/ agent/skill definitions in project structure)
- Next experiment (smallest change to try): Monitor task generation (/sp.tasks) to ensure Matrix Protocol tasks (agent/skill creation) are prioritized appropriately alongside core feature implementation
