---
id: "0023"
title: "Hackathon Execution Plan: Bonus-First Strategy"
stage: plan
date: 2025-12-20T00:00:00Z
surface: claude-code
model: claude-sonnet-4-5-20250929
feature: 001-ai-textbook-platform
branch: 001-ai-textbook-platform
user: user
command: /sp.plan
labels: [hackathon, bonus-focused, constitution-aligned, mcp-first]
links:
  spec: specs/001-ai-textbook-platform/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - path: specs/001-ai-textbook-platform/plan.md
    status: created
tests:
  - name: Constitution Check
    path: specs/001-ai-textbook-platform/plan.md
    status: passed
---

## Prompt

User requested comprehensive implementation plan for hackathon project with bonus-first strategy, emphasizing:
- Constitution-aligned development (AGENTS.md enforcement)
- 4-phase execution: Book Content Creation (agents/skills) → Core Platform (RAG chatbot) → Authentication → Personalization → Translation
- MCP-first documentation reference for all external libraries
- Risk mitigation for free-tier limitations and time constraints
- Clear completion criteria targeting 300 total points (100 base + 200 bonus)

### Additional Context from User Prompt:

**Hackathon Requirements**:
- Base (100 pts): Docusaurus book + RAG chatbot (OpenAI Agents/ChatKit, FastAPI, Neon, Qdrant)
- Bonus (+50 pts each):
  1. Reusable AI intelligence (Claude Code agents + skills)
  2. Authentication with Better Auth (capture software/hardware background)
  3. Per-chapter personalization (button-triggered, user background-based)
  4. Per-chapter Urdu translation (button-triggered, technical term preservation)
- Submission deadline: Nov 30, 2025 at 6:00 PM (already passed - project for reference/iteration)
- Demo video: Must be under 90 seconds

**Course Content Structure**:
- Module 1 (Weeks 1-5): ROS 2 Fundamentals (robotic nervous system)
- Module 2 (Weeks 6-7): Digital Twin (Gazebo & Unity simulation)
- Module 3 (Weeks 8-10): NVIDIA Isaac Platform (AI-robot brain)
- Module 4 (Weeks 11-13): Vision-Language-Action (VLA, conversational robotics)

**Constitutional Constraints**:
- Immutable tech stack (Constitution Principle 2): Docusaurus, FastAPI, BetterAuth, Neon, Qdrant, OpenAI Agents/ChatKit
- MCP-first documentation (Principle 3): Must query context7 MCPs before implementation
- RAG scope strictly book-only (Principle 4): No general-purpose AI, citations required
- Auth gates interactive features (Principle 5): Public book reading, authenticated chatbot/personalization/translation
- Quality gates non-negotiable (Principle 8): Build success, type safety, auth/authz, RAG functionality, deployment

**User's Requested Structure** (from prompt fragments):
- Phase 0: Agent-driven book content creation (bonus points)
- Phase 1: Core platform (Docusaurus + RAG chatbot)
- Phase 2: Authentication (Better Auth + background capture)
- Phase 3: Personalization (chapter-scoped, on-demand)
- Phase 4: Urdu translation (chapter-scoped, technical term preservation)
- Cross-cutting concerns: Testing, security, deployment
- Agent & tooling strategy: MCP usage, Claude Code agents/skills, Spec-Kit Plus workflow
- Risk register: Time overrun, LLM API limits, over-engineering, deployment failures

## Response Snapshot

Created comprehensive 957-line implementation plan with constitution-aligned 4-phase structure:

### Plan Structure:

**Summary**: Production-grade AI-native textbook platform delivering 100 base points + 200 bonus points through strict phase gates and MCP-first development.

**Technical Context**:
- Frontend: Docusaurus 3.6+, React 18.x, TypeScript (strict mode), Vercel/GitHub Pages
- Backend: FastAPI (Python 3.11+), OpenAI Agents/ChatKit, Neon Postgres, Qdrant Cloud
- Auth: BetterAuth with software/hardware background capture
- Testing: Playwright (E2E), TypeScript strict mode, Python mypy/pyright
- Performance: Chatbot < 5s (95%), personalization/translation < 3s, page load < 2s
- Scale: 13 weeks × 4 modules = ~26-40 chapters, 4 bonus categories, 300 total points target

**Constitution Check**: ✅ Full compliance (8/8 principles passed, zero violations)

**Project Structure**: Monorepo with `docs/` (Docusaurus), `backend/` (FastAPI), `.claude/` (agents/skills), `history/` (PHRs), `tests/` (Playwright E2E)

### Implementation Phases:

**Phase 0: Book Content Creation (AGENT-DRIVEN, +50 BONUS)**:
- Create 5 custom Claude Code agents: Curriculum Architect, Chapter Author, Robotics Expert, Pedagogy Simplifier, Review & Accuracy
- Create 4 reusable skills: Chapter Scaffolding, Concept Explanation, Diagram Generation, Review/Simplification
- Write 13 weeks of Physical AI content across 4 modules (26-40 chapters total)
- Agent workflow: Architect → Author → Expert → Simplifier → Review
- Estimated effort: 40-60 hours
- Bonus points: +50 (FR-034 to FR-038, SC-020 to SC-023)

**Phase 1: Core Platform (BASE 100 POINTS)**:
- 1.1: Docusaurus setup + custom components (ChatWidget, PersonalizeButton, TranslateButton)
- 1.2: Qdrant vector database (500-1000 token chunks, OpenAI embeddings)
- 1.3: Neon Postgres schema (users, chat_sessions, chat_messages)
- 1.4: FastAPI RAG chatbot (query endpoint, text selection, off-topic detection < 0.6 similarity)
- 1.5: Chat widget integration (floating panel, citation links, text selection)
- Estimated effort: 30-50 hours
- Points: 100 base (FR-001 to FR-015)

**Phase 2: Authentication (+50 BONUS)**:
- 2.1: BetterAuth integration (email/password, 30-day session, user schema extension)
- 2.2: Signup flow (email, password, software background, hardware background)
- 2.3: Signin flow (email, password)
- 2.4: Auth-gated features (chatbot, personalize button, translate button)
- Estimated effort: 15-25 hours
- Bonus points: +50 (FR-016 to FR-021, SC-011, SC-015)

**Phase 3: Personalization (+50 BONUS)**:
- 3.1: Personalization API (LLM-based content adaptation using user background)
- 3.2: PersonalizeButton component (3-state: Personalize → Loading → Show Original)
- Personalization logic: Adjust complexity, add relevant examples, inject callouts, preserve code/structure
- Estimated effort: 20-30 hours
- Bonus points: +50 (FR-022 to FR-027, SC-012 to SC-014)

**Phase 4: Urdu Translation (+50 BONUS)**:
- 4.1: Translation API (LLM-based Urdu translation with technical term preservation)
- 4.2: TranslateButton component (3-state: Translate → Loading → Show Original English)
- 4.3: RTL styling (right-to-left Urdu, left-to-right code blocks)
- Translation rules: Preserve MDX structure, keep code in English, transliterate technical terms
- Estimated effort: 20-30 hours
- Bonus points: +50 (FR-028 to FR-033, SC-016 to SC-019)

### Cross-Cutting Concerns:

**Testing Strategy**:
- Phase-level validation with explicit Validation and Failure Conditions sections
- Playwright E2E tests for all user journeys (navigation, chatbot, auth, personalization, translation)
- TypeScript strict mode + Python mypy for type safety
- Manual QA for chatbot accuracy and translation quality

**Security**:
- BetterAuth handles password hashing, session management (no custom crypto)
- Session tokens in httpOnly cookies (prevent XSS)
- CORS configured for Vercel deployment
- Environment variables for all secrets (OpenAI, Neon, Qdrant, BetterAuth)

**Deployment**:
- Vercel (preferred): Static Docusaurus + serverless FastAPI functions
- GitHub Pages (fallback): Static Docusaurus + separate FastAPI hosting (Railway/Render/Fly.io)
- Validation: Live URL, HTTPS, API reachability, environment variables, no hardcoded secrets

**MCP Server Usage**: Mandatory queries before implementation for Docusaurus, FastAPI, BetterAuth, Qdrant, Neon, Vercel, Playwright

**Risk Register**:
- HIGH: Time overrun (prioritize base over bonus), LLM API limits (caching), Qdrant/Neon limits (optimization), BetterAuth complexity (MCP queries)
- MEDIUM: Over-engineering (enforce SDD), bonus feature instability (independent modules), chatbot hallucinations (similarity threshold)
- LOW: Deployment failures (staging tests), translation quality (GPT-4 + bilingual review), demo video length (script + rehearse)

### Completion Criteria (300 points):

**Base (100 points)**: Book live, 13 weeks published, code highlighting, responsive, navigation, RAG chatbot functional, citations, vector indexing

**Bonus (200 points)**:
- Authentication (+50): Signup/signin < 3min, background capture, session persistence
- Personalization (+50): Button at chapter start, < 3s adaptation, measurable differences, reversible
- Translation (+50): Button at chapter start, < 3s translation, technical term preservation, RTL rendering, reversible
- Reusable Intelligence (+50): 5+ agents, 4+ skills, Constitution enforcement, 50% faster authoring

**Quality Gates**: Build success, type safety, auth gates, RAG functionality, deployment integrity, documentation complete

## Outcome

- ✅ Impact: Comprehensive 957-line implementation plan covering all hackathon requirements (300 points) with constitution-aligned 4-phase structure, MCP-first strategy, and detailed risk mitigation
- 🧪 Tests: Constitution Check passed (8/8 principles), phase-level validation defined for all 4 phases
- 📁 Files: Created plan.md with complete technical context, project structure, implementation phases, cross-cutting concerns, MCP usage table, risk register, completion criteria
- 🔁 Next prompts: `/sp.tasks` to generate dependency-ordered tasks.md, then implement Phase 0 (agents/skills) → Phase 1 (core platform) → Phase 2-4 (bonus features)
- 🧠 Reflection: Plan successfully balances comprehensive detail with actionable structure. Bonus-first strategy (Phase 0 agents) earns points while creating content. All phases independently testable with explicit validation criteria. MCP-first approach prevents hallucinations. Risk mitigation addresses all identified concerns from user prompt.

## Evaluation Notes (Flywheel)

- **Failure modes observed**: None - planning process completed successfully with full Constitution alignment
- **Graders run and results (PASS/FAIL)**: Constitution Check - PASS (8/8 principles)
- **Prompt variant (if applicable)**: Bonus-First Execution Plan variant - emphasizes agent-driven content creation as Phase 0 to earn +50 points while producing core book content
- **Next experiment (smallest change to try)**: Execute `/sp.tasks` to test whether task breakdown can maintain phase independence (auth/personalization/translation should be fully parallel after core platform)
