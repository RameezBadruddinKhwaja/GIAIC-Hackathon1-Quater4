<!--
═══════════════════════════════════════════════════════════════════════════════
SYNC IMPACT REPORT
═══════════════════════════════════════════════════════════════════════════════
Version Change: NONE → 1.0.0
Rationale: Initial constitution creation for hackathon project

Modified Principles:
  - N/A (initial creation)

Added Sections:
  - Complete constitution structure with 8 core principles
  - Hackathon-specific governance
  - Tech stack constraints
  - Quality gates and acceptance criteria
  - MCP usage mandates

Removed Sections:
  - N/A (initial creation)

Templates Requiring Updates:
  ✅ spec-template.md - Verified alignment with constitution principles
  ✅ plan-template.md - Verified alignment with constitution principles
  ✅ tasks-template.md - Verified alignment with constitution principles
  ✅ AGENTS.md - Constitution references verified

Follow-up TODOs:
  - RATIFICATION_DATE set to 2025-12-20 (constitution creation date)
  - All placeholders filled with concrete values from hackathon brief
  - Version 1.0.0 indicates first complete, production-ready governance document

═══════════════════════════════════════════════════════════════════════════════
-->

# Project Constitution

**Project Name**: Physical AI & Humanoid Robotics Textbook Platform
**Constitution Version**: 1.0.0
**Ratified**: 2025-12-20
**Last Amended**: 2025-12-20
**Governance Model**: Spec-Driven Development (SDD) via Spec-Kit Plus
**Target**: Panaversity Hackathon I Submission

---

## Purpose & Mission

This constitution governs the development of an **AI-Native Textbook Platform** for teaching the course "Physical AI & Humanoid Robotics." The platform combines educational content delivery (Docusaurus), intelligent assistance (RAG chatbot), user personalization, and multilingual support to create an exemplary learning experience.

**Mission Statement**: Build a production-grade, AI-native textbook platform that demonstrates architectural excellence, spec-driven discipline, and delivers all hackathon requirements without technical debt or shortcuts.

**Success Definition**: Judges can evaluate architectural clarity from specifications alone. All implementations trace directly to approved specs. No experimental code or undocumented features exist.

---

## Governance & Amendment Policy

### Amendment Procedure

1. All constitutional changes MUST be proposed via `/sp.constitution` skill
2. Changes require explicit documentation of:
   - Version bump rationale (MAJOR/MINOR/PATCH)
   - Impact on existing specs, plans, and tasks
   - Template synchronization requirements
3. MAJOR version changes (backward incompatible governance changes) require full project audit
4. MINOR version changes (new principles/sections) require template updates
5. PATCH version changes (clarifications, typos) require validation only

### Versioning Policy

- **MAJOR (X.0.0)**: Backward-incompatible governance changes, principle removals/redefinitions
- **MINOR (0.X.0)**: New principles added, material expansion of guidance, new constraints
- **PATCH (0.0.X)**: Clarifications, wording improvements, typo fixes, non-semantic refinements

### Compliance Review

- Constitution MUST be reviewed before each major development phase (spec → plan → tasks)
- Any spec that violates constitution principles MUST be rejected
- All template files MUST reference constitution principles explicitly
- Agents MUST validate against constitution before executing implementation tasks

---

## Core Principles

### Principle 1: Spec-Driven Development is Absolute

**Rule**: No code may be written without a corresponding approved specification artifact.

**Requirements**:
- MUST follow strict hierarchy: Constitution → Spec → Plan → Tasks → Implementation
- MUST NOT skip phases or create "temporary" code outside specs
- MUST NOT "patch" bugs via prompt engineering; all fixes update specs first
- MUST NOT invent features or behavior not documented in specifications
- MUST NOT use "I'll try this approach" without spec approval

**Rationale**: Spec-Driven Development ensures architectural clarity, prevents scope creep, enables independent evaluation by judges, and creates maintainable systems. Hackathon judges evaluate planning quality, not just working code.

**Validation**:
- Every file MUST trace to a task in `tasks.md`
- Every task MUST trace to a requirement in `spec.md`
- Every requirement MUST trace to a user story with acceptance criteria
- Git commits MUST reference task IDs

---

### Principle 2: Technology Stack is Immutable

**Rule**: The technology stack is frozen and MUST NOT be substituted or extended without constitutional amendment.

**Mandated Stack**:

**Frontend / Book**:
- Docusaurus 3.x (primary documentation framework)
- React 18.x (for interactive components within Docusaurus)
- TypeScript (strictly typed, no implicit any)
- Deployment: Vercel OR GitHub Pages (judges must access live URL)

**Backend (Interactive Features)**:
- FastAPI (Python 3.11+, async/await patterns required)
- OpenAI-compatible APIs (OpenAI SDK or Gemini with OpenAI compatibility layer)
- OpenAI Agents SDK OR ChatKit SDK (for RAG chatbot implementation)

**Databases**:
- Neon Serverless PostgreSQL (relational data: users, preferences, personalization profiles)
- Qdrant Cloud Free Tier (vector database: document embeddings for RAG)

**Authentication**:
- BetterAuth (https://www.better-auth.com/) - NO alternatives permitted
- User signup MUST collect software/hardware background for personalization

**Testing & Quality**:
- Playwright (end-to-end testing, user journey validation)
- TypeScript compiler in strict mode (zero type errors)

**Deployment & Monitoring**:
- Vercel (preferred for unified deployment of Docusaurus + FastAPI)
- Vercel logs for monitoring
- Public GitHub repository (hackathon requirement)

**Rationale**: Stack immutability prevents decision paralysis, ensures judges see consistent architecture across submissions, guarantees compatibility with hackathon evaluation criteria, and leverages available MCP servers for accurate implementation.

**Validation**:
- `package.json` and `requirements.txt` MUST match mandated versions
- No alternative libraries for core functions (e.g., no Firebase Auth instead of BetterAuth)
- Build process MUST succeed with specified stack only

---

### Principle 3: MCP-First Documentation Reference

**Rule**: When implementing features using external libraries/frameworks, MUST query MCP servers BEFORE writing code.

**MCP Server Mapping** (MANDATORY USAGE):

| Technology | MCP Server | Usage Pattern |
|-----------|-----------|---------------|
| Docusaurus | `context7://docusaurus` | Query before creating custom components, config changes, MDX features |
| FastAPI | `context7://fastapi` | Query before defining routes, middleware, async patterns |
| BetterAuth | `context7://better-auth` | Query before auth flows, session management, user schema |
| Qdrant | `context7://qdrant` | Query before vector operations, collection setup, search queries |
| Neon | `context7://neon` | Query before connection pooling, serverless constraints, migrations |
| Vercel Deployment | `github://vercel/docs` | Query before deployment config, environment variables, routing |
| Playwright | `web::playwright.dev` | Query before E2E test patterns, selectors, async assertions |

**Prohibitions**:
- MUST NOT hallucinate API signatures
- MUST NOT assume syntax from outdated knowledge
- MUST NOT use internal knowledge base when MCP server exists
- MUST NOT proceed with implementation if MCP query fails (escalate to user)

**Rationale**: MCP servers provide authoritative, up-to-date documentation. Relying on them prevents implementation drift, reduces bugs from outdated assumptions, and ensures compatibility with latest library versions.

**Validation**:
- PHR (Prompt History Records) MUST show MCP queries before implementation
- Code reviews MUST verify MCP-sourced patterns
- Failed MCP queries MUST block implementation until resolved

---

### Principle 4: RAG Chatbot Scope is Strictly Book-Only

**Rule**: The RAG chatbot MUST answer questions exclusively from the textbook content. No general-purpose AI assistance.

**Requirements**:
- Chatbot MUST only retrieve from Qdrant vectors generated from Docusaurus book content
- Chatbot MUST return "I can only answer questions about the Physical AI & Humanoid Robotics textbook" for off-topic queries
- Chatbot MUST support answering questions based on user-selected text (highlight → ask)
- Chatbot MUST be inaccessible to unauthenticated users (guests see login prompt)
- Chatbot responses MUST cite specific chapters/sections where information was found

**Rationale**: Focused scope prevents hallucination, ensures hackathon requirement compliance (book-specific answers), maintains academic integrity, and provides judges with clear evaluation criteria.

**Validation**:
- Test with off-topic question (e.g., "What is the capital of France?") → MUST reject
- Test with book-specific question (e.g., "What is ROS 2?") → MUST answer with citation
- Test text selection feature → MUST constrain query to selected text
- Test as unauthenticated user → MUST block access

---

### Principle 5: Authentication Gates Interactive Features

**Rule**: All interactive features (chatbot, personalization, translation) MUST require authentication. Book reading is public.

**Authentication Boundaries**:

**Public Access (No Auth Required)**:
- Read all textbook chapters (Docusaurus static content)
- View table of contents, navigation, search
- Access public metadata (course overview, hardware requirements, weekly breakdown)

**Authenticated Access Required**:
- RAG chatbot interaction
- Content personalization (per-chapter "Personalize" button)
- Urdu translation (per-chapter "Translate to Urdu" button)
- Saving preferences, bookmarks, progress tracking

**BetterAuth Implementation Requirements**:
- Signup flow MUST ask: "What is your software background?" (text field)
- Signup flow MUST ask: "What is your hardware background?" (text field)
- Responses stored in Neon PostgreSQL for personalization engine
- Session management via BetterAuth (no custom session logic)
- No social auth required (email/password sufficient for hackathon)

**Rationale**: Clear auth boundaries simplify security model, enable personalization features, meet hackathon bonus point criteria, and prevent unauthorized resource usage (LLM API costs).

**Validation**:
- Playwright test: Access chatbot without login → MUST redirect to login
- Playwright test: Click "Personalize" without login → MUST show auth modal
- Playwright test: Read chapter without login → MUST succeed
- Database check: User signup MUST populate `software_background` and `hardware_background` columns

---

### Principle 6: Personalization is Chapter-Scoped and On-Demand

**Rule**: Each chapter MUST have a "Personalize Content" button. Personalization applies ONLY to that chapter and lasts for that session.

**Personalization Behavior**:
- Button appears at START of each chapter (before content)
- Clicking "Personalize" triggers API call to FastAPI backend
- Backend retrieves user's software/hardware background from Neon
- Backend uses LLM to generate personalized version of chapter content:
  - Adjust complexity based on background (beginner vs. advanced)
  - Add relevant examples based on prior experience
  - Highlight prerequisites user may need to review
- Personalized content replaces chapter content dynamically (React component swap)
- Personalization is NOT persisted (re-personalizing on page refresh is acceptable for hackathon)

**Implementation Constraints**:
- MUST NOT personalize entire book at once (resource/time prohibitive)
- MUST NOT persist personalized content in database (session-only acceptable)
- MUST NOT alter navigation, structure, or code examples (only explanatory text)

**Rationale**: Chapter-scoped personalization is feasible within hackathon timeline, demonstrates LLM integration, utilizes signup data meaningfully, and provides judges with clear demonstration path.

**Validation**:
- Playwright test: Login with "expert" background → personalize chapter → MUST see advanced concepts
- Playwright test: Login with "beginner" background → personalize chapter → MUST see simplified explanations
- Playwright test: Personalize chapter → refresh page → content reverts to default (acceptable for hackathon)

---

### Principle 7: Urdu Translation is Chapter-Scoped and On-Demand

**Rule**: Each chapter MUST have ONE "Translate to Urdu" button. Translation applies ONLY to that chapter.

**Translation Behavior**:
- Button appears at START of each chapter (alongside "Personalize" button)
- Clicking "Translate to Urdu" triggers API call to FastAPI backend
- Backend uses LLM to translate chapter content to Urdu:
  - Preserve Markdown/MDX structure (headings, lists, code blocks)
  - Preserve code examples in English (only translate explanatory text)
  - Preserve technical terms with Urdu transliteration (e.g., "ROS 2 (روس ٹو)")
- Translated content replaces chapter content dynamically
- Translation is NOT persisted (session-only acceptable for hackathon)
- Button toggles: "Translate to Urdu" ↔ "Show Original English"

**Implementation Constraints**:
- MUST NOT translate code examples, terminal commands, or API names
- MUST NOT translate entire book at once
- MUST NOT use client-side translation (Google Translate widget) - backend LLM required
- MUST preserve RTL (right-to-left) formatting for Urdu text

**Rationale**: On-demand translation demonstrates LLM capability, meets bonus point criteria, provides accessible education for Urdu speakers, and keeps scope manageable for hackathon.

**Validation**:
- Playwright test: Click "Translate to Urdu" → MUST see Urdu text with English code blocks
- Playwright test: Technical terms MUST appear in both Urdu and English transliteration
- Playwright test: Click "Show Original English" → MUST revert to English
- Visual test: Urdu text MUST render right-to-left

---

### Principle 8: Quality Gates are Non-Negotiable

**Rule**: All implementations MUST pass quality gates before considered complete. Failing gates block deployment.

**Mandatory Quality Gates**:

**1. Build Success Gate**:
- `npm run build` (Docusaurus) MUST succeed with zero errors
- `mypy` or `pyright` (FastAPI) MUST pass with zero type errors
- NO TypeScript `any` types except in explicitly typed `unknown` casts
- NO MDX parsing errors (red error boxes in Docusaurus)

**2. Type Safety Gate**:
- TypeScript strict mode enabled (`strict: true` in `tsconfig.json`)
- All React components MUST have explicit prop types
- All FastAPI endpoints MUST use Pydantic models for request/response
- Database queries MUST use typed ORM/query builder

**3. Authentication & Authorization Gate**:
- Unauthenticated users MUST NOT access chatbot API endpoints (401 response)
- Unauthenticated users MUST NOT access personalization endpoints (401 response)
- Authenticated users MUST NOT access other users' data (403 response)
- Session tokens MUST expire (BetterAuth default behavior)

**4. RAG Functionality Gate**:
- Chatbot MUST reject off-topic queries (tested manually with 5+ off-topic questions)
- Chatbot MUST answer book-specific queries (tested with 10+ book questions)
- Chatbot MUST cite sources (chapter/section references)
- Text selection → query MUST constrain to selected text

**5. Deployment Gate**:
- Live URL MUST be publicly accessible (judges test this)
- Vercel deployment MUST include ALL components (Docusaurus + FastAPI + databases)
- Environment variables MUST be properly configured (no hardcoded secrets)
- HTTPS MUST be enabled (Vercel default)

**6. Documentation Gate**:
- README.md MUST include:
  - Project overview and hackathon context
  - Architecture diagram (tech stack visualization)
  - Setup instructions (local development)
  - Deployment instructions (Vercel)
  - Demo credentials (test user for judges)
  - 90-second demo video link
- CLAUDE.md and AGENTS.md MUST be present and current
- All specs MUST be complete (no `[TODO]` placeholders)

**Rationale**: Quality gates prevent technical debt, ensure demo-readiness for judges, validate hackathon requirements, and demonstrate production-grade development practices.

**Validation**:
- CI/CD pipeline MUST run all gates on every commit
- Failed gates MUST block merge to main branch
- Deployment MUST only occur after all gates pass
- Demo video MUST show all gates passing

---

## Hackathon-Specific Constraints

### Scoring Alignment

This constitution ensures maximum point acquisition:

| Requirement | Points | Constitutional Alignment |
|------------|--------|--------------------------|
| Docusaurus book + deployment | 50 | Principle 2 (Tech Stack), Principle 8 (Quality Gates) |
| RAG chatbot (book-only answers) | 50 | Principle 4 (RAG Scope), Principle 3 (MCP-First) |
| Claude Code Subagents & Skills | 50 (bonus) | Referenced in AGENTS.md, enforced via PHR requirement |
| BetterAuth with background questions | 50 (bonus) | Principle 5 (Authentication), Principle 2 (Tech Stack) |
| Per-chapter personalization | 50 (bonus) | Principle 6 (Personalization) |
| Per-chapter Urdu translation | 50 (bonus) | Principle 7 (Translation) |
| **Maximum Total** | **300** | All principles designed for full compliance |

### Timeline Constraints

- **Submission Deadline**: Sunday, Nov 30, 2025 at 06:00 PM
- **Demo Video**: MUST be under 90 seconds (judges only watch first 90 seconds)
- **Live Presentation**: By invitation only (does not affect scoring)

**Constitutional Implications**:
- All specs MUST be finalized by Nov 25 (5 days before deadline)
- All implementations MUST be complete by Nov 28 (2 days before deadline)
- Nov 29-30 reserved for testing, video creation, and deployment validation
- NO feature additions after Nov 28 (only bug fixes)

### Submission Requirements Mapping

| Hackathon Requirement | Constitutional Requirement | Validation |
|----------------------|---------------------------|------------|
| Public GitHub repo | AGENTS.md → Allowed folders, version control | Repo publicly accessible |
| Published book link | Principle 8 → Deployment Gate | Live URL in submission form |
| Demo video (90 sec) | Principle 8 → Documentation Gate | Video < 90 seconds, shows all features |
| WhatsApp number | N/A (user provides) | Included in submission form |

---

## Folder Structure Constraints

**Allowed Root Folders** (from AGENTS.md):
- `.specify/` - All spec artifacts, templates, memory (constitution)
- `apps/` - Application code (if monorepo structure used)
- `docs/` - Docusaurus book content (chapters, assets, config)
- `history/` - Prompt History Records (PHR), ADRs, decision logs
- `.claude/` - Claude Code configuration (hooks, plugins)

**Prohibited Actions**:
- MUST NOT create random folders at root
- MUST NOT create `/agents/*.yaml` (logical roles only, per AGENTS.md)
- MUST NOT create `/skills/*` (caused bugs in previous iteration)
- MUST NOT create frontend/backend split at root without spec approval

---

## Execution Flow (from AGENTS.md)

All implementations MUST follow this mandatory sequence:

```
READ → VALIDATE → EXECUTE → VERIFY → STOP
```

**Detailed Steps**:
1. **READ**: Load `spec.md`, `plan.md`, `tasks.md` for current feature
2. **VALIDATE**: Confirm task scope matches current work, no ambiguities
3. **EXECUTE**: Implement ONLY the current task (no scope expansion)
4. **VERIFY**: Test behavior matches spec acceptance criteria
5. **STOP**: Wait for next instruction (no autonomous feature additions)

**Prohibitions** (from AGENTS.md):
- MUST NOT rewrite working code without spec justification
- MUST NOT "improve" UI without explicit spec requirement
- MUST NOT guess auth logic (query BetterAuth MCP instead)
- MUST NOT merge frontend + backend logic (maintain separation)
- MUST NOT change deployment architecture without constitutional amendment
- MUST NOT use internal knowledge instead of MCP when MCP server exists

---

## Agent Roles & Responsibilities

**Primary Role**: Super Orchestrator (Claude Code)
- Read and validate all spec artifacts
- Execute tasks precisely as documented
- Stop when ambiguity exists (query user)
- Record all major decisions in PHRs

**Logical Sub-Roles** (internal context switching, not separate files):
- **Spec Analyst**: Validate spec completeness, flag ambiguities
- **Planner**: Translate specs into actionable plans with dependency graphs
- **Task Executor**: Implement tasks with strict scope adherence
- **QA Validator**: Verify acceptance criteria, run quality gates
- **Content Reviewer**: Ensure MDX structure, educational clarity
- **Security Reviewer**: Validate auth boundaries, prevent security vulnerabilities

---

## Testing Philosophy

**Tests are Validation, Not Invention**

**Required Test Coverage**:
- **Frontend UI**: Visual regression (Playwright screenshots), component rendering
- **Auth-based visibility**: Chatbot inaccessible to guests, personalization requires login
- **Chatbot scope**: Off-topic rejection, book-specific answers with citations
- **Translation/Personalization**: Single-button trigger, chapter-scoped behavior
- **Build integrity**: Zero MDX errors, TypeScript strict mode compliance

**Prohibited Testing Behavior**:
- MUST NOT write tests that validate invented features
- MUST NOT create tests without corresponding spec requirements
- MUST NOT use tests as documentation substitute (specs are documentation)

---

## Prompt History Record (PHR) Requirements

Claude MUST create PHRs when executing these skills:
- `/sp.specify` - Feature specification creation
- `/sp.plan` - Implementation planning
- `/sp.tasks` - Task breakdown generation
- `/sp.implement` - Major implementation changes
- `/sp.constitution` - Constitutional amendments

**PHR Storage**: `history/prompts/<feature-name>/`
**PHR Format**: Follow `.specify/templates/phr-template.prompt.md`

**Rationale**: PHRs create audit trail for judges, enable learning from prompts, document decision-making process, and satisfy hackathon bonus points for "reusable intelligence via Claude Code Subagents and Agent Skills."

---

## Deployment Philosophy

**User-Controlled Deployment**

Claude may:
- ✅ Prepare deployment configs (Vercel, environment variables)
- ✅ Validate build artifacts before deployment
- ✅ Suggest deployment steps with commands
- ✅ Create deployment documentation in README.md

Claude MUST NOT:
- ❌ Auto-deploy without explicit user approval
- ❌ Modify production environment variables autonomously
- ❌ Push to main branch without user confirmation

**Rationale**: Deployment is high-risk. User maintains control. Judges evaluate deployed artifact, not deployment automation.

---

## Success Criteria

This project is constitutionally compliant when:

1. ✅ **Spec Stability**: All specs complete, no `[TODO]` placeholders, requirements traceable to user stories
2. ✅ **Task Executability**: `sp.implement` runs without ambiguity or prompt engineering
3. ✅ **Bug Reduction**: Bugs resolved via spec updates, not workarounds
4. ✅ **Deployment Integrity**: Deployment succeeds without hacks, all quality gates pass
5. ✅ **Hackathon Compliance**: All 300 possible points achievable, demo video < 90 seconds
6. ✅ **Architectural Clarity**: Judges understand system from specs alone, no code reading required
7. ✅ **MCP Fidelity**: All external library usage documented via MCP queries in PHRs
8. ✅ **Constitutional Adherence**: Zero principle violations in codebase, specs, or tasks

---

## Final Authority

> **When in doubt, STOP and reference this constitution.**

This constitution is the SINGLE SOURCE OF TRUTH for all project decisions. Any conflict between this document and other artifacts (specs, code, prompts) MUST be resolved in favor of the constitution.

If a spec violates constitutional principles, the spec MUST be amended.
If code violates constitutional principles, the code MUST be rewritten.
If a task violates constitutional principles, the task MUST be rejected.

**No exceptions. No shortcuts. No experimental deviations.**

---

**END OF CONSTITUTION**

*This document governs the "Physical AI & Humanoid Robotics Textbook Platform" project for Panaversity Hackathon I. All development MUST align with these principles. Architectural excellence, spec-driven discipline, and hackathon compliance are non-negotiable.*
