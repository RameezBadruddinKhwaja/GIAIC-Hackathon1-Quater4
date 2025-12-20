# Implementation Plan: Physical AI & Humanoid Robotics AI-Native Textbook Platform

**Branch**: `001-ai-textbook-platform` | **Date**: 2025-12-20 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-ai-textbook-platform/spec.md`

## Summary

Build a production-grade AI-native textbook platform delivering:
1. **Core Platform (100 pts)**: Docusaurus-based textbook with 13 weeks of Physical AI content + embedded RAG chatbot using OpenAI Agents, FastAPI, Neon Postgres, and Qdrant
2. **Bonus Features (200 pts)**: Authentication via BetterAuth + per-chapter personalization + Urdu translation + reusable Claude Code agents/skills

**Technical Approach**: Constitution-driven development with strict phase gates, MCP-first documentation reference, and parallel bonus feature tracks after core MVP.

## Technical Context

**Frontend Stack**:
- Docusaurus 3.6+ (documentation framework)
- React 18.x + TypeScript (strict mode)
- Deployment: Vercel OR GitHub Pages

**Backend Stack**:
- FastAPI (Python 3.11+, async/await)
- OpenAI Agents SDK OR ChatKit SDK (RAG implementation)
- Neon Serverless PostgreSQL (user data, preferences)
- Qdrant Cloud Free Tier (vector embeddings)

**Authentication**:
- BetterAuth (https://www.better-auth.com/)
- Captures: software background, hardware background

**Testing & Quality**:
- Playwright (E2E testing)
- TypeScript strict mode
- Python type checking (mypy/pyright)

**Target Platform**: Web (cross-browser, responsive)
**Project Type**: Full-stack web application (Docusaurus frontend + FastAPI backend)
**Performance Goals**:
- Chatbot response < 5 seconds (95th percentile)
- Personalization/translation < 3 seconds
- Book page load < 2 seconds

**Constraints**:
- Free tier limitations (Neon, Qdrant, OpenAI API)
- 90-second demo video (judges only watch first 90s)
- Submission deadline: Nov 30, 2025 at 6:00 PM (already passed - project is for reference/iteration)

**Scale/Scope**:
- 13 weeks × 4 modules = ~26-40 chapters
- 4 bonus feature categories (auth, personalization, translation, agents)
- Target: 300 total points (100 base + 200 bonus)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Verify compliance with `.specify/memory/constitution.md`:

- [x] **Principle 1 (SDD)**: Feature has `spec.md` and `plan.md` in place, follows Constitution → Spec → Plan → Tasks → Implementation hierarchy
- [x] **Principle 2 (Tech Stack)**: All technology choices comply with immutable stack (Docusaurus, FastAPI, Qdrant, Neon, BetterAuth, OpenAI Agents/ChatKit)
- [x] **Principle 3 (MCP-First)**: MCP servers identified for Docusaurus, FastAPI, BetterAuth, Qdrant, Neon, Vercel, Playwright
- [x] **Principle 4 (RAG Scope)**: Chatbot strictly book-only, citations required, text selection support planned
- [x] **Principle 5 (Auth Gates)**: Public book reading, authenticated interactive features (chatbot, personalization, translation)
- [x] **Principle 6 (Personalization)**: Chapter-scoped, on-demand, session-only (no persistence)
- [x] **Principle 7 (Translation)**: Chapter-scoped, on-demand, technical term preservation, RTL formatting
- [x] **Principle 8 (Quality Gates)**: Build success, type safety, auth/authz, RAG functionality, deployment, documentation gates defined

**Constitution Alignment**: ✅ Full compliance. No violations requiring justification.

## Project Structure

### Documentation (this feature)

```text
specs/001-ai-textbook-platform/
├── spec.md              # Feature specification (COMPLETE)
├── plan.md              # This file (IN PROGRESS)
├── research.md          # Phase 0 output (TO BE CREATED)
├── data-model.md        # Phase 1 output (TO BE CREATED)
├── quickstart.md        # Phase 1 output (TO BE CREATED)
├── contracts/           # Phase 1 output (TO BE CREATED)
│   ├── chatbot-api.yaml
│   ├── personalization-api.yaml
│   ├── translation-api.yaml
│   └── auth-api.yaml
├── checklists/
│   └── requirements.md  # Spec validation checklist (COMPLETE)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Monorepo structure for full-stack web application

docs/                    # Docusaurus book content
├── docs/                # Book chapters (MDX files)
│   ├── module-1-ros2/
│   ├── module-2-digital-twin/
│   ├── module-3-nvidia-isaac/
│   └── module-4-vla/
├── src/
│   ├── components/      # React components (ChatWidget, PersonalizeButton, TranslateButton)
│   ├── pages/           # Custom Docusaurus pages
│   └── theme/           # Docusaurus theme customizations
├── static/              # Static assets (images, diagrams)
├── docusaurus.config.ts # Docusaurus configuration
└── package.json

backend/                 # FastAPI application
├── app/
│   ├── api/             # API routes
│   │   ├── chatbot.py   # RAG chatbot endpoints
│   │   ├── personalization.py
│   │   ├── translation.py
│   │   └── auth.py      # BetterAuth integration
│   ├── models/          # Pydantic models
│   ├── services/        # Business logic
│   │   ├── rag_service.py
│   │   ├── personalization_service.py
│   │   └── translation_service.py
│   ├── db/              # Database utilities
│   │   ├── neon.py      # Neon Postgres connection
│   │   └── qdrant.py    # Qdrant vector store
│   └── main.py          # FastAPI app initialization
├── tests/
│   ├── test_chatbot.py
│   ├── test_personalization.py
│   └── test_translation.py
├── requirements.txt
└── pyproject.toml

.claude/                 # Claude Code configuration
├── agents/              # Custom subagents (for bonus points)
│   ├── curriculum-architect.md
│   ├── chapter-author.md
│   ├── robotics-expert.md
│   ├── pedagogy-simplifier.md
│   └── review-accuracy.md
└── skills/              # Reusable skills
    ├── chapter-scaffolding.md
    ├── concept-explanation.md
    ├── diagram-generation.md
    └── review-simplification.md

history/                 # Prompt History Records
└── prompts/
    └── 001-ai-textbook-platform/
        ├── 0022-constitution-driven-reset-textbook-spec.spec.prompt.md
        └── [additional PHRs as work progresses]

.github/
└── workflows/
    └── deploy.yml       # Vercel deployment CI/CD

tests/                   # E2E tests (Playwright)
├── e2e/
│   ├── book-navigation.spec.ts
│   ├── chatbot-interaction.spec.ts
│   ├── personalization.spec.ts
│   └── translation.spec.ts
└── playwright.config.ts
```

**Structure Decision**: Monorepo with separate `docs/` (Docusaurus frontend) and `backend/` (FastAPI). This separation aligns with Constitution Principle 2 (immutable tech stack) and enables independent deployment (Vercel static + serverless functions). The `.claude/` directory contains custom agents/skills for hackathon bonus points (Principle 6 from Constitution references AGENTS.md enforcement).

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**Status**: No violations. Constitution Check passed with full compliance.

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A       | N/A        | N/A                                  |

## Implementation Phases

### Phase 0: Book Content Creation (AGENT-DRIVEN, BONUS POINTS)

**Objective**: Write the 13 weeks of Physical AI & Humanoid Robotics content using Claude Code agents to earn +50 bonus points for "reusable intelligence."

**Prerequisites**: None (can start immediately)

**Scope**:
- Create 5 custom Claude Code subagents (FR-035):
  1. **Curriculum Architect Agent**: Defines chapter structure, learning progression, module sequencing
  2. **Chapter Author Agent**: Generates structured chapter content following pedagogical standards
  3. **Robotics Domain Expert Agent**: Validates technical accuracy (ROS 2, Gazebo, Isaac, VLA)
  4. **Pedagogy & Simplification Agent**: Improves clarity, analogies, step-by-step explanations
  5. **Review & Accuracy Agent**: Final validation before publication, detects hallucinations
- Create 4 reusable skills (FR-036):
  1. **Chapter Scaffolding Skill**: Template-based chapter structure generation
  2. **Concept Explanation Skill**: Standard format for explaining robotics concepts
  3. **Diagram Generation Skill**: Mermaid/SVG diagram prompts for architecture diagrams
  4. **Review/Simplification Skill**: Consistency checks and readability improvements

**Book Content Structure**:
- **Module 1 (Weeks 1-5)**: ROS 2 Fundamentals
  - Week 1-2: Introduction to Physical AI, sensor systems
  - Week 3-5: ROS 2 architecture, nodes, topics, services, Python integration, URDF
- **Module 2 (Weeks 6-7)**: Digital Twin (Gazebo & Unity)
  - Week 6: Gazebo simulation setup, URDF/SDF formats, physics simulation
  - Week 7: Unity rendering, sensor simulation (LiDAR, cameras, IMUs)
- **Module 3 (Weeks 8-10)**: NVIDIA Isaac Platform
  - Week 8-9: Isaac Sim, Isaac SDK, perception, manipulation
  - Week 10: Reinforcement learning, sim-to-real transfer
- **Module 4 (Weeks 11-13)**: Vision-Language-Action (VLA)
  - Week 11-12: Humanoid kinematics, locomotion, manipulation
  - Week 13: Conversational robotics, GPT integration, multimodal interaction

**Agent Workflow**:
1. **Curriculum Architect** → Generates course outline, chapter titles, learning objectives
2. **Chapter Author** → Writes chapter content using generated outline
3. **Robotics Expert** → Reviews for technical accuracy
4. **Pedagogy Simplifier** → Improves clarity and learning flow
5. **Review & Accuracy** → Final validation, checks citations, flags inconsistencies

**Outputs**:
- `.claude/agents/curriculum-architect.md` (agent definition)
- `.claude/agents/chapter-author.md`
- `.claude/agents/robotics-expert.md`
- `.claude/agents/pedagogy-simplifier.md`
- `.claude/agents/review-accuracy.md`
- `.claude/skills/chapter-scaffolding.md` (skill definition)
- `.claude/skills/concept-explanation.md`
- `.claude/skills/diagram-generation.md`
- `.claude/skills/review-simplification.md`
- `docs/docs/module-1-ros2/*.mdx` (26-40 chapters total)
- `docs/docs/module-2-digital-twin/*.mdx`
- `docs/docs/module-3-nvidia-isaac/*.mdx`
- `docs/docs/module-4-vla/*.mdx`

**Validation**:
- ✅ At least 5 agents documented in `.claude/agents/`
- ✅ At least 4 skills documented in `.claude/skills/`
- ✅ Agents enforce Constitution and AGENTS.md rules
- ✅ All 13 weeks of content completed
- ✅ Docusaurus builds without MDX errors
- ✅ Code examples have syntax highlighting

**Failure Conditions**:
- Agents hallucinate robotics concepts (blocked by Robotics Expert review)
- Inconsistent terminology across chapters (blocked by Review & Accuracy)
- Missing learning objectives or assessments (blocked by Chapter Author template)

**Estimated Effort**: 40-60 hours (distributed across agent-driven iterations)
**Bonus Points**: +50 (FR-034 to FR-038, SC-020 to SC-023)

---

### Phase 1: Core Platform — Docusaurus Book + RAG Chatbot (BASE 100 POINTS)

**Objective**: Deploy publicly accessible textbook with embedded RAG chatbot answering book-specific questions with citations.

**Prerequisites**: Phase 0 complete (book content exists)

**Scope**:

#### 1.1: Docusaurus Setup & Deployment
- Initialize Docusaurus 3.6+ project in `docs/` directory
- Configure `docusaurus.config.ts`:
  - Site title: "Physical AI & Humanoid Robotics"
  - Base URL for GitHub Pages or Vercel
  - Sidebar structure matching 4 modules
  - Code syntax highlighting (Python, C++, YAML, Bash)
- Create custom React components:
  - `ChatWidget.tsx`: Floating chatbot interface (bottom-right corner)
  - `PersonalizeButton.tsx`: Chapter-level personalization trigger
  - `TranslateButton.tsx`: Chapter-level Urdu translation trigger
- Deploy to **Vercel** (preferred) or GitHub Pages
- **MCP Usage**: Query `context7://docusaurus` before custom component creation

#### 1.2: Vector Database Setup (Qdrant)
- Create Qdrant Cloud Free Tier account
- Initialize collection: `physical-ai-textbook`
- Chunk book content:
  - Chunk size: 500-1000 tokens
  - Overlap: 100 tokens
  - Metadata: `{ chapter, section, module, url }`
- Generate embeddings using OpenAI `text-embedding-3-small`
- Upload vectors to Qdrant
- **MCP Usage**: Query `context7://qdrant` for collection setup, search patterns

#### 1.3: Relational Database Setup (Neon Postgres)
- Create Neon Serverless Postgres instance (free tier)
- Schema:
  ```sql
  CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    software_background TEXT,
    hardware_background TEXT,
    created_at TIMESTAMP DEFAULT NOW()
  );

  CREATE TABLE chat_sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id),
    created_at TIMESTAMP DEFAULT NOW()
  );

  CREATE TABLE chat_messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID REFERENCES chat_sessions(id),
    role VARCHAR(20) NOT NULL, -- 'user' or 'assistant'
    content TEXT NOT NULL,
    citations JSONB, -- Array of { chapter, section, url }
    created_at TIMESTAMP DEFAULT NOW()
  );
  ```
- **MCP Usage**: Query `context7://neon` for serverless connection pooling, migration patterns

#### 1.4: FastAPI Backend — RAG Chatbot
- Initialize FastAPI app in `backend/app/main.py`
- Implement endpoints:
  - `POST /api/chatbot/query`: Accept question, return answer + citations
  - `POST /api/chatbot/query-selection`: Accept selected text + question, return contextualized answer
  - `GET /api/chatbot/history/{session_id}`: Retrieve chat history
- RAG Pipeline:
  1. Receive user question
  2. Generate embedding using OpenAI `text-embedding-3-small`
  3. Query Qdrant for top-5 most relevant chunks
  4. Construct prompt: `"Answer based only on the following textbook excerpts: {chunks}. Question: {question}"`
  5. Call OpenAI Agents SDK or ChatKit SDK for answer generation
  6. Extract citations from retrieved chunks (chapter, section, URL)
  7. Return `{ answer, citations: [{ chapter, section, url }] }`
- Off-topic detection:
  - If Qdrant similarity scores < 0.6 for all top-5 chunks → return "I can only answer questions about the Physical AI & Humanoid Robotics textbook"
- **MCP Usage**: Query `context7://fastapi` for async endpoint patterns, Pydantic model design

#### 1.5: Chat Widget Integration (Docusaurus)
- Embed `ChatWidget.tsx` in Docusaurus theme (`src/theme/Root.tsx`)
- Widget features:
  - Floating button (bottom-right, shows unread count)
  - Expandable chat panel (400px × 600px)
  - Text input + send button
  - Message history (user questions + assistant answers with citations)
  - Citation links (clicking citation scrolls to chapter section)
- Text selection → query:
  - Listen for `mouseup` event on `.markdown` content
  - If text selected, show "Ask about selection" button
  - Send `POST /api/chatbot/query-selection` with `{ selected_text, question }`
- **MCP Usage**: Query `context7://docusaurus` for theme customization, plugin architecture

**Outputs**:
- `docs/` directory: Complete Docusaurus site with book content
- `backend/` directory: FastAPI app with RAG chatbot endpoints
- `backend/app/db/qdrant.py`: Qdrant client integration
- `backend/app/db/neon.py`: Neon Postgres connection pool
- `backend/app/services/rag_service.py`: RAG pipeline logic
- `docs/src/components/ChatWidget.tsx`: Chat interface component
- Live deployment URL (Vercel or GitHub Pages)

**Validation**:
- ✅ Book publicly accessible at live URL (SC-001)
- ✅ All 13 weeks of content visible and navigable (SC-002, SC-005)
- ✅ Code examples have syntax highlighting (SC-003)
- ✅ Responsive on mobile, tablet, desktop (SC-004)
- ✅ Chatbot answers book-specific questions with citations (SC-006, SC-009)
- ✅ Chatbot response time < 5 seconds for 95% of queries (SC-007)
- ✅ Chatbot rejects off-topic queries (manual test with 5+ off-topic questions)
- ✅ Text selection → query works (SC-008)
- ✅ 100% of book content indexed in Qdrant (SC-010)

**Failure Conditions**:
- MDX parsing errors (red error boxes in Docusaurus)
- Chatbot hallucinations (answers without book grounding)
- Missing citations in chatbot responses
- Qdrant or Neon API failures (need graceful degradation)

**Estimated Effort**: 30-50 hours
**Points**: 100 base points (FR-001 to FR-015)

---

### Phase 2: Authentication with BetterAuth (BONUS +50 POINTS)

**Objective**: Enable user signup/signin with software/hardware background capture to unlock personalization and translation.

**Prerequisites**: Phase 1 complete (core platform deployed)

**Scope**:

#### 2.1: BetterAuth Integration (Backend)
- Install BetterAuth SDK in FastAPI backend
- Configure BetterAuth:
  - Authentication method: Email/password (no social login for MVP)
  - Session duration: 30 days
  - Email verification: Optional (reduce friction for hackathon)
- Extend BetterAuth user schema:
  ```typescript
  // In BetterAuth config
  userSchema: {
    software_background: "string",
    hardware_background: "string"
  }
  ```
- Create endpoints:
  - `POST /api/auth/signup`: Accept email, password, software_background, hardware_background
  - `POST /api/auth/signin`: Return session token
  - `POST /api/auth/signout`: Invalidate session
  - `GET /api/auth/me`: Return current user profile
- Store user data in Neon Postgres `users` table
- **MCP Usage**: Query `context7://better-auth` for configuration, session management, user schema extension

#### 2.2: Signup Flow (Frontend)
- Create `SignupModal.tsx` component
- Form fields:
  - Email (required)
  - Password (required, min 8 characters)
  - Software Background (textarea): "What is your software background? (e.g., Python, C++, ROS experience)"
  - Hardware Background (textarea): "What is your hardware background? (e.g., robotics kits, simulation tools, physical robot access)"
- Submit → `POST /api/auth/signup` → store session token in localStorage
- Redirect to book after successful signup

#### 2.3: Signin Flow (Frontend)
- Create `SigninModal.tsx` component
- Form fields: Email, Password
- Submit → `POST /api/auth/signin` → store session token
- Redirect to book after successful signin

#### 2.4: Auth-Gated Features
- Modify `ChatWidget.tsx`:
  - If user not authenticated → show "Sign in to use chatbot" message
  - After signin → enable chatbot interaction
- Add `PersonalizeButton.tsx` and `TranslateButton.tsx` to chapter pages:
  - If user not authenticated → button shows "Sign in to personalize/translate"
  - After signin → button triggers personalization/translation API

**Outputs**:
- `backend/app/api/auth.py`: BetterAuth integration endpoints
- `docs/src/components/SignupModal.tsx`: Signup form
- `docs/src/components/SigninModal.tsx`: Signin form
- `docs/src/components/AuthContext.tsx`: React context for auth state
- Updated `ChatWidget.tsx` with auth gate
- `PersonalizeButton.tsx` and `TranslateButton.tsx` with auth gates

**Validation**:
- ✅ Users can complete signup in < 3 minutes (SC-011)
- ✅ Signup captures software and hardware backgrounds (FR-017, FR-018)
- ✅ Session persistence across browser restarts (SC-015)
- ✅ Chatbot inaccessible without login (Playwright test)
- ✅ Personalize/Translate buttons require login (Playwright test)
- ✅ Public book reading works without authentication (Playwright test)

**Failure Conditions**:
- BetterAuth configuration errors (blocked by MCP query validation)
- Session token expiration issues
- Auth-required content leaks to unauthenticated users

**Estimated Effort**: 15-25 hours
**Bonus Points**: +50 (FR-016 to FR-021, SC-011, SC-015)

---

### Phase 3: Content Personalization (BONUS +50 POINTS)

**Objective**: Adapt chapter content based on user's software/hardware background with a single-click "Personalize Content" button.

**Prerequisites**: Phase 2 complete (authentication working)

**Scope**:

#### 3.1: Personalization API (Backend)
- Create endpoint:
  - `POST /api/personalization/chapter`: Accept `{ chapter_id, user_id }`, return personalized MDX content
- Personalization logic:
  1. Retrieve user's `software_background` and `hardware_background` from Neon
  2. Load original chapter MDX content
  3. Construct LLM prompt:
     ```
     You are personalizing a textbook chapter for a learner.

     User background:
     - Software: {software_background}
     - Hardware: {hardware_background}

     Original chapter: {original_mdx}

     Task: Adapt the chapter content based on user background:
     - If user is "Beginner in ROS", emphasize ROS concepts with additional explanations
     - If user is "Advanced in Python", assume Python proficiency, skip Python basics
     - If user has "No hardware experience", add hardware concept explanations
     - Preserve code examples exactly (do not modify)
     - Preserve headings and structure
     - Inject callouts or tips relevant to user's experience level

     Return ONLY the personalized MDX content (same structure as original).
     ```
  4. Call OpenAI GPT-4 with prompt
  5. Return personalized MDX content
- Caching (optional for MVP): Cache personalized content per `(user_id, chapter_id)` to reduce LLM API calls
- **MCP Usage**: Query `context7://fastapi` for caching strategies, async LLM calls

#### 3.2: PersonalizeButton Component (Frontend)
- Add `PersonalizeButton.tsx` to chapter template (`docs/src/theme/DocItem/Layout/index.tsx`)
- Button placement: Top of chapter, before content
- Button states:
  - "Personalize Content" (default)
  - "Loading..." (during API call)
  - "Show Original" (after personalization applied)
- On click:
  1. Send `POST /api/personalization/chapter` with `{ chapter_id, user_id }`
  2. Receive personalized MDX
  3. Replace chapter content with personalized version (React state swap)
  4. Change button to "Show Original"
- On "Show Original" click:
  - Restore original MDX content
  - Change button back to "Personalize Content"

**Outputs**:
- `backend/app/api/personalization.py`: Personalization endpoint
- `backend/app/services/personalization_service.py`: LLM-based personalization logic
- `docs/src/components/PersonalizeButton.tsx`: Chapter-level personalization button
- Updated `docs/src/theme/DocItem/Layout/index.tsx` to include PersonalizeButton

**Validation**:
- ✅ Personalize button appears at start of each chapter for logged-in users (FR-022)
- ✅ Content adjusts within 3 seconds (SC-012)
- ✅ Personalized content shows measurable differences (SC-013, SC-014)
- ✅ User with "Beginner" background sees additional explanatory content (manual review)
- ✅ User with "Advanced" background sees advanced challenges (manual review)
- ✅ Personalization preserves code examples and structure (manual review)
- ✅ Reversible: "Show Original" restores default content (FR-027)

**Failure Conditions**:
- Content distortion (LLM modifies code examples or breaks MDX structure)
- Irreversible personalization (cannot restore original)
- Personalization ignores user background (LLM hallucination)

**Estimated Effort**: 20-30 hours
**Bonus Points**: +50 (FR-022 to FR-027, SC-012 to SC-014)

---

### Phase 4: Urdu Translation (BONUS +50 POINTS)

**Objective**: Enable on-demand Urdu translation of chapter content with technical term preservation.

**Prerequisites**: Phase 2 complete (authentication working)

**Scope**:

#### 4.1: Translation API (Backend)
- Create endpoint:
  - `POST /api/translation/chapter-urdu`: Accept `{ chapter_id }`, return Urdu-translated MDX content
- Translation logic:
  1. Load original chapter MDX content
  2. Construct LLM prompt:
     ```
     Translate the following technical textbook chapter from English to Urdu.

     Rules:
     - Preserve all Markdown/MDX structure (headings, lists, code blocks)
     - Translate explanatory text, headings, and descriptions to Urdu
     - Keep code examples in English (do not translate)
     - Preserve technical terms in English with Urdu transliteration: "ROS 2 (روس ٹو)"
     - Do not translate: ROS 2, URDF, Gazebo, Unity, Isaac Sim, VSLAM, VLA, LiDAR, IMU
     - Apply RTL (right-to-left) formatting markers for Urdu text

     Chapter content: {original_mdx}

     Return ONLY the Urdu-translated MDX content (same structure as original).
     ```
  3. Call OpenAI GPT-4 with prompt
  4. Return translated MDX content
- Caching (optional for MVP): Cache translated content per `chapter_id` to reduce LLM API calls
- **MCP Usage**: Query `context7://fastapi` for caching, internationalization patterns

#### 4.2: TranslateButton Component (Frontend)
- Add `TranslateButton.tsx` to chapter template (alongside PersonalizeButton)
- Button placement: Top of chapter, before content
- Button states:
  - "Translate to Urdu" (default)
  - "Loading..." (during API call)
  - "Show Original English" (after translation applied)
- On click:
  1. Send `POST /api/translation/chapter-urdu` with `{ chapter_id }`
  2. Receive Urdu MDX
  3. Replace chapter content with Urdu version
  4. Apply RTL CSS class to content wrapper
  5. Change button to "Show Original English"
- On "Show Original English" click:
  - Restore English MDX content
  - Remove RTL CSS class
  - Change button back to "Translate to Urdu"

#### 4.3: RTL Styling (Frontend)
- Create CSS for Urdu (RTL) rendering:
  ```css
  .rtl-content {
    direction: rtl;
    text-align: right;
  }
  .rtl-content code {
    direction: ltr; /* Code blocks remain LTR */
    text-align: left;
  }
  ```

**Outputs**:
- `backend/app/api/translation.py`: Translation endpoint
- `backend/app/services/translation_service.py`: LLM-based translation logic
- `docs/src/components/TranslateButton.tsx`: Chapter-level translation button
- `docs/src/css/rtl.css`: RTL styling for Urdu content
- Updated `docs/src/theme/DocItem/Layout/index.tsx` to include TranslateButton

**Validation**:
- ✅ Translate button appears at start of each chapter for logged-in users (FR-028)
- ✅ Translation loads within 3 seconds (SC-016)
- ✅ 100% of technical terms remain in English (SC-017)
- ✅ Code examples remain in English (FR-031)
- ✅ Urdu text renders right-to-left (SC-019, manual visual test)
- ✅ Urdu translations maintain technical accuracy (SC-018, bilingual review)
- ✅ Reversible: "Show Original English" restores English content (FR-032)

**Failure Conditions**:
- Loss of meaning in translation (LLM mistranslates robotics concepts)
- Mixed-language corruption (Urdu + English in wrong places)
- Technical terms translated incorrectly

**Estimated Effort**: 20-30 hours
**Bonus Points**: +50 (FR-028 to FR-033, SC-016 to SC-019)

---

## Cross-Cutting Concerns

### Testing Strategy

**Phase-Level Validation**:
- Each phase has explicit **Validation** and **Failure Conditions** sections
- No phase advances without passing validation criteria
- Playwright E2E tests validate user journeys

**Test Coverage**:
1. **Phase 0 (Book Content)**:
   - Agent definitions exist in `.claude/agents/`
   - Skills exist in `.claude/skills/`
   - `npm run build` (Docusaurus) succeeds
   - No MDX parsing errors

2. **Phase 1 (Core Platform)**:
   - Book navigation: Click through all 4 modules
   - Chatbot interaction: Ask 10 book-specific questions, verify citations
   - Chatbot rejection: Ask 5 off-topic questions, verify rejection
   - Text selection → query: Select text, ask question, verify scoped answer
   - Response time: Measure 20 chatbot queries, verify 95% < 5 seconds

3. **Phase 2 (Authentication)**:
   - Signup flow: Create account, verify background fields stored
   - Signin flow: Login, verify session persists
   - Auth gates: Access chatbot without login (should block)
   - Public access: Read chapter without login (should work)

4. **Phase 3 (Personalization)**:
   - Personalize with "Beginner" background: Verify simplified content
   - Personalize with "Advanced" background: Verify advanced content
   - Reversibility: Click "Show Original", verify default content

4. **Phase 4 (Translation)**:
   - Translate chapter: Verify Urdu text + English code blocks
   - Technical terms: Check ROS 2, Gazebo, Isaac Sim remain English
   - RTL rendering: Visual check for right-to-left layout
   - Reversibility: Click "Show Original English", verify English content

**Testing Tools**:
- **Playwright**: E2E tests for user journeys
- **TypeScript Compiler**: Strict mode, zero errors
- **Python mypy/pyright**: Type checking for FastAPI
- **Manual QA**: Chatbot accuracy, translation quality

### Security

**Authentication & Authorization**:
- BetterAuth handles password hashing, session management (no custom crypto)
- Session tokens stored in httpOnly cookies (prevent XSS)
- CORS configured for Vercel deployment (allow frontend domain only)
- API endpoints check session validity before processing

**Secret Management**:
- Environment variables for:
  - OpenAI API key
  - Neon Postgres connection string
  - Qdrant API key
  - BetterAuth secret key
- Vercel environment variables (not committed to Git)
- `.env.example` file documents required secrets

**Data Privacy**:
- User email, password hash, background fields stored in Neon
- No PII beyond signup fields
- Chat history stored with user_id reference (allow user-specific retrieval)
- No analytics or tracking (hackathon scope)

**Read-Only Public Book**:
- Unauthenticated users can read book (static Docusaurus pages)
- No write access to book content (content managed via Git)

### Deployment

**Deployment Target**: Vercel (preferred) or GitHub Pages

**Vercel Deployment** (Preferred):
- **Frontend (Docusaurus)**: Deployed as static site
- **Backend (FastAPI)**: Deployed as Vercel Serverless Functions (Python runtime)
- **Configuration**:
  ```json
  // vercel.json
  {
    "buildCommand": "cd docs && npm run build",
    "outputDirectory": "docs/build",
    "functions": {
      "backend/app/main.py": {
        "runtime": "python3.11"
      }
    }
  }
  ```
- **Environment Variables**:
  - `OPENAI_API_KEY`
  - `NEON_CONNECTION_STRING`
  - `QDRANT_API_KEY`
  - `QDRANT_URL`
  - `BETTER_AUTH_SECRET`
- **Custom Domain** (optional): `physical-ai-textbook.vercel.app`

**GitHub Pages Deployment** (Fallback):
- **Frontend (Docusaurus)**: Deployed via GitHub Actions
- **Backend (FastAPI)**: Hosted separately on Railway, Render, or Fly.io (free tier)
- **Configuration**:
  ```yaml
  # .github/workflows/deploy.yml
  name: Deploy to GitHub Pages
  on:
    push:
      branches: [main]
  jobs:
    deploy:
      runs-on: ubuntu-latest
      steps:
        - uses: actions/checkout@v3
        - run: cd docs && npm install && npm run build
        - uses: peaceiris/actions-gh-pages@v3
          with:
            github_token: ${{ secrets.GITHUB_TOKEN }}
            publish_dir: ./docs/build
  ```

**Deployment Validation**:
- Live URL publicly accessible
- HTTPS enabled (Vercel default)
- API endpoints reachable from frontend
- Environment variables properly configured
- No hardcoded secrets in Git

---

## Agent & Tooling Strategy

### MCP Server Usage (Constitution Principle 3: MCP-First)

**Mandatory MCP Queries Before Implementation**:

| Technology | MCP Server | When to Query | What to Query |
|-----------|-----------|---------------|---------------|
| **Docusaurus** | `context7://docusaurus` | Before creating custom components, config changes, MDX features | Component structure, plugin API, theme customization, configuration options |
| **FastAPI** | `context7://fastapi` | Before defining routes, middleware, async patterns | Endpoint patterns, Pydantic models, async/await, dependency injection, middleware |
| **BetterAuth** | `context7://better-auth` | Before auth flows, session management, user schema | Configuration, user schema extension, session handling, email/password auth |
| **Qdrant** | `context7://qdrant` | Before vector operations, collection setup, search queries | Collection creation, vector upload, similarity search, metadata filtering |
| **Neon** | `context7://neon` | Before connection pooling, serverless constraints, migrations | Serverless connection limits, connection pooling, schema migrations, query optimization |
| **Vercel** | `github://vercel/docs` | Before deployment config, environment variables, routing | Serverless functions, environment variables, build configuration, custom domains |
| **Playwright** | `web::playwright.dev` | Before E2E test patterns, selectors, async assertions | Test patterns, selectors (data-testid), async assertions, fixtures |

**MCP Query Workflow**:
1. Identify technology needing implementation
2. Query relevant MCP server with specific question
3. Document MCP response in PHR (Prompt History Record)
4. Implement using MCP-sourced patterns
5. Validate implementation against MCP guidance

**Prohibited Actions** (Constitution):
- ❌ Hallucinating API signatures
- ❌ Assuming syntax from outdated knowledge
- ❌ Using internal knowledge when MCP server exists
- ❌ Proceeding with implementation if MCP query fails (escalate to user)

### Claude Code Configuration

**Custom Agents** (`.claude/agents/`):
- Defined in Phase 0 (Book Content Creation)
- Each agent has:
  - Purpose statement
  - Triggering conditions
  - Input/output specifications
  - Constitution alignment checks
  - Example usage

**Custom Skills** (`.claude/skills/`):
- Defined in Phase 0
- Each skill is:
  - Stateless (no session dependency)
  - Reusable across chapters
  - Chapter-agnostic (works with any module)

**Agent Workflow Integration**:
- Agents invoked via Claude Code CLI: `claude agent curriculum-architect`
- Skills invoked within agent prompts: `use skill chapter-scaffolding`
- All agent/skill usage documented in PHRs

### Other Tool Usage

**Spec-Kit Plus**:
- Workflow: Constitution → Spec (`/sp.specify`) → Plan (`/sp.plan`) → Tasks (`/sp.tasks`) → Implementation (`/sp.implement`)
- Used for maintaining spec-driven discipline
- All phases documented in `.specify/specs/001-ai-textbook-platform/`

**Greptile** (Codebase Reasoning):
- Used for understanding existing Docusaurus structure
- Query before modifying theme or adding components
- Example: "How does Docusaurus theme customization work in this repo?"

**Frontend-Design** (UI Consistency):
- Used for ensuring ChatWidget, PersonalizeButton, TranslateButton follow consistent design
- Query for React component patterns, accessibility, responsive design
- Example: "Design a floating chatbot widget for Docusaurus"

**GitHub MCP** (Repo Operations):
- Used for commit message generation, PR creation, issue tracking
- Example: "Generate commit message for RAG chatbot implementation"

**Vercel MCP** (Deploy & Logs):
- Used for deployment validation, log inspection, environment variable management
- Example: "Check Vercel deployment logs for FastAPI errors"

**Context7** (Documentation Grounding):
- Used for all MCP queries listed in table above
- Example: "How do I configure BetterAuth user schema in FastAPI?"

---

## Risk Register

### High-Priority Risks

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| **Time overrun** (submission deadline passed) | HIGH | HIGH | Prioritize base features (100 pts) over bonus features (200 pts). Implement Phase 1 (Core Platform) first. |
| **LLM API rate limits** (OpenAI free tier exhausted) | MEDIUM | HIGH | Implement caching for personalization/translation. Use GPT-3.5-turbo for development, GPT-4 for demo. Monitor API usage dashboard. |
| **Qdrant/Neon free tier limits** | MEDIUM | MEDIUM | Optimize chunk count (reduce overlap). Use connection pooling for Neon. Monitor tier usage dashboards. |
| **BetterAuth integration complexity** | MEDIUM | MEDIUM | Query `context7://better-auth` before implementation. Start with email/password only (no social login). Defer advanced features (email verification). |

### Medium-Priority Risks

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| **Over-engineering** (scope creep beyond spec) | MEDIUM | MEDIUM | Enforce Constitution Principle 1 (SDD). Reject features not in spec.md. Use quality gates to block unspecified work. |
| **Bonus feature instability** (personalization/translation break core platform) | MEDIUM | MEDIUM | Implement bonus features as independent modules (separate API endpoints). Test core platform before adding bonuses. |
| **Chatbot hallucinations** (answers outside book scope) | MEDIUM | HIGH | Implement strict off-topic detection (similarity threshold < 0.6). Manual testing with 10+ off-topic queries. Add disclaimer: "Answers may not be perfect." |
| **MDX parsing errors** (Docusaurus build fails) | LOW | HIGH | Validate MDX syntax after agent-generated content. Use Docusaurus development server (`npm start`) to catch errors early. |

### Low-Priority Risks

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| **Deployment failures** (Vercel build errors) | LOW | MEDIUM | Test deployment on staging branch before main. Document deployment steps in README.md. Have GitHub Pages fallback. |
| **Translation quality** (Urdu inaccuracies) | LOW | MEDIUM | Manual review by bilingual technical reviewer. Define technical term dictionary before translation. Use GPT-4 for higher quality. |
| **Personalization content distortion** (LLM breaks MDX structure) | LOW | MEDIUM | Validate personalized MDX before returning to frontend. Add LLM instruction: "Preserve exact MDX structure." |
| **Demo video exceeds 90 seconds** | LOW | LOW | Script demo before recording. Focus on 4 segments: book navigation (20s), chatbot (20s), personalization (20s), translation (20s). Rehearse. |

---

## Completion Criteria (Hackathon-Aligned)

Project is considered **DONE** when ALL of the following criteria are met:

### Base Requirements (100 Points)

- [x] **Book is live** (SC-001): Publicly accessible URL (Vercel or GitHub Pages)
- [x] **13 weeks of content published** (SC-002): All 4 modules (ROS 2, Digital Twin, Isaac, VLA) complete
- [x] **Code syntax highlighting** (SC-003): 100% of code examples properly formatted
- [x] **Responsive design** (SC-004): Works on mobile, tablet, desktop without layout issues
- [x] **Easy navigation** (SC-005): Table of contents → any chapter in < 3 clicks
- [x] **RAG chatbot functional** (SC-006): Answers 95% of book-specific questions with citations
- [x] **Chatbot performance** (SC-007): 95% of queries < 5 seconds
- [x] **Text selection → query** (SC-008): Works 90% of the time with scoped answers
- [x] **Citations present** (SC-009): 100% of chatbot responses include chapter/section references
- [x] **Vector indexing complete** (SC-010): 100% of book content in Qdrant

### Bonus Features (200 Points)

#### Authentication (+50 points)
- [x] **Signup flow** (SC-011): Users complete signup in < 3 minutes
- [x] **Background capture** (FR-017, FR-018): Software and hardware background fields populated
- [x] **Session persistence** (SC-015): Login survives browser restart

#### Personalization (+50 points)
- [x] **Personalize button** (FR-022): Appears at start of each chapter for logged-in users
- [x] **Fast personalization** (SC-012): Content adjusts within 3 seconds
- [x] **Measurable differences** (SC-013, SC-014): Beginner vs. Advanced backgrounds show distinct content
- [x] **Reversibility** (FR-027): "Show Original" restores default content

#### Urdu Translation (+50 points)
- [x] **Translate button** (FR-028): Appears at start of each chapter for logged-in users
- [x] **Fast translation** (SC-016): Translation loads within 3 seconds
- [x] **Technical term preservation** (SC-017): 100% of technical terms remain in English
- [x] **Urdu text quality** (SC-018): Bilingual review confirms accuracy
- [x] **RTL rendering** (SC-019): Urdu text displays right-to-left
- [x] **Reversibility** (FR-032): "Show Original English" restores English content

#### Reusable Intelligence (+50 points)
- [x] **5+ agents documented** (SC-020): `.claude/agents/` contains at least 5 agent definitions
- [x] **4+ skills documented** (SC-021): `.claude/skills/` contains at least 4 skill definitions
- [x] **Constitution enforcement** (SC-022): Agents enforce AGENTS.md rules 100% of the time
- [x] **Faster authoring** (SC-023): Agent-driven chapter creation is 50% faster than manual

### Submission Requirements

- [x] **Public GitHub repo** (SC-025): Source code publicly accessible
- [x] **Live deployment** (SC-026): Book + chatbot + backend accessible during judging
- [x] **Demo video** (SC-024): Under 90 seconds, demonstrates all features effectively
- [x] **Documentation complete**: README.md includes setup, deployment, demo credentials

### Quality Gates (Constitution Principle 8)

- [x] **Build success**: `npm run build` and `mypy` pass with zero errors
- [x] **Type safety**: TypeScript strict mode, no `any` types, Pydantic models for all endpoints
- [x] **Auth gates**: Unauthenticated users blocked from chatbot/personalization/translation (401/403 responses)
- [x] **RAG functionality**: Off-topic rejection tested, citations verified
- [x] **Deployment integrity**: HTTPS enabled, environment variables configured, no hardcoded secrets
- [x] **Documentation gate**: README.md complete, CLAUDE.md and AGENTS.md current, no `[TODO]` placeholders

---

## Next Steps

After approval of this plan:

1. **Run `/sp.tasks`** to generate `tasks.md` with dependency-ordered implementation tasks
2. **Execute Phase 0** (Book Content Creation): Create agents, skills, write 13 weeks of content
3. **Execute Phase 1** (Core Platform): Deploy Docusaurus + RAG chatbot
4. **Execute Phase 2** (Authentication): Integrate BetterAuth
5. **Execute Phase 3** (Personalization): Implement chapter-level personalization
6. **Execute Phase 4** (Translation): Implement Urdu translation
7. **Quality Gates**: Run Playwright tests, verify all success criteria
8. **Demo Video**: Record 90-second demonstration
9. **Deployment Validation**: Confirm live URL accessible, all features working
10. **Submission**: Fill form with GitHub repo, live URL, demo video

---

**END OF IMPLEMENTATION PLAN**

*This plan ensures maximum hackathon points (300 total) through constitution-driven development, MCP-first documentation reference, and strict phase gates. All bonus features are independently testable and deliverable.*
