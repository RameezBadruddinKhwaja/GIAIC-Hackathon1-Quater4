---
id: 0005
title: Matrix Gemini Edition Tasks Generation
stage: tasks
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-ai-textbook-platform
branch: 001-ai-textbook-platform
user: system
command: /sp.tasks
labels: ["task-generation", "matrix-protocol", "gemini", "monorepo", "user-stories", "hackathon"]
links:
  spec: specs/001-ai-textbook-platform/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-ai-textbook-platform/tasks.md (created, 155 tasks across 7 phases organized by user story priority)
tests:
  - Task format validation: All 155 tasks follow checkbox format ✅
  - User story mapping: All 5 user stories have dedicated phases ✅
  - Parallel opportunities: 45 tasks marked [P] for parallel execution ✅
  - Gemini configuration: All AI tasks reference gemini_client.py and GEMINI_API_KEY ✅
  - Matrix Protocol: Phase 1.3 creates .claude/ infrastructure, Phase 3.3 implements skill loading ✅
---

## Prompt

**Task Generation Strategy: The "Matrix" & "Gemini" Edition**

**Objective:**
Generate a comprehensive `tasks.md` that breaks down the implementation into independent, testable phases based on the Spec and Plan.

**Core Organization:**
Group tasks by **User Story Priority** to ensure we hit the MVP and Bonus targets systematically.

**Phase 1: Foundation & Agentic Infrastructure (CRITICAL)**
*Goal: Setup the Monorepo and the "Virtual Institute".*
- Initialize Monorepo: `apps/docs` (Docusaurus) and `apps/api` (FastAPI).
- **Agent Setup:** Create tasks to manually create the markdown files in `.claude/agents/` (Author, Coder, Architect) and `.claude/skills/`.
- **Environment:** Setup `.env` template with `GEMINI_API_KEY` (mapped to OpenAI Base URL config).
- **Database:** Setup Neon (Postgres) schema and Qdrant connection.

**Phase 2: The Content Engine (User Story 1 - P1)**
*Goal: A working Textbook.*
- Configure Docusaurus v3 with Mermaid.js & MDX support.
- Create skeleton `.md` files for all 4 Modules (ROS 2, Digital Twin, Isaac, VLA).
- Implement the "Hardware Lab Guide" page (Sim vs Edge).

**Phase 3: The Intelligence (User Story 2 & 6 - P2/P6)**
*Goal: The RAG Chatbot using Gemini.*
- Implement `ingest.py` to index docs into Qdrant.
- Implement FastAPI backend using **OpenAI SDK** pointing to **Google Gemini 1.5**.
- Implement the "Matrix Skill Loader" logic (Dynamic System Prompts).
- Create the React Chat Widget in Docusaurus.

**Phase 4: The Bonus Features (User Stories 3, 4, 5)**
*Goal: Maximize Hackathon Points.*
- **Auth:** Implement Better-Auth integration and Onboarding Quiz.
- **Personalization:** Implement the "Personalize" button (Hardware-aware content toggling).
- **Localization:** Implement the "Translate to Urdu" button (Gemini-powered on-the-fly translation).

**Directive:**
- Use `[P]` marker for tasks that can run in parallel.
- Ensure every task references the specific file path it modifies.
- **Strict Rule:** Phase 1 (Infrastructure) must be completed before Phase 3 (Intelligence).

## Response snapshot

Successfully generated comprehensive tasks.md with **155 tasks** organized into **7 phases** following Matrix & Gemini strategy for hackathon targeting 300/100 points. All tasks strictly follow required checkbox format with Task IDs, [P] markers for parallel execution, [Story] labels for user story phases, and exact file paths.

## Outcome

- ✅ Impact: Comprehensive tasks.md generated with 155 tasks organized by user story priority; enables incremental delivery (MVP first, then 4 bonus features); targets 300/100 hackathon score with clear dependency graph and parallel execution opportunities (45 parallelizable tasks); Matrix Protocol infrastructure prioritized in Phase 1.3 as CRITICAL
- 🧪 Tests: Task format validation (155/155 tasks follow checkbox + ID + [P?] + [Story?] + file path format), user story mapping (5 stories → 7 phases), parallel opportunities (45 tasks marked [P]), Gemini configuration (all AI operations use gemini_client.py + GEMINI_API_KEY), Matrix Protocol coverage (Phase 1.3 agents/skills + Phase 3.3 dynamic loading)
- 📁 Files: 1 artifact created (tasks.md: 155 tasks, 7 phases, Mermaid dependency graph, implementation strategy with MVP path, parallel execution guide, hackathon scoring breakdown)
- 🔁 Next prompts: Begin implementation with Phase 1 (Foundation) using `/sp.implement` command; Prioritize T011-T019 (agentic infrastructure) and T034-T036 (Gemini client) for Matrix Protocol and AI readiness; Consider creating ADR for "Matrix Protocol Dynamic Skill Loading Strategy" after Phase 3.3 completion
- 🧠 Reflection: User-provided Matrix & Gemini strategy aligned perfectly with spec.md user story priorities (P1-P4); Task generation successfully decomposed 5 user stories into 155 granular, testable tasks with clear dependencies; Gemini configuration emphasized throughout (GEMINI_API_KEY, 768-dim embeddings, base_url reminders); Matrix Protocol infrastructure (.claude/ agents/skills) correctly marked as CRITICAL blocking dependency for Intelligence phase

## Evaluation notes (flywheel)

- Failure modes observed: None - user directive provided comprehensive phase structure matching spec.md organization; all 5 user stories successfully mapped to phases with independent test criteria
- Graders run and results (PASS/FAIL): PASS - All 155 tasks validated (checkbox format, sequential IDs T001-T155, 45 [P] markers, user story labels [US1]-[US5], file paths present), dependency graph correct (Phase 1 blocks Phase 3, US4 blocks US3/US5), parallel opportunities identified (29% of tasks), Gemini configuration consistent
- Prompt variant (if applicable): tasks-matrix-gemini-hackathon-v1 (explicit agentic infrastructure phase, Gemini drop-in replacement throughout, hackathon scoring optimization with MVP-first strategy)
- Next experiment (smallest change to try): Monitor Phase 1.3 implementation (.claude/ agents/skills creation) to ensure Matrix Protocol infrastructure exists before Phase 3.3 (skill loading); Track parallel execution of Phase 2 content tasks (T042-T054) for time efficiency validation
