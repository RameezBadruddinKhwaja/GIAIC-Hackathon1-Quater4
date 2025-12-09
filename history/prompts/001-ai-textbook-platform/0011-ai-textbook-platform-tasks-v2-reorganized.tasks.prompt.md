---
id: 0011
title: AI Textbook Platform Tasks v2 Reorganized
stage: tasks
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5
feature: 001-ai-textbook-platform
branch: main
user: system
command: /sp.tasks
labels: ["tasks", "reorganized", "category-structure", "243-tasks", "subagent-skill-mapping"]
links:
  spec: specs/001-ai-textbook-platform/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-ai-textbook-platform/tasks.md
tests:
 - Manual testing strategy documented for all 7 user stories
 - Independent test criteria per category
 - Playwright MCP UI testing integration
---

## Prompt

```text
/sp.tasks

Please generate a complete and dependency-ordered tasks.md for the feature branch `001-ai-textbook-platform`. Use the finalized specification (v2) at `specs/001-ai-textbook-platform/spec.md` and align all deliverables with Constitution v5.0.0.

Key instructions:
- Use the 7 user stories and 44 functional requirements to derive detailed tasks.
- Group tasks under categories: Planning, Specification, Content Authoring, Validation, Assessment, Bonus Features, Deployment.
- Link each task to responsible Claude subagent (from Article VIII).
- Include all 28 skills (23 canonical + 5 bonus) wherever needed.
- Add MCP integration steps (Context7, GitHub, Playwright, Vercel).
- Define clear start and end points, and identify parallelizable tasks.
- Ensure tasks cover both frontend (Docusaurus) and backend (RAG chatbot with FastAPI).
- Keep hackathon scoring focus — ensure +200 bonus features are explicitly broken into tasks.

Goal: Create a fully orchestrated execution roadmap for maximum hackathon score.
```

## Response snapshot

Completely reorganized tasks.md with 243 tasks across 7 workflow categories, with explicit Claude subagent assignments and skill integrations throughout.

**New Category Structure (User-Requested)**:
- **Category 1: Planning & Infrastructure Setup** (64 tasks) - Monorepo, Gemini, Neon, Qdrant, 9 agents, 28 skills, agentic infrastructure
- **Category 2: Specification & Design** (12 tasks) - Curriculum architecture via chapter-planner, pedagogical design, spec validation
- **Category 3: Content Authoring** (25 tasks) - 13-week Docusaurus textbook with Mermaid diagrams, code tabs, Hardware Lab Guide (MVP: 100/100 points)
- **Category 4: Validation & Quality Assurance** (21 tasks) - Technical accuracy via Context7 MCP, educational alignment, format validation, Playwright MCP UI testing
- **Category 5: Assessment & Interactive Features** (14 tasks) - Quizzes, coding challenges, module assessments
- **Category 6: Bonus Features** (74 tasks) - RAG Chatbot (+50), BetterAuth (+50), Personalization (+50), Urdu Translation (+50), ROS2 Text-to-Code (bonus)
- **Category 7: Deployment & Production Readiness** (33 tasks) - Health checks, security, GitHub MCP, Vercel MCP, Playwright MCP verification

**Key Improvements from v1**:
1. **Explicit Agent Assignments**: Every task now shows responsible Claude subagent in format `[agent-name]` (e.g., `[content-implementor]`, `[super-orchestrator]`, `[factual-verifier]`)
2. **Explicit Skill Integrations**: Every task shows applicable skills in format `[skill1, skill2, skill3]` from Constitution Article IX (e.g., `[code-example-generator, concept-scaffolding, book-scaffolding]`)
3. **MCP Integration Markers**: Tasks using MCP servers explicitly noted (e.g., `[Context7 MCP]`, `[GitHub MCP]`, `[Playwright MCP]`, `[Vercel MCP]`)
4. **Category-Based Workflow**: Tasks grouped by development phase instead of user story, enabling clearer execution flow
5. **Scoring Transparency**: Each category header shows hackathon point contribution (e.g., Category 3 = 100/100 base, Category 6 = +200 bonus)

**Task Format Enhancement**:
```
Old: - [ ] T001 [P] [US1] Description
New: - [ ] T001 [P] [super-orchestrator] [tool-selection-framework] Description
```

**Agent-Skill-Task Mapping Examples**:
- **content-implementor**: T082-T094 (13 week MDX files) + T137-T159 (RAG chatbot) + T178-T210 (personalization, localization, ROS2 codegen)
- **factual-verifier**: T102-T107 (technical validation with Context7 MCP)
- **validation-auditor**: T113-T122 (format validation + Playwright MCP UI testing) + T230-T236 (deployment verification)
- **assessment-architect**: T123-T136 (quizzes, challenges, module assessments)
- **super-orchestrator**: T001-T064 (foundation setup) + T211-T243 (deployment + final verification)

**MCP Integration Points (Explicit)**:
- **Context7 MCP**: T102-T106 (factual verification: fetch ROS 2, Gazebo, Unity, Isaac docs)
- **GitHub MCP**: T223-T226 (create repo, workflows, push files)
- **Playwright MCP**: T118-T122 (local UI testing), T230-T236 (production verification)
- **Vercel MCP**: T227-T229 (frontend/backend deployment, env var config)

**Parallelization Strategy**:
- Category 1: 45 tasks parallelizable (9 agents + 28 skills + 5 models + 3 misc)
- Category 3: 13 tasks parallelizable (13 week MDX files)
- Category 5: 10 tasks parallelizable (quizzes + coding challenges)
- **Total: ~68 tasks (28% of 243) can run in parallel**

**Constitutional Compliance Documentation**:
- All 9 subagents from Article VIII assigned throughout tasks
- All 28 skills from Article IX integrated in task descriptions
- All 4 MCP servers from Article III and XI mapped to specific tasks
- Hackathon scoring strategy aligned with Article X (Bonus Features)

**Dependency Graph Unchanged**:
- Critical path: Cat1 → Cat2 → Cat3 (MVP 100/100)
- Bonus path: Cat1 + Cat3 → Cat6.2 (Auth) → Cat6.4 & Cat6.5 (Personalization/Localization)
- Cat6.1 (RAG) can run parallel with Cat6.2 (Auth)
- Deployment (Cat7) requires all categories complete

## Outcome

- ✅ Impact: Complete reorganization of tasks.md with 243 tasks across 7 workflow categories, explicit agent/skill assignments, MCP integration markers, and hackathon scoring transparency. Tasks now aligned with user-requested category structure (Planning → Specification → Content Authoring → Validation → Assessment → Bonus Features → Deployment).
- 🧪 Tests: Manual testing strategy per category with independent criteria. Playwright MCP UI testing integrated in Cat4 (T118-T122) and Cat7 (T230-T236).
- 📁 Files: specs/001-ai-textbook-platform/tasks.md (completely rewritten, 652 lines)
- 🔁 Next prompts: /sp.implement to begin execution starting with Category 1 (Planning & Infrastructure), or selectively implement MVP-first (Categories 1-3) for 100/100 baseline
- 🧠 Reflection: Reorganization improves clarity by grouping tasks by development phase rather than user story. Explicit agent/skill assignments enable direct tracing to Constitution Articles VIII & IX. MCP integration markers highlight automation opportunities. Category structure supports incremental delivery: Cat1-3 = MVP (100/100), Cat1-6 = Full (300/100). Task format `[agent] [skills] description` provides complete execution context per task.

## Evaluation notes (flywheel)

- Failure modes observed: None. Tasks reorganization followed user-requested category structure, maintained all 243 tasks from v1, enhanced every task with explicit agent/skill assignments, added MCP integration markers, preserved dependency graph.
- Graders run and results (PASS/FAIL): PASS - All 7 categories present, all 9 subagents assigned, all 28 skills integrated, all 4 MCP servers mapped, dependency graph preserved, parallelization strategy maintained, hackathon scoring transparent
- Prompt variant (if applicable): tasks-v2-category-reorganized-agent-skill-explicit
- Next experiment (smallest change to try): Begin MVP-first implementation with /sp.implement focusing on Categories 1-3 (64+12+25=101 tasks) to achieve 100/100 baseline before adding bonus features. This validates foundation before committing to full 243-task execution.
