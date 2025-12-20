---
id: "0022"
title: "Constitution-Driven Reset: Textbook Specification"
stage: spec
date: 2025-12-20T00:00:00Z
surface: claude-code
model: claude-sonnet-4-5-20250929
feature: 001-ai-textbook-platform
branch: 001-ai-textbook-platform
user: user
command: /sp.specify
labels: [hackathon, constitution, bonus-focused, reset]
links:
  spec: specs/001-ai-textbook-platform/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - path: specs/001-ai-textbook-platform/spec.md
    status: created
  - path: specs/001-ai-textbook-platform/checklists/requirements.md
    status: created
tests:
  - name: Specification Quality Validation
    path: specs/001-ai-textbook-platform/checklists/requirements.md
    status: passed
---

## Prompt

User requested a complete constitution-driven reset for the hackathon project "AI-Native Textbook: Physical AI & Humanoid Robotics". This is a hard reset where all previous bonus implementations were deleted, leaving only the Docusaurus book structure.

### Hackathon Requirements Summary:

**Base Requirements (100 points)**:
1. AI/Spec-Driven Book Creation using Docusaurus, Spec-Kit Plus, and Claude Code
2. 13 weeks of content covering Physical AI & Humanoid Robotics (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA)
3. Integrated RAG Chatbot using OpenAI Agents/ChatKit, FastAPI, Neon Postgres, Qdrant Cloud
4. Deployment to GitHub Pages or Vercel

**Bonus Points (up to 200 total)**:
- +50 points: Reusable intelligence via Claude Code Subagents and Agent Skills
- +50 points: Authentication using Better Auth with user background profiling
- +50 points: Content personalization based on user background (button at chapter start)
- +50 points: Urdu translation with preserved technical terms (button at chapter start)

**Submission Deadline**: Nov 30, 2025 at 6:00 PM (already passed, spec is for reference/iteration)

### Course Content Structure:
- Module 1: The Robotic Nervous System (ROS 2) - Weeks 1-5
- Module 2: The Digital Twin (Gazebo & Unity) - Weeks 6-7
- Module 3: The AI-Robot Brain (NVIDIA Isaac) - Weeks 8-10
- Module 4: Vision-Language-Action (VLA) - Weeks 11-13

### Constitution Alignment Requirements:
- Enforce AGENTS.md agent architecture
- No agent works independently
- No agent invents scope
- No agent violates Constitution
- Mandatory agents: Curriculum Architect, Chapter Author, Robotics Domain Expert, Pedagogy & Simplification, Review & Accuracy

## Response Snapshot

Successfully created comprehensive specification following Spec-Kit Plus methodology and Constitution enforcement:

### Specification Structure:

**Feature Branch**: `001-ai-textbook-platform`

**5 Prioritized User Stories**:
1. **P1**: Read and Navigate Textbook Content (base requirement)
2. **P1**: Ask Questions via RAG Chatbot (base requirement)
3. **P2**: Create Account and Personalize Learning Experience (+50 bonus points)
4. **P2**: Translate Content to Urdu (+50 bonus points)
5. **P2**: Develop Reusable AI Agents for Book Creation (+50 bonus points)

**38 Functional Requirements** organized into 5 categories:
- Core Book Platform: FR-001 to FR-006 (P1)
- RAG Chatbot: FR-007 to FR-015 (P1)
- Authentication & User Management: FR-016 to FR-021 (P2)
- Content Personalization: FR-022 to FR-027 (P2)
- Urdu Translation: FR-028 to FR-033 (P2)
- Reusable AI Intelligence: FR-034 to FR-038 (P2)

**27 Success Criteria** with measurable outcomes:
- Core Book Platform: SC-001 to SC-005
- RAG Chatbot: SC-006 to SC-010
- Authentication & Personalization: SC-011 to SC-015
- Urdu Translation: SC-016 to SC-019
- Reusable AI Intelligence: SC-020 to SC-023
- Hackathon Submission: SC-024 to SC-027

**8 Key Entities**: Chapter, Module, User, Chat Session, Chat Message, Vector Chunk, Personalization Profile, Translation Cache

**6 Edge Cases** covering chatbot ambiguity, conflicting user backgrounds, translation caching, prerequisite warnings, text selection handling, service availability

**10 Documented Assumptions** including infrastructure access, content creation approach, translation strategy, personalization logic, RAG architecture, agent reusability, performance targets, and scope boundaries

**10 Out of Scope Items** explicitly excluding interactive simulations, video content, grading, community features, instructor dashboard, mobile apps, offline access, multi-language beyond Urdu, advanced analytics, and hardware integration

**5 Dependency Categories**: External services, development tools, technical prerequisites, content dependencies, governance documents

**Constitution Alignment Section** documenting enforcement of AGENTS.md and phase discipline

**9 Identified Risks** with mitigations (3 high-priority, 3 medium-priority, 3 low-priority)

### Validation Results:

Created specification quality checklist at `specs/001-ai-textbook-platform/checklists/requirements.md` and validated against all criteria:

**✅ All 12 Checklist Items PASSED**:
- Content Quality: 4/4 passed
- Requirement Completeness: 8/8 passed
- Feature Readiness: 4/4 passed

**Zero [NEEDS CLARIFICATION] markers** - all requirements fully specified using hackathon prompt details and informed assumptions

**Specification is ready for `/sp.plan`** - no clarifications needed

## Outcome

- ✅ Impact: Complete, constitution-aligned specification created for hackathon project covering all base requirements and 4 bonus point categories (200+ potential bonus points)
- 🧪 Tests: Specification quality validation passed 12/12 checklist items
- 📁 Files: Created spec.md (396 lines) and requirements.md validation checklist
- 🔁 Next prompts: `/sp.plan` to create implementation architecture, or `/sp.clarify` if any requirements need user clarification (none identified)
- 🧠 Reflection: Specification successfully balances comprehensive detail with technology-agnostic language. All bonus point features explicitly scoped as independently testable P2 user stories. AGENTS.md integration ensures agent discipline during implementation.

## Evaluation Notes (Flywheel)

- **Failure modes observed**: None - specification process completed successfully with zero clarifications needed due to comprehensive hackathon prompt
- **Graders run and results (PASS/FAIL)**: Specification Quality Checklist - PASS (12/12 items)
- **Prompt variant (if applicable)**: Constitution-Driven Reset variant - emphasizes AGENTS.md enforcement and bonus point maximization
- **Next experiment (smallest change to try)**: Proceed to `/sp.plan` to test whether planning phase can decompose bonus features into parallel implementation tracks (auth/personalization/translation/agents can be developed independently after core book+RAG)
