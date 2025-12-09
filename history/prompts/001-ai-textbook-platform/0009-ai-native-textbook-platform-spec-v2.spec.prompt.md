---
id: 0009
title: AI-Native Textbook Platform Spec v2 (Constitution v5.0.0 Alignment)
stage: spec
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5
feature: 001-ai-textbook-platform
branch: main
user: system
command: /sp.specify
labels: ["specification", "ai-native-textbook", "constitution-v5", "subagent-orchestration", "hackathon"]
links:
  spec: specs/001-ai-textbook-platform/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-ai-textbook-platform/spec.md
 - specs/001-ai-textbook-platform/checklists/requirements.md
tests:
 - Specification quality validation (all criteria passed)
---

## Prompt

```text
I want to generate a feature specification for the complete AI-native technical book titled "Physical AI & Humanoid Robotics" that aligns with the Panaversity hackathon goals. This feature should cover:

1. AI-native textbook creation using Docusaurus.
2. Full chapter breakdown based on the 13-week syllabus.
3. Integration of Claude subagents and skills for auto-generation and validation of content.
4. Personalized content flow based on user profile (captured via BetterAuth).
5. Chapter-level content toggles for:
   - Personalization (user background)
   - Urdu translation
6. Embedded RAG Chatbot with Qdrant, ChatKit SDK, FastAPI backend, capable of answering questions from selected text.
7. Backend API must be auto-deployed to Vercel and integrated with frontend.
8. All environment variables and deployment configs must be set using Vercel MCP.
9. GitHub MCP should auto-push all major commits.
10. Playwright MCP will perform UI testing on book layout and chatbot integration.
11. Context7 MCP to assist with Docusaurus documentation during setup if needed.
12. Bonus: Integration of ROS2-based code examples and robot commands simulation using text-to-code generation for robotics.

Make the spec modular, extensible, and optimized for use with SpecKit Plus and Claude CLI.
```

## Response snapshot

Updated existing specification (001-ai-textbook-platform/spec.md) to align with Constitution v5.0.0 (MAJOR amendment 2025-12-09).

**Key Enhancements:**
- Expanded from 5 to **7 user stories** (added Story 7: P0 - Automated Content Generation via 9 Claude Subagents)
- Increased from 22 to **44 functional requirements** (doubled coverage)
- Grew from 10 to **23 success criteria** (comprehensive measurement framework)
- Added **Story 2 enhancement**: ChatKit contextual text selection (+50 bonus points)
- Added **Story 6**: ROS2 text-to-code generator (additional bonus)
- Documented all **9 specialized Claude subagents** from Constitution Article VIII with acceptance scenarios
- Referenced all **23 canonical + 5 bonus skills** from Constitution Article IX throughout requirements
- Mapped all **4 MCP servers** (Context7, GitHub, Playwright, Vercel) to specific workflows per Constitution Article III

**User Stories:**
- P0 (Foundation): Story 7 - Subagent orchestration (chapter-planner → content-implementor → factual-verifier → validation-auditor → super-orchestrator)
- P1: Story 1 - Browse 13-week AI-native curriculum with Mermaid diagrams, code tabs
- P2: Story 2 - RAG Chatbot with ChatKit contextual text selection, Qdrant vector search
- P2: Story 4 - BetterAuth authentication with onboarding quiz (hardware + programming background)
- P3: Story 3 - AI-powered personalization (RTX 4090 vs Jetson Orin Nano optimization)
- P4: Story 5 - Urdu translation (Roman/Formal) with code preservation
- P5 (Bonus): Story 6 - ROS2 text-to-code playground

**Constitutional Compliance:**
- ✅ All 11 Articles (I-XI) from Constitution v5.0.0 documented
- ✅ Article VIII: All 9 subagents (chapter-planner, spec-architect, pedagogical-designer, educational-validator, content-implementor, factual-verifier, assessment-architect, validation-auditor, super-orchestrator)
- ✅ Article IX: 28 skills total (23 canonical: book-scaffolding, chapter-planner, quiz-generator, etc. + 5 bonus: urdu-translator, user-profile-initializer, ros2-code-generator, rag-chatbot-integrator, personalization-engine)
- ✅ Article X: All 5 bonus features for +200 hackathon points
- ✅ Article XI: Vercel MCP (frontend/backend deployment), GitHub MCP (version control), Playwright MCP (UI testing)

**Hackathon Scoring Strategy:**
- Base 100 points: Stories 1 + 2 (Content + RAG)
- Bonus +50: ChatKit contextual selection
- Bonus +50: BetterAuth (Story 4)
- Bonus +50: Personalization (Story 3)
- Bonus +50: Urdu Translation (Story 5)
- Additional: ROS2 text-to-code (Story 6)
- **Total Target: 300+/100 points**

**Validation Checklist Created:**
- specs/001-ai-textbook-platform/checklists/requirements.md
- All quality criteria PASSED (no [NEEDS CLARIFICATION] markers, all requirements testable, success criteria measurable and technology-agnostic)
- Constitutional compliance verified for all 11 articles
- Ready for `/sp.plan` (no clarifications needed)

## Outcome

- ✅ Impact: Comprehensive specification update aligning existing feature with Constitution v5.0.0, adding subagent orchestration workflow (Story 7 - P0 Foundation), expanding to 7 user stories, 44 functional requirements, 23 success criteria, full hackathon bonus coverage
- 🧪 Tests: Specification quality validation passed all criteria (content quality, requirement completeness, feature readiness, constitutional compliance)
- 📁 Files: specs/001-ai-textbook-platform/spec.md (updated 510 lines), specs/001-ai-textbook-platform/checklists/requirements.md (created)
- 🔁 Next prompts: /sp.plan to create architectural plan with 9 subagent coordination workflow, deployment pipeline design, MCP server integration
- 🧠 Reflection: Specification now serves as complete blueprint for hackathon delivery with 300+/100 point target. Addition of Story 7 (P0 - Foundation) critical for documenting content generation workflow via subagent orchestration. All bonus features mapped to specific Constitution skills. MCP server usage patterns documented per Article III.

## Evaluation notes (flywheel)

- Failure modes observed: None. Specification update followed template requirements, expanded coverage systematically, maintained technology-agnostic language in success criteria.
- Graders run and results (PASS/FAIL): PASS - All validation criteria met (no implementation details in requirements, all success criteria measurable and technology-agnostic, no unresolved clarifications, constitutional compliance verified for all 11 articles)
- Prompt variant (if applicable): spec-v2-constitution-v5-alignment-subagent-orchestration
- Next experiment (smallest change to try): Create plan.md documenting subagent coordination architecture (super-orchestrator workflow), MCP server integration patterns (Context7 for factual-verifier, Playwright for validation-auditor, Vercel/GitHub for deployment), and deployment pipeline design.
