---
id: 0003
title: Hackathon Orchestration Architecture v5.0.0
stage: constitution
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5
feature: none
branch: main
user: system
command: /sp.constitution
labels: ["constitution", "orchestration", "hackathon", "subagents", "skills", "mcp-integration"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - CLAUDE.md
tests:
 - none
---

## Prompt

```text
You are an expert AI-native textbook development assistant working for Panaversity. I am creating a technical textbook titled "Physical AI & Humanoid Robotics" for a hackathon. Your job is to orchestrate the complete book generation, testing, and deployment pipeline using Spec-Kit Plus, Claude Subagents, Skills, and integrated MCPs.

## GOAL
Your primary objective is to create a Spec-Driven AI-native textbook project that fulfills all mandatory and bonus requirements for the hackathon:
- Book structure, writing, validation, and deployment using Claude Code + Spec-Kit Plus.
- Integration of RAG chatbot (FastAPI + Qdrant + OpenAI Agent + ChatKit).
- Signup/Signin using BetterAuth and user-background-based content personalization.
- Urdu translation and content customization via chapter-level toggles.

## SYSTEM
Use MCP-connected services to automate the pipeline:
- Use GitHub MCP to push updates to GitHub. Ask for authentication via CLI when needed.
- Use Vercel MCP to deploy both frontend (Docusaurus book) and backend (RAG API). If Vercel does not support mono-repo deploys, split and deploy separately.
- Use Playwright MCP for UI testing and web crawling if needed (e.g., chatbot UI or chapter checks).
- Use Context7 MCP for deeply reading Docusaurus documentation or other relevant pages.

## AGENT ARCHITECTURE
Design the following Claude Subagents under `.claude/agents`, connect them with relevant Skills:

1. `chapter-planner`: Divide book into 13 parts based on course weeks/modules from syllabus.
2. `spec-architect`: Design initial book spec from provided syllabus and hackathon goals.
3. `pedagogical-designer`: Apply learning objectives and outcome-driven design to each chapter.
4. `educational-validator`: Ensure content matches learning outcomes and core robotics topics.
5. `content-implementor`: Write rich, agent-friendly markdown using canonical styles.
6. `factual-verifier`: Validate every chapter for accuracy, especially for ROS 2, Isaac, Gazebo, Unity, Whisper.
7. `assessment-architect`: Add quizzes, assignments, and challenges per module.
8. `validation-auditor`: Run style/format validation and correctness checks across book.
9. `super-orchestrator`: Connect all agents to build, write, validate, and finalize content.

## RELEVANT SKILLS TO CONNECT (From `.claude/skills`)

Connect the above agents with following skillsets:

- `book-scaffolding`
- `chapter-planner`
- `concept-scaffolding`
- `summary-generator`
- `quiz-generator`
- `technical-clarity`
- `canonical-format-checker`
- `assessment-builder`
- `mvp-builder`
- `learning-objectives`
- `docusaurs-deployer`
- `prompt-template-designer`
- `code-example-generator`
- `exercise-designer`
- `frontend-design`
- `validation-auditor`
- `skill-creator`
- `playwright-test-runner` (for UI test via MCP)
- `image-generator` (for concept diagrams)
- `ux-evaluator`
- `tool-selection-framework`
- `summary-generator`
- `notebooklm-slides` (optional slides gen)
- `session-intelligence-harvester` (for personalization logic)

## BONUS FEATURES (Subagents/Skills)
Enable the following features to earn bonus points:

- Chapter-level personalization via toggle using skill: `session-intelligence-harvester`
- Urdu translation toggle: implement via Claude translation skill (name it `urdu-translator`)
- Signup/Signin UI via BetterAuth, collect user tech background (skill: `user-profile-initializer`)
- Chatbot integration that uses selected text via contextual RAG flow (OpenAI Agents, Qdrant, ChatKit, FastAPI)
- Add one skill to simulate robot commands using ROS2 text-to-code (e.g., "Pick up object" => Python ROS2 code snippet)

## DEPLOYMENT
- Frontend (Docusaurus): deploy to Vercel via Vercel MCP
- Backend (FastAPI RAG Chatbot): deploy to Vercel. If mono-repo not supported, deploy as separate service.
- Use GitHub MCP to commit/push book contents to GitHub
- Use `.env` auto-config via Vercel MCP for any backend variables

## FINAL NOTES
- This book should follow canonical style.
- Write clean markdown with reusable intelligent blocks.
- Use Claude CLI for everything — from agent creation, skill connection to final push.
```

## Response snapshot

Updated constitution from v4.1.0 to v5.0.0 (MAJOR) with comprehensive hackathon orchestration architecture. Key changes:

**New Articles Added:**
- **Article VIII: Agentic Orchestration Architecture** - Defines 9 specialized Claude Subagents (chapter-planner, spec-architect, pedagogical-designer, educational-validator, content-implementor, factual-verifier, assessment-architect, validation-auditor, super-orchestrator) with explicit responsibilities and skill mappings.

- **Article IX: Skill System** - Codifies 23 required canonical skills + 5 bonus skills with agent-skill mapping matrix. Includes meta-skill (skill-creator) for dynamic skill generation.

- **Article X: Bonus Features Configuration** - Details all 5 bonus features for +200 hackathon points (personalization, Urdu translation, BetterAuth, RAG chatbot, ROS2 text-to-code).

- **Article XI: Deployment Requirements** - Specifies Vercel MCP for frontend/backend deployment, GitHub MCP integration, and Playwright MCP for testing.

**Enhanced Existing Articles:**
- Article II: Added ChatKit integration and Orchestration Framework tech stack items
- Article III: Enhanced with Playwright MCP for UI testing and Vercel MCP for deployment
- Article VII: Clarified agent artifact storage locations

**Updated CLAUDE.md:**
- Added cross-references to Constitution Article III (MCP Tool Mandate) and Article VIII (Orchestration Architecture)
- Noted 9-subagent architecture in execution flow section

## Outcome

- ✅ Impact: MAJOR constitutional revision establishing hackathon-optimized orchestration system with 9 subagents, 28 skills, and 4 MCP servers
- 🧪 Tests: Constitution formatting validated, no unresolved placeholders, version incremented correctly
- 📁 Files: `.specify/memory/constitution.md` (updated), `CLAUDE.md` (cross-referenced)
- 🔁 Next prompts: Create 9 subagent definitions (`.claude/agents/*.md`), create 28 skill definitions (`.claude/skills/*.md`), implement orchestration workflow
- 🧠 Reflection: Constitution now serves as comprehensive blueprint for hackathon delivery. Agent-skill mapping matrix provides clear responsibility assignment. Bonus features configuration ensures all +200 points requirements are documented.

## Evaluation notes (flywheel)

- Failure modes observed: None. Constitution update followed template requirements, incremented version semantically (MAJOR for backward-incompatible architectural changes).
- Graders run and results (PASS/FAIL): PASS - All placeholders resolved, YAML structure valid, cross-references accurate, sync impact report complete
- Prompt variant (if applicable): constitution-v5.0.0-hackathon-orchestration
- Next experiment (smallest change to try): Create first subagent definition (chapter-planner.md) as template for remaining 8 agents, then implement skill-creator for automated skill generation.
