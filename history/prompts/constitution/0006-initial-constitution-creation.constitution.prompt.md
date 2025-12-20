---
id: 0006
title: Initial Constitution Creation
stage: constitution
date: 2025-12-20
surface: claude-code-cli
model: claude-sonnet-4-5
feature: none
branch: main
user: rameez_badruddin
command: /sp.constitution
labels: [governance, hackathon, initial-setup]
links:
  spec: .specify/memory/constitution.md
  ticket: N/A
  adr: N/A
  pr: N/A
files:
  - path: .specify/memory/constitution.md
    action: created
    lines: 676
tests:
  - name: Constitution structure validation
    status: N/A
    notes: Manual validation against hackathon requirements
---

## Prompt

You are Claude Code operating inside a fresh repository.

Current state:
- Only a Docusaurus-based book exists.
- All previous implementations (chatbot, auth, personalization, translation) were intentionally removed.
- This is a CLEAN RESET.

Your task is to WRITE a COMPLETE, FINAL, HACKATHON-GRADE CONSTITUTION
for the project:

"Hackathon I: Create a Textbook for Teaching Physical AI & Humanoid Robotics"

This constitution is the SINGLE SOURCE OF TRUTH.
All future plans, tasks, agents, skills, and implementations MUST follow it.
Nothing is allowed outside this constitution.

Key requirements from hackathon brief:
- Docusaurus book with deployment
- RAG chatbot (book-only answers, OpenAI Agents/ChatKit, FastAPI, Neon, Qdrant)
- BetterAuth signup with software/hardware background questions
- Per-chapter personalization button
- Per-chapter Urdu translation button
- Deployment via Vercel or GitHub Pages
- Public GitHub repo
- 90-second demo video

Scoring structure: 100 base points + up to 200 bonus points (max 300 total)

Tech stack mandates: Docusaurus, React, FastAPI, OpenAI/Gemini APIs, Neon PostgreSQL, Qdrant Cloud, BetterAuth, Playwright, Vercel

Development philosophy: Spec-Driven Development (SDD) - Constitution → Plan → Tasks → Implement - no shortcuts, no experimental code

## Response snapshot

Created comprehensive constitution v1.0.0 with 8 core principles:

1. **Spec-Driven Development is Absolute**: No code without specs, strict hierarchy enforcement
2. **Technology Stack is Immutable**: Frozen tech stack matching hackathon requirements
3. **MCP-First Documentation Reference**: Mandatory MCP server queries before implementation
4. **RAG Chatbot Scope is Strictly Book-Only**: No general-purpose AI, citation requirements
5. **Authentication Gates Interactive Features**: Public reading, authenticated chatbot/personalization
6. **Personalization is Chapter-Scoped and On-Demand**: Single-button, session-based, user-background-driven
7. **Urdu Translation is Chapter-Scoped and On-Demand**: Single-button, preserves code blocks, RTL formatting
8. **Quality Gates are Non-Negotiable**: Build success, type safety, auth/authz, RAG functionality, deployment, documentation

Additional sections:
- Hackathon scoring alignment (300 points mapped to principles)
- Timeline constraints (deadline Nov 30, 2025)
- Folder structure constraints (from AGENTS.md)
- Execution flow (READ → VALIDATE → EXECUTE → VERIFY → STOP)
- Agent roles & responsibilities
- Testing philosophy
- PHR requirements
- Deployment philosophy
- Success criteria

## Outcome

- ✅ Impact: Complete governance framework created for hackathon project. All 300 possible points addressed.
- 🧪 Tests: N/A (constitutional document, validated against hackathon brief)
- 📁 Files: 1 file created (.specify/memory/constitution.md, 676 lines)
- 🔁 Next prompts: /sp.specify (create feature spec for RAG chatbot), /sp.plan (implementation planning)
- 🧠 Reflection: Constitution successfully integrates hackathon requirements, AGENTS.md governance, tech stack constraints, and MCP usage mandates. All principles are testable and traceable. Version 1.0.0 indicates production-ready governance.

## Evaluation notes (flywheel)

- Failure modes observed: None (initial creation)
- Graders run and results (PASS/FAIL): Manual validation PASS - all hackathon requirements mapped to constitutional principles
- Prompt variant (if applicable): N/A (initial version)
- Next experiment (smallest change to try): Test constitution enforcement during /sp.specify execution to ensure spec validation against principles
