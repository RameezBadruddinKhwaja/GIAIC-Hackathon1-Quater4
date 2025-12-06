---
id: 0001
title: AI-Native Institute Constitution v2.0.0
stage: constitution
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: system
command: /sp.constitution
labels: ["governance", "hackathon", "tech-stack", "language-protocol"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
  - .specify/templates/plan-template.md
tests:
  - N/A (governance document)
---

## Prompt

Act as the **Chief Governance Officer** of the AI-Native Institute.

Your task is to draft the **FINAL, REVISED CONSTITUTION** (`.specify/memory/constitution.md`). This document governs all Agents working on the "Physical AI & Humanoid Robotics" Hackathon project.

**Context:**
- **Methodology:** Panaversity Spec-Driven Development (SDD) + Model Context Protocol (MCP).
- **Mandatory Stack:** Docusaurus (Docs), FastAPI (Backend), Qdrant Cloud (Vector DB), Neon (Serverless Postgres), OpenAI Agents SDK.
- **Language:** **Roman Urdu** for chat, **English** for work.

**Action:**
Overwrite `.specify/memory/constitution.md` with the following content exactly.

---

# CONSTITUTION OF THE AI-NATIVE INSTITUTE

## PREAMBLE
We, the Agents of this System, commit to **Spec-Driven Engineering**. We reject guesswork. We adhere to the approved technology stack and secure coding practices to ensure a 100/100 Hackathon score.

## ARTICLE I: SPEC-DRIVEN DEVELOPMENT (SDD)
1.  **No Spec, No Code:** Implementation (`/sp.implement`) is forbidden without a strictly defined `spec.md` and `plan.md`.
2.  **Single Source of Truth:** The `specs/` folder governs the `src/` folder. If code deviates from the spec, the code must be discarded.
3.  **Architectural Decisions:** All major technical choices must be recorded in an **ADR** (`history/adr/`).

## ARTICLE II: THE AUTHORIZED TECH STACK (Immutable)
*Agents are forbidden from using alternatives to the following:*
1.  **Documentation:** Docusaurus (React/MDX).
2.  **RAG Database:** **Qdrant Cloud** (Vector Store).
3.  **Primary Database:** **Neon** (Serverless PostgreSQL).
4.  **Backend API:** FastAPI (Python).
5.  **Integration Standard:** **Model Context Protocol (MCP)** for all tool connections.

## ARTICLE III: AGENT BEHAVIOR & PROTOCOLS
1.  **Language Protocol:**
    - **Chat/Conversation:** MUST be in **Roman Urdu** (Clear, professional tone).
    - **Code/Docs:** MUST be in **Professional US English**.
2.  **Anti-Hallucination:**
    - Never invent libraries or commands.
    - If a task requires external data, use **MCP tools** (not guessing).
3.  **Role Fidelity:** Stay in character. The @Author does not write SQL queries.

## ARTICLE IV: THE PUBLISHING STANDARDS (Book Wing)
1.  **Docusaurus First:** Content must be structured for Docusaurus (Use `sidebars.js`, `:::tip`, `<Tabs>`).
2.  **Technical Accuracy:** All Physical AI concepts must align with ROS 2 and NVIDIA Isaac standards.
3.  **Visuals:** Complex topics must use **Mermaid.js** diagrams.

## ARTICLE V: THE ENGINEERING STANDARDS (Lab Wing)
1.  **Security Absolute (SOC Protocol):**
    - **Zero Hardcoding:** API Keys must use `.env`.
    - **Sanitization:** All user inputs to the Chatbot must be sanitized to prevent Injection attacks.
2.  **Test-Driven Development (TDD):** Write the test -> Run and Fail -> Write Code -> Pass.
3.  **RAG Integrity:** The Chatbot must cite sources from the book chapters.

## ARTICLE VI: THE MATRIX PROTOCOL (Skill & MCP)
1.  **Dynamic Skill Loading:** Agents must explicitly load relevant skills from `.claude/skills/` to conserve context.
2.  **MCP Usage:** When interacting with Qdrant or Neon, Agents must utilize the configured **MCP Servers** or approved SDKs, rather than raw HTTP requests where possible.

## ARTICLE VII: INTELLIGENCE PRESERVATION
1.  **Prompt History (PHR):** Log successful prompt chains in `history/prompts/` for future learning.
2.  **Folder Integrity:** Do not create new root directories. Work within `src/`, `docs/`, `specs/`.

---

**System Instruction:**
This Constitution is effective immediately. All Agents must read this file before executing any command.

## Response snapshot

**Constitution Successfully Updated to v2.0.0**

Changes implemented:
1. **Version bump**: 1.0.0 → 2.0.0 (MAJOR)
   - Rationale: Complete governance restructure from generic principles to hackathon-specific Articles

2. **New Structure**: Replaced 7 generic principles with 7 Articles:
   - **Article I**: Spec-Driven Development (SDD) - No code without spec/plan
   - **Article II**: Authorized Tech Stack (Immutable) - Docusaurus, Qdrant, Neon, FastAPI, MCP
   - **Article III**: Agent Behavior & Protocols - Language protocol (Roman Urdu chat, English code)
   - **Article IV**: Publishing Standards (Book Wing) - Docusaurus conventions, ROS 2/NVIDIA Isaac
   - **Article V**: Engineering Standards (Lab Wing) - Security, TDD, RAG integrity
   - **Article VI**: Matrix Protocol - Dynamic skill loading, MCP usage
   - **Article VII**: Intelligence Preservation - PHR logging, folder integrity

3. **Template Updates**:
   - ✅ Updated `plan-template.md` Constitution Check section with new Article-based gates
   - ✅ Spec and tasks templates remain compatible
   - ⚠️ CLAUDE.md may need language protocol alignment

4. **Sync Impact Report**: Embedded as HTML comment at top of constitution file

## Outcome

- ✅ Impact: Complete governance restructure for AI-Native Institute Hackathon; immutable tech stack defined; language protocol established
- 🧪 Tests: N/A (governance document)
- 📁 Files: 2 files modified (constitution.md, plan-template.md)
- 🔁 Next prompts: Review CLAUDE.md for language protocol alignment; create first feature spec under new constitution
- 🧠 Reflection: Major version bump warranted; new Articles provide clear boundaries for hackathon compliance

## Evaluation notes (flywheel)

- Failure modes observed: None - clean execution with template consistency validation
- Graders run and results (PASS/FAIL): PASS - All placeholders filled, no bracket tokens remaining, version incremented correctly
- Prompt variant (if applicable): N/A - Initial constitution establishment for hackathon
- Next experiment (smallest change to try): Test constitution enforcement in first `/sp.specify` or `/sp.plan` execution
