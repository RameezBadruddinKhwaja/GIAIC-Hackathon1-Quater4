---
id: 0002
title: MCP-Enabled Constitution Update
stage: constitution
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 001-ai-textbook-platform
user: system
command: /sp.constitution
labels: ["governance", "mcp", "constitution", "version-4.0.0"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - None (governance document update)
---

## Prompt

Act as the **Chief Governance Officer** of the AI-Native Institute.

**Action:** Overwrite `.specify/memory/constitution.md` with the following **FINAL UPDATED CONSTITUTION**. This version explicitly mandates the use of installed MCP Servers (Context7, Playwright, GitHub).

---

# CONSTITUTION OF THE AI-NATIVE INSTITUTE

## PREAMBLE
We, the Agents of this System, commit to **Spec-Driven Engineering**. We reject guesswork. We harness the power of **Model Context Protocol (MCP)** to interact with the world, the codebase, and the knowledge bases.

## ARTICLE I: SPEC-DRIVEN DEVELOPMENT (SDD)
1.  **No Spec, No Code:** Implementation (`/sp.implement`) is forbidden without a strictly defined `spec.md` and `plan.md` in the `specs/` directory.
2.  **Single Source of Truth:** The `specs/` folder governs the `src/` and `docs/` folders. If code deviates from the spec, the code is incorrect.
3.  **Architectural Decisions:** All major technical choices must be recorded in an **ADR** (`history/adr/`).

## ARTICLE II: THE AUTHORIZED TECH STACK (Immutable)
1.  **Documentation Engine:** Docusaurus v3 (React/MDX/TypeScript).
2.  **RAG Database:** **Qdrant Cloud** (Vector Store) Free Tier.
3.  **Primary Database:** **Neon** (Serverless PostgreSQL).
4.  **Backend API:** FastAPI (Python).
5.  **Authentication:** **Better-Auth** (Bonus Requirement).
6.  **AI Orchestration:** OpenAI Agents SDK / ChatKit.

## ARTICLE III: MCP TOOL MANDATE (The "Matrix" Tools)
*Agents MUST use the following MCP Servers for their respective domains:*

1.  **Context7 (The Librarian):**
    - **Usage:** MUST be used to fetch the latest **Docusaurus v3**, **FastAPI**, and **Neon** documentation.
    - **Rule:** Never hallucinate configuration options. Fetch the real docs via Context7 first.
2.  **GitHub MCP (The Operator):**
    - **Usage:** MUST be used for creating repositories, managing Pull Requests, reading issues, and triggering GitHub Pages deployment workflows.
    - **Rule:** Do not rely on local git CLI alone for remote operations.
3.  **Playwright (The Navigator):**
    - **Usage:** Use for verifying deployed pages, taking screenshots for docs, or gathering web data if needed.

## ARTICLE IV: AGENT BEHAVIOR & PROTOCOLS
1.  **Language Protocol:**
    - **Chat/Conversation:** MUST be in **Roman Urdu** (Clear, professional tone).
    - **Code/Docs:** MUST be in **Professional US English**.
2.  **Anti-Hallucination:**
    - If a task involves external knowledge, **Load the Context7 Tool** immediately.
3.  **Role Fidelity:** Stay in character. The @Author writes content; the @Coder implements functionality.

## ARTICLE V: THE PUBLISHING STANDARDS (Book Wing)
1.  **Docusaurus Native:** Content must be structured for Docusaurus (`sidebars.js`, `:::tip`, `<Tabs>`).
2.  **Visuals:** Complex topics must use **Mermaid.js** diagrams.
3.  **Reference Integrity:** Use **Context7** to verify any external links or technical claims about ROS 2 / NVIDIA Isaac.

## ARTICLE VI: THE ENGINEERING STANDARDS (Lab Wing)
1.  **Security Absolute (SOC Protocol):**
    - **Zero Hardcoding:** API Keys must use `.env`.
    - **Sanitization:** All user inputs to the Chatbot must be sanitized.
2.  **RAG Integrity:** The Chatbot must cite sources from the book chapters.
3.  **Deployment:** Deployment to **GitHub Pages** must be automated via GitHub Actions (setup using GitHub MCP).

## ARTICLE VII: INTELLIGENCE PRESERVATION
1.  **Prompt History (PHR):** Log successful prompt chains in `history/prompts/` via `/sp.phr`.
2.  **Folder Integrity:** Do not create new root directories. Work within `src/`, `docs/`, `specs/`.

**Version:** 4.0.0 (MCP Enabled Edition)
**Ratified:** [Current Date]

## Response snapshot

Constitution successfully updated to version 4.0.0 with explicit MCP Server mandates:

**Version Bump Rationale (2.0.0 → 4.0.0 MAJOR):**
- Complete restructuring of Article III from generic "Agent Behavior & Protocols" to explicit "MCP Tool Mandate"
- Added mandatory usage rules for three MCP servers: Context7, GitHub MCP, and Playwright
- Added Better-Auth to authorized tech stack (Article II)
- Reorganized articles for clarity: MCP tools first, then behavior, then domain-specific standards
- Breaking change: Agents must now use MCP tools for their domains (backward incompatible governance)

**Key Changes:**
1. **Article III (NEW)**: MCP Tool Mandate with three explicit tools and their usage rules
2. **Article IV**: Retained agent behavior protocols (language, anti-hallucination, role fidelity)
3. **Article V**: Enhanced publishing standards (Docusaurus-native, Mermaid.js, Context7 verification)
4. **Article VI**: Enhanced engineering standards (SOC Protocol, RAG integrity, GitHub Actions deployment)
5. **Article VII**: Intelligence preservation (PHR logging, folder integrity)

**Templates Status:**
✅ plan-template.md - Already aligned (Constitution Check section references all articles)
✅ spec-template.md - Compatible (no direct references)
✅ tasks-template.md - Compatible (no direct references)
⚠ CLAUDE.md - Requires update to reference Article III MCP mandates

**Sync Impact Report** prepended to constitution file with complete change tracking.

## Outcome

- ✅ Impact: Constitution upgraded to v4.0.0 with explicit MCP Server usage mandates; all agents now required to use Context7, GitHub MCP, and Playwright for their respective domains
- 🧪 Tests: None required (governance document)
- 📁 Files: 1 file modified (.specify/memory/constitution.md)
- 🔁 Next prompts: Consider updating CLAUDE.md to explicitly reference Article III; Create ADR for MCP Server selection rationale if needed
- 🧠 Reflection: Major version bump justified by backward-incompatible governance changes requiring MCP tool usage; constitution now enforces anti-hallucination through mandatory documentation fetching

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - All placeholders replaced, no unresolved tokens, version bump justified, sync impact report complete
- Prompt variant (if applicable): constitution-update-v2 (with explicit MCP server mandates)
- Next experiment (smallest change to try): Verify agent compliance with Article III by monitoring Context7 usage for Docusaurus/FastAPI/Neon queries
