<!--
========================================
SYNC IMPACT REPORT
========================================
Version Change: 2.0.0 → 4.0.0 (MAJOR) → 4.1.0 (MINOR)

Rationale: Complete governance restructure with explicit MCP Server mandates.
- Expanded Article III with explicit MCP tool mandates (Context7, GitHub MCP, Playwright)
- Added Article IV, Article V, and Article VI for comprehensive operational coverage
- Added Better-Auth to authorized tech stack (bonus requirement)
- Replaced generic "Agent Behavior & Protocols" with detailed behavior rules
- Replaced "Matrix Protocol" with "MCP Tool Mandate" for clarity and enforcement
- Consolidated publishing and engineering standards into clear, actionable articles

**Amendment 4.1.0 (2025-12-05):**
- Article II, Item 6: Clarified AI Orchestration to use OpenAI Agents SDK configured with Google Gemini 2.5 Flash
- Rationale: Project will use OpenAI SDK as client library but route requests to Google Gemini 2.5 Flash models via `base_url` configuration
- Implementation Note: Use model string `gemini-2.5-flash` in OpenAI SDK client initialization (NOT `gemini-1.5-flash`)
- CRITICAL: Do NOT replace OpenAI SDK with Google's SDK. Use OpenAI SDK with custom base_url pointing to Gemini API endpoint

Modified Principles:
- Article I: No changes in SDD fundamentals
- Article II: Added "Better-Auth" (authentication requirement); AMENDED Item 6 to specify Gemini model configuration
- Article III: COMPLETELY REWRITTEN - Changed from "Agent Behavior & Protocols" to "MCP Tool Mandate (The Matrix Tools)" with explicit tool usage rules
- Article IV: EXPANDED - Changed from "Publishing Standards" to comprehensive agent behavior protocols
- Article V: EXPANDED - Enhanced from basic publishing rules to detailed Docusaurus-native standards
- Article VI: EXPANDED - Enhanced security and engineering rules (SOC Protocol, TDD, RAG Integrity)
- Article VII: RENAMED - Changed from "Matrix Protocol" to "Intelligence Preservation" for clarity

Added Sections:
- Article III: MCP Tool Mandate (Context7, GitHub MCP, Playwright) - MANDATORY usage rules
- Detailed usage rules for each MCP server with specific responsibilities
- Anti-hallucination protocols tied to MCP tool usage

Removed Sections:
- Generic "Matrix Protocol" skill loading instructions (replaced with specific MCP mandates)
- Vague "Dynamic Skill Loading" language (superseded by MCP tool specifications)

Templates Requiring Updates:
✅ plan-template.md - Constitution Check section already updated (lines 30-42)
✅ spec-template.md - No direct references, compatible as-is
✅ tasks-template.md - No direct references, compatible as-is
⚠ CLAUDE.md - Should reference new Article III MCP mandates explicitly
✅ specs/001-ai-textbook-platform/research.md - UPDATED Section 7 (AI Orchestration) with Gemini 2.5 Flash configuration details
✅ specs/001-ai-textbook-platform/plan.md - UPDATED Technical Context with `gemini-2.5-flash` model specification

Follow-up TODOs:
- ⚠ Update CLAUDE.md to explicitly reference Article III MCP Tool Mandate
- ✅ Update research.md Section 7 (AI Orchestration Decision) to document OpenAI SDK + Gemini configuration
- ✅ Update plan.md Technical Context to specify `gemini-2.5-flash` as model
- Verify all agents are aware of Context7, GitHub MCP, and Playwright availability
- Create ADR for MCP Server selection rationale if architectural discussions arise

Ratified: 2025-12-03
Last Amended: 2025-12-05 (v4.1.0 - Gemini Model Configuration Amendment)
========================================
-->

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
6.  **AI Orchestration:** **OpenAI Agents SDK** (Client configured with `base_url` for Google Gemini 2.5 Flash).

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

---

## GOVERNANCE

This constitution supersedes all other project instructions.
Any deviation requires a documented amendment with rationale and impact analysis.
All future specifications must explicitly declare compliance with this constitution.

**Version**: 4.1.0
**Ratified**: 2025-12-03
**Last Amended**: 2025-12-05 (Amendment: Gemini Model Configuration)

---

**System Instruction:**
This Constitution is effective immediately. All Agents must read this file before executing any command.
