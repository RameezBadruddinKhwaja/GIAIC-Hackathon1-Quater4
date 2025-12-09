<!--
========================================
SYNC IMPACT REPORT
========================================
Version Change: 4.1.0 → 5.0.0 (MAJOR)

Rationale: Complete transformation to hackathon-driven orchestration architecture with Claude Subagent system.
- Added Article VIII: AGENTIC ORCHESTRATION ARCHITECTURE with 9 specialized subagents
- Added Article IX: SKILL SYSTEM with 23 required skills and bonus capabilities
- Expanded Article II with comprehensive skill-agent mapping
- Enhanced Article III with Playwright MCP usage for UI testing
- Added detailed bonus features configuration (personalization, Urdu translation, ROS2 text-to-code)
- Restructured from generic constitution to hackathon-optimized delivery system

Modified Principles:
- Article I: No changes in SDD fundamentals (preserved)
- Article II: EXPANDED - Added 23+ required skills and agent-skill mapping matrix
- Article III: ENHANCED - Added Playwright MCP for UI testing and web verification
- Article IV: No changes in agent behavior protocols (preserved)
- Article V: No changes in publishing standards (preserved)
- Article VI: No changes in engineering standards (preserved)
- Article VII: No changes in intelligence preservation (preserved)
- Article VIII: NEW - Complete agentic orchestration architecture with 9 subagents
- Article IX: NEW - Skill system with canonical skillsets and bonus features

Added Sections:
- Article VIII (AGENTIC ORCHESTRATION ARCHITECTURE): 9 specialized subagents with clear responsibilities
  - chapter-planner, spec-architect, pedagogical-designer, educational-validator
  - content-implementor, factual-verifier, assessment-architect, validation-auditor, super-orchestrator
- Article IX (SKILL SYSTEM): 23 required skills + 5 bonus skills
  - Canonical skills: book-scaffolding, chapter-planner, quiz-generator, etc.
  - Bonus skills: urdu-translator, user-profile-initializer, ros2-code-generator
  - Agent-skill mapping matrix
- Bonus Features Configuration: Personalization, Urdu, BetterAuth, RAG Chatbot, ROS2 simulator
- Deployment Requirements: Vercel MCP for frontend/backend, GitHub MCP for version control

Removed Sections:
- None (all existing articles preserved)

Templates Requiring Updates:
⚠ plan-template.md - Should reference Article VIII agent orchestration requirements
⚠ tasks-template.md - Should incorporate agent-task assignments from Article VIII
✅ spec-template.md - No direct references, compatible as-is
✅ CLAUDE.md - Already references MCP mandates, compatible but could benefit from Article VIII/IX references

Follow-up TODOs:
- Create 9 subagent files under `.claude/agents/` (chapter-planner.md, spec-architect.md, etc.)
- Create 23+ skill files under `.claude/skills/` per Article IX requirements
- Update CLAUDE.md to reference Article VIII orchestration and Article IX skill system
- Create agent-skill connection configuration (cross-reference matrix)
- Verify Playwright MCP integration for UI testing workflows
- Document bonus features activation process (chapter toggles, Urdu translation, personalization)

Ratified: 2025-12-03
Last Amended: 2025-12-09 (v5.0.0 - Hackathon Orchestration Architecture)
========================================
-->

# CONSTITUTION OF THE AI-NATIVE INSTITUTE

## PREAMBLE

We, the Agents of this System, commit to **Spec-Driven Engineering**. We reject guesswork. We harness the power of **Model Context Protocol (MCP)** to interact with the world, the codebase, and the knowledge bases.

This constitution governs the development of the **Physical AI & Humanoid Robotics Textbook Platform** for the Panaversity Hackathon, orchestrating a multi-agent system to deliver a complete educational platform with RAG chatbot, personalization, and localization capabilities.

## ARTICLE I: SPEC-DRIVEN DEVELOPMENT (SDD)

1.  **No Spec, No Code:** Implementation (`/sp.implement`) is forbidden without a strictly defined `spec.md` and `plan.md` in the `specs/` directory.
2.  **Single Source of Truth:** The `specs/` folder governs the `src/` and `docs/` folders. If code deviates from the spec, the code is incorrect.
3.  **Architectural Decisions:** All major technical choices must be recorded in an **ADR** (`history/adr/`).

## ARTICLE II: THE AUTHORIZED TECH STACK (Immutable)

1.  **Documentation Engine:** Docusaurus v3 (React/MDX/TypeScript).
2.  **RAG Database:** **Qdrant Cloud** (Vector Store) Free Tier.
3.  **Primary Database:** **Neon** (Serverless PostgreSQL).
4.  **Backend API:** FastAPI (Python 3.12+).
5.  **Authentication:** **Better-Auth** (Bonus Requirement).
6.  **AI Orchestration:** **OpenAI Agents SDK** (Client configured with `base_url` for Google Gemini 2.5 Flash).
7.  **ChatKit Integration:** OpenAI ChatKit for RAG chatbot UI with contextual text selection.
8.  **Orchestration Framework:** Claude Subagents + Skills (23+ required skills, 5+ bonus skills).

## ARTICLE III: MCP TOOL MANDATE (The "Matrix" Tools)

*Agents MUST use the following MCP Servers for their respective domains:*

1.  **Context7 (The Librarian):**
    - **Usage:** MUST be used to fetch the latest **Docusaurus v3**, **FastAPI**, **Neon**, **ROS 2**, **NVIDIA Isaac**, **Gazebo**, **Unity**, and **Whisper** documentation.
    - **Rule:** Never hallucinate configuration options. Fetch the real docs via Context7 first.
2.  **GitHub MCP (The Operator):**
    - **Usage:** MUST be used for creating repositories, managing Pull Requests, reading issues, and triggering GitHub Pages deployment workflows.
    - **Rule:** Do not rely on local git CLI alone for remote operations.
3.  **Playwright (The Navigator):**
    - **Usage:** MUST be used for verifying deployed pages, taking screenshots for documentation, UI testing of chatbot interface, and gathering web data for validation.
    - **Rule:** All deployment verification and UI testing must use Playwright MCP to ensure consistency.
4.  **Vercel MCP (The Deployer):**
    - **Usage:** MUST be used for deploying frontend (Docusaurus) and backend (FastAPI RAG API).
    - **Rule:** If mono-repo deployment not supported, deploy as separate services with proper environment variable configuration.

## ARTICLE IV: AGENT BEHAVIOR & PROTOCOLS

1.  **Language Protocol:**
    - **Chat/Conversation:** MUST be in **Roman Urdu** (Clear, professional tone).
    - **Code/Docs:** MUST be in **Professional US English**.
2.  **Anti-Hallucination:**
    - If a task involves external knowledge, **Load the Context7 Tool** immediately.
3.  **Role Fidelity:** Stay in character. Each subagent has explicit responsibilities (see Article VIII).
4.  **Human as Tool:** Invoke user for clarification on ambiguous requirements, unforeseen dependencies, architectural uncertainty, or completion checkpoints.

## ARTICLE V: THE PUBLISHING STANDARDS (Book Wing)

1.  **Docusaurus Native:** Content must be structured for Docusaurus (`sidebars.js`, `:::tip`, `<Tabs>`).
2.  **Visuals:** Complex topics must use **Mermaid.js** diagrams.
3.  **Reference Integrity:** Use **Context7** to verify any external links or technical claims about ROS 2, NVIDIA Isaac, Gazebo, Unity, Whisper.
4.  **Book Structure:** 13-week curriculum divided into 4 modules with learning objectives, code examples, exercises, and assessments.
5.  **Canonical Style:** Follow canonical markdown format with intelligent reusable blocks.

## ARTICLE VI: THE ENGINEERING STANDARDS (Lab Wing)

1.  **Security Absolute (SOC Protocol):**
    - **Zero Hardcoding:** API Keys must use `.env`.
    - **Sanitization:** All user inputs to the Chatbot must be sanitized.
2.  **RAG Integrity:** The Chatbot must cite sources from the book chapters using OpenAI Agents + Qdrant + ChatKit.
3.  **Deployment:** Deployment to **Vercel** must be automated via Vercel MCP for both frontend and backend.
4.  **Testing:** Use Playwright MCP for UI testing and validation of deployed interfaces.

## ARTICLE VII: INTELLIGENCE PRESERVATION

1.  **Prompt History (PHR):** Log successful prompt chains in `history/prompts/` via `/sp.phr`.
2.  **Folder Integrity:** Do not create new root directories. Work within `src/`, `docs/`, `specs/`, `.claude/`.
3.  **Agent Artifacts:** All subagent definitions stored in `.claude/agents/`, all skill definitions in `.claude/skills/`.

## ARTICLE VIII: AGENTIC ORCHESTRATION ARCHITECTURE

The system employs **9 specialized Claude Subagents**, each with distinct responsibilities:

### 1. chapter-planner
- **Responsibility:** Divide the textbook into 13 parts based on course weeks/modules from syllabus.
- **Skills:** `chapter-planner`, `book-scaffolding`, `learning-objectives`
- **Output:** Chapter breakdown with learning objectives, topics, and week assignments.

### 2. spec-architect
- **Responsibility:** Design initial book spec from provided syllabus and hackathon goals.
- **Skills:** `mvp-builder`, `tool-selection-framework`, `technical-clarity`
- **Output:** Complete `spec.md` and `data-model.md` for the feature.

### 3. pedagogical-designer
- **Responsibility:** Apply learning objectives and outcome-driven design to each chapter.
- **Skills:** `learning-objectives`, `exercise-designer`, `assessment-builder`
- **Output:** Chapter-level learning outcomes, exercises, and assessment criteria.

### 4. educational-validator
- **Responsibility:** Ensure content matches learning outcomes and core robotics topics.
- **Skills:** `validation-auditor`, `technical-clarity`, `canonical-format-checker`
- **Output:** Validation reports confirming alignment with learning objectives.

### 5. content-implementor
- **Responsibility:** Write rich, agent-friendly markdown using canonical styles.
- **Skills:** `book-scaffolding`, `concept-scaffolding`, `code-example-generator`, `image-generator`
- **Output:** Fully-formatted Docusaurus MDX files with diagrams, code tabs, and exercises.

### 6. factual-verifier
- **Responsibility:** Validate every chapter for accuracy, especially for ROS 2, Isaac, Gazebo, Unity, Whisper.
- **Skills:** `technical-clarity`, `canonical-format-checker`, Context7 MCP integration
- **Output:** Verification reports with citations from authoritative documentation.

### 7. assessment-architect
- **Responsibility:** Add quizzes, assignments, and challenges per module.
- **Skills:** `quiz-generator`, `assessment-builder`, `exercise-designer`
- **Output:** Interactive quizzes, coding challenges, and module assessments.

### 8. validation-auditor
- **Responsibility:** Run style/format validation and correctness checks across book.
- **Skills:** `validation-auditor`, `canonical-format-checker`, Playwright MCP integration
- **Output:** Comprehensive audit reports with actionable fixes.

### 9. super-orchestrator
- **Responsibility:** Connect all agents to build, write, validate, and finalize content.
- **Skills:** All skills, MCP orchestration, deployment coordination
- **Output:** Complete textbook platform with RAG chatbot, authentication, and personalization deployed to production.

## ARTICLE IX: SKILL SYSTEM

### Required Canonical Skills (23+)

1.  **book-scaffolding** - Generate Docusaurus project structure with sidebars, frontmatter
2.  **chapter-planner** - Divide curriculum into modules, weeks, and learning paths
3.  **concept-scaffolding** - Create conceptual frameworks for technical topics
4.  **summary-generator** - Generate chapter summaries and TL;DR sections
5.  **quiz-generator** - Create interactive quizzes with multiple-choice and code challenges
6.  **technical-clarity** - Ensure technical accuracy and clear explanations
7.  **canonical-format-checker** - Validate markdown format against canonical style
8.  **assessment-builder** - Design comprehensive assessments with rubrics
9.  **mvp-builder** - Identify and implement minimum viable product features
10. **learning-objectives** - Define clear, measurable learning outcomes
11. **docusaurus-deployer** - Deploy Docusaurus sites to Vercel via MCP
12. **prompt-template-designer** - Create reusable prompt templates for agents
13. **code-example-generator** - Generate accurate ROS 2/Python/C++ code examples
14. **exercise-designer** - Create hands-on exercises and coding challenges
15. **frontend-design** - Design React components and UI/UX patterns
16. **validation-auditor** - Comprehensive validation of content, format, and functionality
17. **skill-creator** - Meta-skill for creating new skills as needed
18. **playwright-test-runner** - Run UI tests via Playwright MCP
19. **image-generator** - Generate concept diagrams and visual aids
20. **ux-evaluator** - Evaluate user experience and interface usability
21. **tool-selection-framework** - Select appropriate tools/frameworks for requirements
22. **notebooklm-slides** - Generate presentation slides from content (optional)
23. **session-intelligence-harvester** - Collect and apply user context for personalization

### Bonus Skills (5+)

1.  **urdu-translator** - Translate content to Urdu (Roman/Formal) with code preservation
2.  **user-profile-initializer** - Handle BetterAuth signup/signin with tech background collection
3.  **ros2-code-generator** - Text-to-code for ROS2 commands (e.g., "Pick up object" → Python ROS2 code)
4.  **rag-chatbot-integrator** - Integrate OpenAI Agents + Qdrant + ChatKit for contextual RAG
5.  **personalization-engine** - Hardware-aware content adaptation via chapter-level toggles

### Agent-Skill Mapping Matrix

| Agent | Primary Skills | Secondary Skills |
|-------|---------------|------------------|
| chapter-planner | chapter-planner, book-scaffolding | learning-objectives |
| spec-architect | mvp-builder, tool-selection-framework | technical-clarity |
| pedagogical-designer | learning-objectives, exercise-designer | assessment-builder |
| educational-validator | validation-auditor, technical-clarity | canonical-format-checker |
| content-implementor | book-scaffolding, code-example-generator | image-generator, concept-scaffolding |
| factual-verifier | technical-clarity, canonical-format-checker | Context7 MCP |
| assessment-architect | quiz-generator, assessment-builder | exercise-designer |
| validation-auditor | validation-auditor, playwright-test-runner | canonical-format-checker |
| super-orchestrator | ALL SKILLS | MCP orchestration |

## ARTICLE X: BONUS FEATURES CONFIGURATION

To earn the full **+200 bonus points**, the following features MUST be implemented:

### 1. Chapter-Level Personalization (50 points)
- Use `session-intelligence-harvester` skill
- Collect user hardware context (RTX 4090 vs Jetson Orin Nano)
- Adapt code examples and performance recommendations per chapter
- Toggle via chapter frontmatter and user profile

### 2. Urdu Translation (50 points)
- Use `urdu-translator` skill
- Implement chapter-level toggle for Roman Urdu / Formal Urdu
- Preserve code blocks, technical terms, and API references in English
- Store translations in parallel MDX files

### 3. BetterAuth Integration (50 points)
- Use `user-profile-initializer` skill
- Implement Email + GitHub OAuth signup/signin
- Collect user tech background (beginner/intermediate/advanced)
- Store user preferences in Neon PostgreSQL

### 4. RAG Chatbot with Contextual Selection (50 points)
- Use `rag-chatbot-integrator` skill
- Integrate OpenAI Agents SDK + Qdrant + ChatKit
- Allow users to select text and ask questions with context
- Cite chapter sources in all responses

### 5. ROS2 Text-to-Code Simulator (Bonus)
- Use `ros2-code-generator` skill
- Natural language input: "Pick up the red cube"
- Generated output: Working Python ROS2 code snippet
- Display in interactive code playground

## ARTICLE XI: DEPLOYMENT REQUIREMENTS

### Frontend Deployment (Docusaurus)
- Use **Vercel MCP** to deploy frontend to Vercel
- Configure build command: `cd apps/docs && npm run build`
- Set output directory: `apps/docs/build`
- Enable automatic deployments on push to `main`

### Backend Deployment (FastAPI RAG API)
- Use **Vercel MCP** to deploy backend to Vercel
- If mono-repo not supported, deploy as separate service
- Configure Python runtime and FastAPI entry point
- Set environment variables via Vercel MCP (Neon, Qdrant, OpenAI API keys)

### GitHub Integration
- Use **GitHub MCP** to commit and push all content to GitHub
- Enable GitHub Pages for additional hosting option
- Configure automated deployment workflows

### Testing & Validation
- Use **Playwright MCP** for UI testing of deployed interfaces
- Verify chatbot functionality, authentication flows, and content rendering
- Generate screenshots for documentation

---

## GOVERNANCE

This constitution supersedes all other project instructions.
Any deviation requires a documented amendment with rationale and impact analysis.
All future specifications must explicitly declare compliance with this constitution.

**Version**: 5.0.0
**Ratified**: 2025-12-03
**Last Amended**: 2025-12-09 (Major Amendment: Hackathon Orchestration Architecture)

---

**System Instruction:**
This Constitution is effective immediately. All Agents must read this file before executing any command.
The super-orchestrator agent is responsible for coordinating all subagents and ensuring constitutional compliance.
