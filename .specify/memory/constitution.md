<!--
========================================
SYNC IMPACT REPORT
========================================
Version Change: 6.0.0 → 6.1.0 (PATCH)

Rationale: Three targeted fixes for clarity, parser safety, and workflow enforcement.
- FIX #1: AGENTS.md reference formatting - Added backticks for parser safety
- FIX #2: RAG Integrity rule relocation - Moved from FastAPI Standards to Security Protocol (proper categorization)
- FIX #3: Spec modification prohibition - Added enforcement rule to prevent direct spec file edits

Modified Principles:
- Article I (Enforcement): ENHANCED - Added prohibition on direct spec file modification
- Article III (Agent Protocol): CLARIFIED - Fixed AGENTS.md reference formatting (backticks added)
- Article IV (Security Protocol): ENHANCED - Added RAG Integrity subsection with explicit rules

Removed Content:
- Misplaced RAG rule under FastAPI Standards (lines 187-188)

Added Content:
- Article I → Enforcement: Spec workflow enforcement rule
- Article IV → Security Protocol: RAG Integrity subsection

Templates Requiring Updates:
✅ No template changes required (clarifications only)

Follow-up TODOs:
- None (patch-level fixes only)

Ratified: 2025-12-03
Last Amended: 2025-12-12 (v6.1.0 - Patch: Formatting, categorization, and workflow fixes)
========================================
-->

# CONSTITUTION OF THE AI-NATIVE TEXTBOOK PROJECT

## PREAMBLE

This Constitution governs the creation of a complete **AI-Native Textbook on Physical AI & Humanoid Robotics** using Docusaurus, FastAPI, RAG (Retrieval-Augmented Generation), and Claude CLI automation with specialized subagents.

We commit to **Spec-Driven Development**: no code without specification, no implementation without plan, no deployment without validation.

---

## ARTICLE I: SPEC-DRIVEN DEVELOPMENT

### Core Rules

1. **No spec → No plan**
2. **No plan → No tasks**
3. **No tasks → No implementation**
4. **`.specify/specs` is source of truth**
5. **Code MUST follow specs exactly**
6. **Claude must update specs when new behavior is required**
7. **Plans and tasks must explicitly reference the corresponding spec section numbers for full traceability**

### Enforcement

- All implementation work requires completed `spec.md` and `plan.md` in `.specify/specs/[feature]/`
- Code that deviates from spec is incorrect by definition
- Spec changes require explicit approval and documentation
- Agents MUST NOT modify any specification file directly. All specification changes must be made through Spec-Kit Plus workflows (e.g., `/sp.specify update`). Direct manual edits are prohibited.

---

## ARTICLE II: AUTHORIZED TECHNOLOGY STACK

### Core Technologies

- **Frontend**: Docusaurus v3 (React/MDX/TypeScript)
- **Backend API**: FastAPI (Python 3.12+)
- **Vector Database**: Qdrant Cloud (Free Tier)
- **Primary Database**: Neon (Serverless PostgreSQL)
- **Authentication**: BetterAuth
- **AI Orchestration**: OpenAI Agents SDK + ChatKit UI
- **Automation**: Claude CLI (Spec-Kit Plus Orchestrated)

### Educational Content Stack

- **Robotics Framework**: ROS 2
- **Simulation**: Gazebo + NVIDIA Isaac + Unity
- **AI Integration**: OpenAI Whisper

### Constraints

- No technology substitutions without constitutional amendment
- All dependencies must be documented in specs
- Version pinning required for production dependencies

---

## ARTICLE III: AGENT PROTOCOL

### Mandatory Behaviors

1. **Agents MUST read entire repository before making changes**
   - Scan `.specify/specs`, `.specify/memory/constitution.md`, `AGENTS.md`, `CLAUDE.md`
   - Review existing implementation in `docs/`, `src/`, `apps/`

2. **Agents MUST respect folder structure**
   - `.specify/specs` - Feature specifications and plans
   - `.specify/memory` - Constitution and project memory
   - `history/prompts` - Prompt History Records
   - `history/adr` - Architectural Decision Records
   - `docs/` - Docusaurus content
   - `apps/` - Application code (frontend/backend)

3. **Agents MUST NOT delete or overwrite existing files without diff reasoning**
   - Provide clear rationale for any destructive changes
   - Preserve working functionality

4. **Agents MUST ask for clarification when requirements are ambiguous**
   - Use AskUserQuestion tool for unclear specs
   - Document assumptions in specs when clarifications received

5. **Agents MUST avoid hallucination**
   - Use MCP (Model Context Protocol) tools for external documentation
   - Verify all API syntax and library usage
   - Never invent APIs, endpoints, or configurations

6. **Agents MUST produce clean, consistent, deterministic output**
   - Follow established code patterns
   - Maintain consistent formatting
   - Ensure reproducible builds

Before performing any action, agents MUST load and follow `AGENTS.md` including:
- subagent orchestration rules
- skill definitions
- agent responsibilities

Agents MUST NOT create new top-level folders. Only the following root folders are allowed:
.specify, docs, apps, history, .claude

Agents SHOULD generate diffs for destructive changes when feasible.

MCP usage is mandatory only when external documentation or APIs are involved.
Agents MUST use Vercel MCP, GitHub MCP, or Playwright MCP when required and not invent any commands or endpoints.

---

## ARTICLE IV: ENGINEERING STANDARDS

### Language & Communication

- **Code and documentation**: Professional English
- **Conversation**: English or Roman Urdu (as preferred)
- **Comments**: English only

### Docusaurus Requirements

- **Format**: MDX (Markdown + JSX)
- **Structure**: Standard Docusaurus conventions
- **Components**: Use Docusaurus built-in components (Tabs, Admonitions, etc.)
- **Navigation**: Properly configured `sidebars.js`
- **Frontend (Docusaurus) and Backend (FastAPI)**: must remain isolated and deployable independently. No          cross-runtime imports or shared build steps

### FastAPI Standards

- **Project Layout**: Standard FastAPI structure
  - `app/main.py` - Application entry point
  - `app/api/` - API routes
  - `app/models/` - Data models
  - `app/services/` - Business logic
- **Type Hints**: Required for all function signatures
- **Documentation**: OpenAPI/Swagger auto-generation

### Security Protocol

- **No hardcoded secrets**: All sensitive keys in `.env` files
- **Environment Variables**: Use `.env.example` for templates
- **Git Security**: `.env` files MUST be in `.gitignore`
- **Code Review**: No API keys, tokens, or credentials in committed code
- **RAG Citations**: Chatbot must only cite book content sources
- **Input Sanitization**: All user inputs must be validated and sanitized

### RAG Integrity

- RAG chatbot MUST strictly cite book content only.
- External knowledge is forbidden unless explicitly defined in specs.

### Testing Requirements

- **Testing before deployment**: All features tested before merge
- **Test Coverage**: Critical paths must have automated tests
- **Manual Validation**: User-facing features validated manually

### Deployment Standards

- **Build Verification**: `npm run build` must succeed before deployment
- **Environment Separation**: Dev/staging/production environments clearly separated
- **Deployment Automation**: Use CI/CD for consistent deployments
- **Rollback Plan**: All deployments must be reversible

---

## ARTICLE V: BOOK PUBLISHING REQUIREMENTS

### Chapter Structure

Each chapter MUST follow this canonical structure:

1. **Title** - Clear, descriptive chapter name
2. **Introduction** - Chapter overview and learning objectives
3. **Concepts** - Theoretical foundations and explanations
4. **Code Examples** - Practical, executable code samples
5. **Exercises** - Hands-on practice activities
6. **Quiz** - Knowledge validation questions
7. **Summary** - Key takeaways and next steps

### Content Standards

- **MDX Format**: All content in Docusaurus-compatible MDX
- **Component Usage**: Use standard Docusaurus components
- **No Experimental Tech**: Avoid bleeding-edge or unstable dependencies
- **Proper Layout**: Content in `/docs/*` following Docusaurus conventions
- **Build Compatibility**: All content must build with `npm run build`

### Quality Requirements

- **Technical Accuracy**: All code examples must be verified
- **Clarity**: Explanations must be clear and accessible
- **Consistency**: Consistent terminology and style throughout
- **Diagrams**: Complex concepts visualized with Mermaid.js or images

---

## ARTICLE VI: BONUS FEATURES

The following features provide additional value beyond core textbook functionality:

### High-Level Features List

1. **Urdu Translation** - Chapter-level translation toggle
2. **Content Personalization** - Adaptive content based on user background
3. **BetterAuth Integration** - Signup/signin with user profiling
4. **ROS2 Code Examples** - Practical robotics code samples
5. **RAG Chatbot** - Embedded question-answering system

### Implementation Notes

- Feature specifications live in `.specify/specs/[feature]/`
- Implementation details defined in individual feature plans
- Agent/skill mappings documented in `AGENTS.md`
- Each feature must follow full spec-driven development cycle

---

## ARTICLE VII: GOVERNANCE & CHANGE RULES

### Decision Documentation

- **Major Decisions**: All architectural decisions documented in ADRs (`history/adr/`)
- **Spec Updates**: All requirement changes reflected in specs
- **Constitutional Amendments**: Constitution changes require explicit approval

### Change Management

- **Constitutional Compliance**: All updates must align with Constitution
- **Backward Compatibility**: Claude must not break existing working features
- **Change Authorization**: Claude must not modify Constitution without explicit request

### Security Governance

- **API Key Protection**: Never commit API keys to GitHub
- **Code Review**: Security-sensitive changes require careful review
- **Best Practices**: Follow OWASP security guidelines
- **Vulnerability Remediation**: Security issues addressed promptly

### Version Control

- **Constitution Versioning**: Semantic versioning (MAJOR.MINOR.PATCH)
- **Change Tracking**: All changes logged in Sync Impact Report
- **Amendment History**: Ratification and amendment dates maintained

---

## GOVERNANCE

### Authority

This Constitution supersedes all other project instructions except where explicitly deferred to `AGENTS.md` for agent/skill implementation details.

### Amendments

Any deviation requires:
1. Documented amendment with rationale
2. Impact analysis across templates and workflows
3. Updated version number following semantic versioning
4. User approval for constitutional changes

### Compliance

All future specifications, plans, and tasks must explicitly declare compliance with this Constitution.

---

**Version**: 6.1.0
**Ratified**: 2025-12-03
**Last Amended**: 2025-12-12 (Patch Amendment: Formatting, categorization, and workflow fixes)

---

**System Instruction:**
This Constitution is effective immediately. All agents must read this file and `AGENTS.md` before executing any command. Agents are responsible for ensuring constitutional compliance in all work.
