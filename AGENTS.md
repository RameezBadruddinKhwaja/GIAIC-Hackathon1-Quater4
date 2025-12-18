# AGENTS.md
## AI-Native Textbook Project — Agent Governance

This file defines **how Claude CLI works inside this repository**.
It does NOT define requirements.  
It does NOT define architecture.  
It only defines **agent behavior, scope, and execution rules**.

This project follows **Spec-Driven Development (SDD)** using **Spec-Kit Plus**.

---

## 🎯 PROJECT GOAL (HIGH LEVEL)

Build an **AI-Native Textbook Platform** for the course:

**Physical AI & Humanoid Robotics**

Core Deliverables (Hackathon):
1. Docusaurus textbook
2. Embedded RAG chatbot (book-only answers)
3. Auth (BetterAuth)
4. Personalization per chapter
5. Urdu translation per chapter
6. Deployment (Vercel)

---

## 🧠 SOURCE OF TRUTH (VERY IMPORTANT)

Claude MUST follow this hierarchy strictly:

1. `.specify/memory/constitution.md` → RULES
2. `.specify/specs/*/spec.md` → WHAT to build
3. `.specify/specs/*/plan.md` → HOW to build
4. `.specify/specs/*/tasks.md` → STEP-BY-STEP work
5. Codebase → IMPLEMENTATION

❌ **Claude must NEVER invent behavior outside specs**  
❌ **Claude must NEVER fix bugs via prompts**  
✅ **All fixes happen by updating specs / tasks**

---

## 📂 ALLOWED ROOT FOLDERS

Claude MUST NOT create random folders.

Allowed only:

.specify/
apps/
docs/
history/
.claude/

yaml
Copy code

Everything else is forbidden unless explicitly requested.

---

## 🤖 AGENT EXECUTION MODEL (SIMPLIFIED)

This project uses **ONE orchestrator mindset**, not free-for-all agents.

Claude behaves as:

### 🟢 Super Orchestrator (default)
Responsible for:
- Reading specs, plans, tasks
- Executing tasks **exactly**
- Stopping when ambiguity exists
- Asking the user when needed

Sub-roles are **logical**, not physical files.

---

## 🧩 LOGICAL AGENT ROLES (NO YAML FILES)

Claude may internally assume these roles **ONLY when needed**:

| Role | Responsibility |
|----|---------------|
| Spec Analyst | Read & validate specs |
| Planner | Translate spec → plan |
| Task Executor | Implement tasks |
| QA Validator | Verify expected behavior |
| Content Reviewer | MDX structure & clarity |
| Security Reviewer | Auth & access checks |

❌ Do NOT create `/agents/*.yaml`  
❌ Do NOT create `/skills/*`  
(Those were overkill and caused bugs)

---

## 🔁 EXECUTION FLOW (MANDATORY)

Claude MUST follow this flow every time:

READ → VALIDATE → EXECUTE → VERIFY → STOP

yaml
Copy code

### Detailed Steps:
1. Read spec.md, plan.md, tasks.md
2. Confirm task scope
3. Implement ONLY the current task
4. Verify behavior matches spec
5. Stop and wait for next instruction

---

## 🛑 STRICT PROHIBITIONS

Claude MUST NOT:

- Rewrite working code unnecessarily
- “Improve” UI without spec instruction
- Guess auth logic
- Merge frontend + backend logic
- Change deployment architecture
- Use internal knowledge instead of MCP

---

## 🌐 MCP USAGE RULES

Claude MUST use MCP when needed:

| Area | MCP |
|----|----|
| Docusaurus | context7://docusaurus |
| FastAPI | context7://fastapi |
| BetterAuth | context7://better-auth |
| Qdrant | context7://qdrant |
| Neon | context7://neon |
| Vercel | github://vercel/docs |
| Playwright | web::playwright.dev |

❌ No hallucinated APIs  
❌ No assumed syntax  

---

## 🧪 TESTING RESPONSIBILITY

Claude must ensure:

- Frontend UI matches expected behavior
- Auth-based visibility works
- Chatbot inaccessible to guests
- Translation works with ONE button only
- No red MDX errors
- Build succeeds

Tests are validation — **not invention**.

---

## 📜 PROMPT HISTORY (PHR)

Claude MUST record prompts when:
- Running `/sp.specify`
- Running `/sp.plan`
- Running `/sp.tasks`
- Running `/sp.implement` (major changes)

PHRs go to:
history/prompts/<feature>/

yaml
Copy code

---

## 🚀 DEPLOYMENT RULE

Claude does NOT auto-deploy.

Claude may:
- Prepare configs
- Validate build
- Suggest deployment steps

User performs deployment.

---

## 🧠 FINAL RULE (MOST IMPORTANT)

> **If something is broken, STOP and point to the spec or task that needs correction.**

Claude must NEVER “patch around” a bad spec.

---

## ✅ SUCCESS CRITERIA

This project is considered correct when:

- Specs are stable
- Tasks are executable without prompts
- `sp.implement` runs cleanly
- Bugs reduce instead of repeating
- Deployment works without hacks

---

**This AGENTS.md is final and minimal.**