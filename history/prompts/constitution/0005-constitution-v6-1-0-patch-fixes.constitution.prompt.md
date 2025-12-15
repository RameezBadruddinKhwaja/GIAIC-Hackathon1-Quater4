---
id: 0005
title: Constitution v6.1.0 Patch Fixes
stage: constitution
date: 2025-12-12
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: claude-agent
command: /sp.constitution
labels: ["constitution", "patch", "formatting", "categorization", "workflow-enforcement"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - N/A (governance document)
---

## Prompt

You must update the Constitution (v6.0.0) with ONLY the following 3 fixes.
Do NOT modify anything else. Maintain the exact structure, headings,
article numbers, formatting, and style.

-------------------------------------
FIX #1 — AGENTS.md loading line format correction
-------------------------------------
Locate the line:

"Before performing any action, agents MUST load and follow AGENTS.md,
including:"

Replace it with the stable, parser-safe version:

"Before performing any action, agents MUST load and follow `AGENTS.md`
including:"

-------------------------------------
FIX #2 — Move RAG rule into Security Protocol
-------------------------------------
Locate this misplaced line under **FastAPI Standards**:

"RAG chatbot must strictly cite book content and MUST NOT generate external
knowledge unless explicitly defined in specifications."

REMOVE it from FastAPI Standards and ADD it under **Security Protocol** as:

### RAG Integrity
- RAG chatbot MUST strictly cite book content only.
- External knowledge is forbidden unless explicitly defined in specs.

-------------------------------------
FIX #3 — Prohibit direct spec modification
-------------------------------------
Under ARTICLE I → Enforcement section, ADD this exact clause:

"Agents MUST NOT modify any specification file directly. All specification
changes must be made through Spec-Kit Plus workflows (e.g.,
`/sp.specify update`). Direct manual edits are prohibited."

-------------------------------------
IMPORTANT RULES
-------------------------------------
- Do NOT rename articles.
- Do NOT change any numbering.
- Do NOT rewrite sections other than the specific edits above.
- Apply these 3 fixes cleanly and return the updated Constitution
  as v6.1.0 with an updated Sync Impact Report.

## Response snapshot

Successfully applied all 3 fixes to Constitution, creating v6.1.0 (PATCH):

**Fix #1: AGENTS.md Reference Formatting (Article III)**
- Line 126: Changed `AGENTS.md,` to backtick-wrapped `\`AGENTS.md\``
- Reason: Parser safety and markdown consistency

**Fix #2: RAG Rule Relocation (Article IV)**
- Removed misplaced line from FastAPI Standards
- Added new "RAG Integrity" subsection under Security Protocol (lines 176-179)
- Improved categorization: RAG security rules now properly grouped with security protocols

**Fix #3: Spec Modification Prohibition (Article I)**
- Added enforcement rule to Article I → Enforcement (line 62)
- Explicit prohibition: "Agents MUST NOT modify any specification file directly"
- Mandates use of Spec-Kit Plus workflows (e.g., `/sp.specify update`)

**Version Bump Rationale:**
- PATCH (6.0.0 → 6.1.0): Clarifications, formatting fixes, and rule refinements
- No new principles added
- No backward-incompatible changes
- All existing article numbers and structure preserved

## Outcome

- ✅ Impact: Constitution v6.1.0 created with 3 targeted fixes applied cleanly
- 🧪 Tests: N/A (governance document)
- 📁 Files: Updated `.specify/memory/constitution.md` (305 lines, same structure)
- 🔁 Next prompts: Test enforcement of new spec modification rule during `/sp.specify` workflows
- 🧠 Reflection: Clean patch application - no unintended changes, all fixes applied precisely as requested

## Evaluation notes (flywheel)

- Failure modes observed: None - precise surgical edits completed successfully
- Graders run and results (PASS/FAIL): Manual validation PASS (all 3 fixes verified)
- Prompt variant (if applicable): Constrained edit approach (modify only specified lines)
- Next experiment (smallest change to try): Monitor agent compliance with new spec modification prohibition rule
