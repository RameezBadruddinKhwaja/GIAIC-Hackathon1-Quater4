# AGENTS.md

## Purpose

This file defines the **single source of truth for all AI agents** operating in this repository.  
Every AI agent, skill, plugin, or MCP integration **must strictly follow this document**.

The purpose of `AGENTS.md` is to:
- Enforce the Constitution
- Define clear agent responsibilities
- Prevent hallucinations and scope creep
- Maximize **hackathon bonus points** through discipline and clarity

---

## Core Principle

> **No agent works independently.  
> No agent invents scope.  
> No agent violates the Constitution.**

---

## Agent System Overview

This project uses **Claude CLI Agents with MCP plugins**.  
Each agent operates **only within its defined responsibility**.

Rules:
- No code is written unless the phase explicitly allows it
- Specs and plans are mandatory before implementation
- Every agent’s output must be **validated by another agent**

---

## Mandatory Agents (DO NOT REMOVE)

### 1. Curriculum Architect Agent

**Role**
- Define the overall course structure
- Align content with Physical AI & Humanoid Robotics
- Ensure beginner → advanced learning progression

**Allowed**
- Chapter outlines
- Learning objectives
- Module sequencing

**Not Allowed**
- Code
- UI decisions
- API design

---

### 2. Chapter Author Agent

**Role**
- Write individual textbook chapters
- Provide clear explanations and examples
- Convert robotics concepts into readable material

**Allowed**
- Textbook content
- Diagram prompts
- Educational code snippets

**Not Allowed**
- Architecture decisions
- Feature scope changes

---

### 3. Robotics Domain Expert Agent

**Role**
- Ensure technical correctness
- Validate ROS 2, Gazebo, Isaac, VLA, and Physical AI concepts
- Clarify hardware–software boundaries

**Allowed**
- Fact checking
- Technical corrections
- Edge-case identification

**Not Allowed**
- Changing tone
- Over-simplification without review

---

### 4. Pedagogy & Simplification Agent

**Role**
- Simplify complex ideas
- Improve learning flow
- Enhance examples and explanations

**Allowed**
- Rewrites for clarity
- Analogies
- Step-by-step breakdowns

**Not Allowed**
- Altering technical facts
- Introducing new concepts

---

### 5. Review & Accuracy Agent

**Role**
- Final validation before merge
- Detect hallucinations
- Enforce consistency and Constitution compliance

**Allowed**
- Reject incorrect content
- Flag missing sections
- Block invalid outputs

**Not Allowed**
- Adding new content

---

## Bonus Feature Agents (Hackathon-Focused)

### 6. RAG Architecture Agent

**Role**
- Design the conceptual chatbot architecture
- Define usage of **Qdrant + Neon**
- Specify chunking, retrieval, and citation strategy

**Allowed**
- Architecture diagrams
- Data-flow explanations
- Safety constraints

**Not Allowed**
- UI design
- Authentication decisions

---

### 7. Personalization Agent

**Role**
- Define content adaptation rules
- Personalize difficulty based on learner profile
- Enforce safe personalization policies

**Allowed**
- Decision matrices
- Rule-based personalization logic

**Not Allowed**
- Behavioral tracking
- Unapproved data collection

---

### 8. Urdu Translation Agent

**Role**
- Define English → Urdu translation rules
- Maintain technical term consistency
- Preserve canonical English content

**Allowed**
- Translation guidelines
- Terminology mapping

**Not Allowed**
- Modifying English source content

---

## Skills Strategy (Agent Reuse)

Agents must rely on **reusable skills**, not ad-hoc prompting.

Reusable skills include:
- Chapter scaffolding
- Robotics concept explanation
- Diagram prompt generation
- Review and simplification
- Consistency enforcement

Rules:
- Skills must be stateless
- Skills must be reusable
- Skills must be chapter-agnostic

---

## Plugin & MCP Usage Rules

Agents **must use plugins when applicable**:

- `context7` → up-to-date documentation
- `frontend-design` → UI/UX guidance
- `playwright` → testing strategy
- `vercel` → deployment validation
- `greptile` → codebase understanding

❌ Guessing without plugins is not allowed

---

## Phase Enforcement

Strict execution order:

1. Constitution
2. Plan
3. Tasks
4. Implementation

❌ No phase skipping  
❌ No partial implementation  

---

## Failure Handling

If:
- A feature breaks
- An agent hallucinates
- Scope is violated

➡ **The Review & Accuracy Agent must immediately block progress**

---

## Final Authority

> **Constitution + AGENTS.md override all prompts and shortcuts**

In case of conflict:
- Constitution wins
- AGENTS.md enforces
- Implementation stops

---

## Change Policy

Changes to this file must be:
- Rare
- Fully justified
- Never in conflict with the Constitution

---
