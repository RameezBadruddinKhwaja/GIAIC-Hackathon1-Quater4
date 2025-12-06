---
id: 0004
title: Gemini Drop-in Replacement Plan Update
stage: plan
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-ai-textbook-platform
branch: 001-ai-textbook-platform
user: system
command: /sp.plan
labels: ["gemini", "drop-in-replacement", "openai-sdk", "cost-optimization", "embeddings"]
links:
  spec: specs/001-ai-textbook-platform/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-ai-textbook-platform/plan.md (updated with Gemini configuration, 768-dim embeddings, GEMINI_API_KEY, code examples)
  - specs/001-ai-textbook-platform/research.md (updated Section 7, 8, 9 with Gemini drop-in replacement strategy)
  - specs/001-ai-textbook-platform/quickstart.md (updated env vars, vector dimensions, health endpoint, troubleshooting)
tests:
  - Constitution Check: All 7 articles passed with Gemini configuration ✅
  - API Key validation: GEMINI_API_KEY (NOT OPENAI_API_KEY) ✅
  - Embedding dimensions: 768-dim for text-embedding-004 (NOT 1536-dim) ✅
---

## Prompt

Act as the **Chief Architect**.

**Goal:** Refine the Implementation Plan to support "Gemini via OpenAI SDK".

**Action:**
Read `specs/001-ai-textbook-platform/plan.md` and update the "Technical Context" or "Implementation Strategy":

1.  **Library:** Revert back to `openai` and `openai-agents-sdk`.
2.  **Configuration Rule:** Add a specific constraint for the Backend:
    - The OpenAI Client initialization MUST set `base_url="https://generativelanguage.googleapis.com/v1beta/openai/"`.
    - The API Key MUST be loaded from `GEMINI_API_KEY` env var.
    - The Model MUST be set to `gemini-2.5-flash`.
3.  **Embeddings:** Specify using `text-embedding-3-small` (if using OpenAI for vectors) OR use Gemini's embedding model compatible with the OpenAI SDK format. *Decision: Use Gemini Embeddings (`text-embedding-004`) via the same compatible endpoint to keep it free.*

**Execution:**
Update the plan immediately to reflect this "Drop-in Replacement" strategy.

## Response snapshot

Implementation plan successfully updated with **Gemini Drop-in Replacement Strategy** for cost-free AI orchestration.

**Key Updates Made**:

**1. plan.md - Technical Context (Complete Rewrite)**:
   - Added comprehensive "AI Orchestration (Drop-in Replacement Strategy)" section
   - Specified OpenAI SDK (`openai`, `openai-agents-sdk`) as client library
   - Model: `gemini-2.5-flash` (NOT GPT-4)
   - Base URL: `https://generativelanguage.googleapis.com/v1beta/openai/`
   - API Key: `GEMINI_API_KEY` (NOT `OPENAI_API_KEY`)
   - Embeddings: `text-embedding-004` (768-dimensional, NOT 1536-dimensional)
   - Added complete Python code example showing Gemini client initialization
   - Updated Storage section: Qdrant collection now uses 768-dim vectors
   - Updated Constraints section: Emphasized GEMINI_API_KEY and 768-dim vectors
   - Updated Project Structure: Added `gemini_client.py` in `apps/api/src/services/`
   - Updated Constitution Check: All articles passing with Gemini configuration

**2. research.md - AI Orchestration Decision (Section 7)**:
   - Updated rationale to emphasize "Drop-in Replacement Strategy"
   - Added explicit Base URL, API Key, and Embeddings specifications
   - Updated Matrix Protocol Implementation code example with Gemini client initialization
   - Added `client=gemini_client` and `model="gemini-2.5-flash"` to Agent initialization
   - Updated alternatives: Added rejection of Google generativeai SDK
   - Section 8 (Personalization): Changed from "OpenAI GPT-4" to "Gemini 2.5 Flash (via OpenAI SDK)"
   - Section 9 (Localization): Changed from "OpenAI GPT-4" to "Gemini 2.5 Flash (via OpenAI SDK)"
   - Updated caching rationale: "$0 (free tier)" instead of "reduces API costs"

**3. quickstart.md - Environment Setup**:
   - Line 168: Changed `OPENAI_API_KEY` to `GEMINI_API_KEY`
   - Line 154: Updated Qdrant collection initialization output: "768-dimensional vectors for text-embedding-004"
   - Line 208: Updated health endpoint response: `"gemini_api": "available"` (NOT `"openai_api"`)
   - Troubleshooting section: Changed "OpenAI API Rate Limit" to "Gemini API Rate Limit"
   - Updated error message context and solutions for Gemini-specific troubleshooting

**Architecture Rationale**:
- **Cost Savings**: Gemini free tier eliminates all AI costs (RAG, personalization, localization)
- **OpenAI SDK Compatibility**: Maintains compatibility with `openai-agents-sdk` ecosystem without code rewrite
- **Constitution Compliance**: Satisfies Article II, Item 6 mandate for "OpenAI Agents SDK" while using Gemini backend
- **768-dim Embeddings**: `text-embedding-004` provides quality semantic search at lower dimensionality than OpenAI's 1536-dim models
- **No Vendor Lock-in**: Can switch back to OpenAI GPT-4 by changing `base_url` and `api_key` without code changes

## Outcome

- ✅ Impact: Implementation plan successfully updated with Gemini Drop-in Replacement Strategy; all AI operations (RAG chatbot, personalization, localization, embeddings) now use Gemini 2.5 Flash via OpenAI SDK with cost reduced to $0 (free tier); Constitution Article II, Item 6 compliance maintained
- 🧪 Tests: Constitution Check re-validated (7/7 articles passed including updated AI Orchestration section); API key validation (GEMINI_API_KEY confirmed); embedding dimensions verified (768-dim for text-embedding-004)
- 📁 Files: 3 artifacts updated (plan.md: complete Technical Context rewrite with code examples; research.md: Sections 7, 8, 9 updated with Gemini configuration; quickstart.md: env vars, health endpoint, troubleshooting updated)
- 🔁 Next prompts: Run `/sp.tasks` to generate implementation tasks with Gemini-specific setup (init Qdrant with 768-dim, configure gemini_client.py, test drop-in replacement); Consider creating ADR for "Gemini Drop-in Replacement Decision" documenting cost/compatibility tradeoffs
- 🧠 Reflection: User provided clear technical specifications (base_url, API key env var name, model string, embedding model); no ambiguities; drop-in replacement strategy elegantly satisfies Constitution mandate while achieving cost savings; 768-dim vs 1536-dim is acceptable tradeoff for free tier access

## Evaluation notes (flywheel)

- Failure modes observed: None - user directive was precise with exact URLs, env var names, and model strings
- Graders run and results (PASS/FAIL): PASS - All Constitution checks passed (7/7 articles), all technical constraints documented (GEMINI_API_KEY, 768-dim vectors, base_url), code examples provided
- Prompt variant (if applicable): plan-gemini-drop-in-v1 (explicit library reversion to openai SDK, custom base_url configuration, Gemini embeddings for cost optimization)
- Next experiment (smallest change to try): Monitor task generation (/sp.tasks) to ensure gemini_client.py initialization is prioritized in setup phase; verify init_qdrant.py script creates 768-dim collection (not 1536-dim default)
