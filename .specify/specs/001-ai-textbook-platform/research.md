# Research Document: Physical AI & Humanoid Robotics Textbook Platform

**Feature**: 001-ai-textbook-platform
**Date**: 2025-12-05
**Phase**: Phase 0 - Technology Research & Architectural Decisions

## Executive Summary

This research document consolidates architectural decisions for building the Physical AI & Humanoid Robotics Textbook Platform targeting 300/100 Hackathon points. All technology choices strictly adhere to Constitution Article II (Authorized Tech Stack) and leverage MCP Tool Mandate (Article III).

## 1. Project Structure Decision

### Decision: Monorepo Strategy with `apps/` and `packages/`

**Rationale**:
- **Code Sharing**: Monorepo enables shared TypeScript types between frontend (Docusaurus) and backend (FastAPI) via `packages/shared-types`
- **Atomic Deployments**: Changes to API contracts and frontend components can be deployed together, reducing version mismatch issues
- **Developer Experience**: Single repository simplifies development workflow - one clone, one install, one place for all code
- **Agentic Infrastructure**: Centralized `.claude/` directory at root houses all specialized agents and skills for Matrix Protocol

**Alternatives Considered**:
1. **Multi-repo (Separate repos for docs and API)**: Rejected due to increased complexity in managing shared types and synchronized deployments
2. **Single `src/` directory**: Rejected as mixing Python (FastAPI) and TypeScript (Docusaurus) in same directory violates separation of concerns

**Structure**:
```
/
├── apps/
│   ├── docs/          # Docusaurus v3 (React/TypeScript)
│   └── api/           # FastAPI (Python 3.11+)
├── packages/
│   └── shared-types/  # Common TypeScript interfaces
├── .claude/
│   ├── agents/        # Specialized AI agents
│   └── skills/        # Reusable knowledge modules
├── specs/             # Feature specifications
├── history/           # PHRs and ADRs
└── docker-compose.yml # Local orchestration
```

---

## 2. Frontend Technology Stack

### Decision: Docusaurus v3 with React/TypeScript

**Rationale**:
- **Constitution Compliance**: Mandated by Article II as the "Documentation Engine"
- **MDX Support**: Native support for Markdown + JSX enables embedding React components (Chat Widget, Personalize/Translate buttons) directly in content
- **Mermaid.js Integration**: Docusaurus has built-in `@docusaurus/theme-mermaid` plugin for ROS 2 diagrams
- **Tabs Component**: `@theme/Tabs` component perfect for "Simulated vs Real Robot" code variations
- **Static Site Generation**: Pre-renders all pages for fast load times (<2s requirement from SC-001)
- **GitHub Pages Ready**: Native support for deployment to GitHub Pages

**Key Libraries**:
- **Styling**: CSS Modules for component-scoped styles + custom CSS for "Matrix Theme"
- **Auth Client**: Better-Auth React hooks (`useBetterAuth`) for authentication state management
- **API Client**: `fetch` API with custom wrapper for `/api/chat`, `/api/personalize`, `/api/translate` endpoints

**Alternatives Considered**:
1. **Next.js**: Rejected as Constitution specifies Docusaurus for documentation-focused sites
2. **VuePress**: Rejected due to Constitution mandate and team familiarity with React ecosystem

---

## 3. Backend Technology Stack

### Decision: FastAPI (Python 3.11+) with Uvicorn

**Rationale**:
- **Constitution Compliance**: Mandated by Article II as "Backend API"
- **Async/Await**: Native async support critical for concurrent RAG queries and personalization requests
- **OpenAPI Auto-generation**: FastAPI automatically generates OpenAPI schema for `/contracts/` documentation
- **Type Safety**: Pydantic models provide runtime validation (aligns with SOC Protocol input sanitization)
- **Performance**: Uvicorn (ASGI server) handles 500 concurrent users requirement (SC-006)
- **MCP Integration**: Python ecosystem has mature clients for Qdrant and Neon via MCP or SDKs

**Key Libraries**:
- **Database ORM**: SQLAlchemy (Async) for Neon PostgreSQL interactions
- **Vector Store**: `qdrant-client` for semantic search in `book_knowledge` collection
- **AI SDK**: OpenAI Agents SDK + ChatKit for RAG orchestration
- **Auth**: Better-Auth Python SDK for token validation and user session management
- **Security**: `python-multipart` + custom sanitization layer for XSS/SQL injection prevention

**Alternatives Considered**:
1. **Django**: Rejected due to heavier footprint and Constitution mandating FastAPI
2. **Flask**: Rejected as lacks native async support and OpenAPI auto-generation

---

## 4. Database Architecture Decision

### Decision: Neon (Serverless PostgreSQL) for Relational Data

**Rationale**:
- **Constitution Compliance**: Mandated by Article II as "Primary Database"
- **Serverless**: Auto-scales to zero during inactivity (cost-effective for hackathon evaluation periods)
- **Postgres Compatibility**: Full PostgreSQL compatibility enables complex queries (e.g., JOIN on `users` and `chat_logs`)
- **MCP Integration**: Neon has official MCP server (`@neondatabase/mcp-server-neon`) for safe database interactions
- **Connection Pooling**: Built-in connection pooling handles 500 concurrent user requirement

**Schema Design**:
```sql
-- Users table (FR-013: User profiles)
CREATE TABLE users (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  email VARCHAR(255) UNIQUE NOT NULL,
  auth_provider VARCHAR(50) NOT NULL, -- 'email' or 'github'
  hardware_profile VARCHAR(50), -- 'rtx_4090' or 'jetson_orin_nano'
  programming_language VARCHAR(20), -- 'python' or 'cpp'
  created_at TIMESTAMP DEFAULT NOW(),
  last_login TIMESTAMP
);

-- Chat logs table (FR-009: Audit trail)
CREATE TABLE chat_logs (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID REFERENCES users(id),
  query_text TEXT NOT NULL,
  response_text TEXT NOT NULL,
  cited_chapters JSONB, -- Array of chapter IDs
  skills_loaded JSONB, -- Array of skill names (Matrix Protocol)
  sanitized_input BOOLEAN DEFAULT true,
  created_at TIMESTAMP DEFAULT NOW()
);

-- Personalized content cache (FR-017: Cached personalized versions)
CREATE TABLE personalized_content (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID REFERENCES users(id),
  chapter_id VARCHAR(100) NOT NULL,
  hardware_profile VARCHAR(50) NOT NULL,
  personalized_mdx TEXT NOT NULL,
  generated_at TIMESTAMP DEFAULT NOW(),
  cache_expiry TIMESTAMP DEFAULT NOW() + INTERVAL '7 days'
);

-- Translated content cache (FR-021: Cached translations)
CREATE TABLE translated_content (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID REFERENCES users(id),
  chapter_id VARCHAR(100) NOT NULL,
  target_language VARCHAR(20) NOT NULL, -- 'roman_urdu' or 'formal_urdu'
  translated_mdx TEXT NOT NULL,
  generated_at TIMESTAMP DEFAULT NOW(),
  cache_expiry TIMESTAMP DEFAULT NOW() + INTERVAL '7 days'
);

-- Audit logs table (FR-025: Security events)
CREATE TABLE audit_logs (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  event_type VARCHAR(100) NOT NULL, -- 'failed_auth', 'sanitized_input', 'rate_limit', etc.
  user_id UUID REFERENCES users(id),
  ip_address INET,
  action_details JSONB,
  timestamp TIMESTAMP DEFAULT NOW()
);
```

**Alternatives Considered**:
1. **MongoDB**: Rejected as Constitution mandates relational database (Neon PostgreSQL)
2. **Supabase**: Rejected due to Constitution specificity to Neon

---

## 5. Vector Store Decision

### Decision: Qdrant Cloud for RAG Semantic Search

**Rationale**:
- **Constitution Compliance**: Mandated by Article II as "RAG Database"
- **Vector Search**: Optimized for cosine similarity search on OpenAI embeddings (`text-embedding-ada-002`)
- **Metadata Filtering**: Supports filtering by `chapter_id`, `section_id`, `part_number` (from Docusaurus frontmatter)
- **Cloud Free Tier**: 1GB storage + 1M vectors sufficient for 13 weeks of course content
- **MCP Integration**: Qdrant has official MCP server for safe vector operations
- **Performance**: Sub-second search (supports SC-003: 3-second chatbot response time)

**Collection Schema**:
```python
# Collection: book_knowledge
{
  "vectors": {
    "size": 1536,  # OpenAI ada-002 embedding dimension
    "distance": "Cosine"
  },
  "payload_schema": {
    "chapter_id": "keyword",     # e.g., "week-01-ros2-basics"
    "section_id": "keyword",     # e.g., "nodes-and-topics"
    "part_number": "integer",    # 1-4 (The Nervous System, Digital Twin, Brain, VLA)
    "week_number": "integer",    # 1-13
    "content_text": "text",      # Original MDX content chunk
    "content_type": "keyword",   # 'prose', 'code', 'mermaid'
    "hardware_context": "keyword" # 'rtx_4090', 'jetson_orin', 'both'
  }
}
```

**Ingestion Strategy**:
- **Chunking**: Split MDX files into 500-word chunks with 50-word overlap (preserves context)
- **Frontmatter Parsing**: Extract `chapter_id`, `part_number`, `week_number` from Docusaurus frontmatter
- **Code Block Handling**: Index code blocks separately with `content_type: 'code'` for accurate citation
- **Mermaid Preservation**: Extract Mermaid diagram text for keyword search (not embedded)

**Alternatives Considered**:
1. **Pinecone**: Rejected due to Constitution mandate for Qdrant
2. **Weaviate**: Rejected as not specified in Constitution

---

## 6. Authentication Decision

### Decision: Better-Auth with Email/GitHub OAuth

**Rationale**:
- **Constitution Compliance**: Mandated by Article II as "Authentication" (Bonus Requirement)
- **Multi-Provider**: Supports both email/password and GitHub OAuth (FR-012)
- **Session Management**: Built-in JWT token generation and refresh logic
- **Onboarding Flow**: Custom middleware for post-auth onboarding questionnaire (FR-012: "Do you own NVIDIA RTX GPU?")
- **Security**: Built-in CSRF protection and rate limiting (aligns with FR-026)
- **TypeScript Client**: `@better-auth/react` hooks integrate seamlessly with Docusaurus

**Implementation Details**:
- **Backend**: Better-Auth Python SDK validates JWT tokens on `/api/chat`, `/api/personalize`, `/api/translate` endpoints
- **Frontend**: `useBetterAuth()` hook manages auth state and redirects
- **Onboarding**: Custom `/api/auth/onboarding` endpoint stores hardware_profile and programming_language in Neon `users` table

**Alternatives Considered**:
1. **Auth0**: Rejected due to external dependency and Constitution mandate for Better-Auth
2. **NextAuth.js**: Rejected as Docusaurus is not Next.js-based

---

## 7. AI Orchestration Decision

### Decision: OpenAI Agents SDK + ChatKit for RAG

**Rationale**:
- **Constitution Compliance**: Mandated by Article II as "AI Orchestration" (Article II, Item 6)
- **Drop-in Replacement Strategy**: OpenAI Agents SDK configured with `base_url="https://generativelanguage.googleapis.com/v1beta/openai/"` to use **Google Gemini 2.5 Flash** (`gemini-2.5-flash`)
- **API Key**: Loaded from `GEMINI_API_KEY` environment variable (NOT `OPENAI_API_KEY`)
- **Embeddings**: Gemini `text-embedding-004` (768-dimensional) via same compatible endpoint - **keeps costs at $0 (free tier)**
- **Implementation Note**: Use OpenAI SDK as client library (NOT Google's generativeai SDK) but route all requests to Gemini API via custom `base_url` configuration
- **Agents SDK**: Provides `Agent` abstraction for defining chatbot behavior, tool calling (Qdrant search), and response formatting
- **ChatKit**: Pre-built conversation management with message history and context window optimization
- **Dynamic Skill Loading (Matrix Protocol)**: Agents SDK supports loading external "tools" (skills) based on query keywords
- **Citation Generation**: Custom tool (`cite_source`) appends chapter links to responses (FR-009)

**Matrix Protocol Implementation**:
```python
from openai import OpenAI
from openai_agents_sdk import Agent, Tool
import os

# Initialize OpenAI client with Gemini backend (Drop-in Replacement)
gemini_client = OpenAI(
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
    api_key=os.getenv("GEMINI_API_KEY")  # NOT OPENAI_API_KEY
)

# Define skill loader tool
def load_skill(skill_name: str) -> str:
    """Dynamically loads context from .claude/skills/{skill_name}.md"""
    skill_path = f".claude/skills/{skill_name}.md"
    with open(skill_path, "r") as f:
        return f.read()

# Create agent with dynamic skill loading (using Gemini 2.5 Flash backend)
chatbot_agent = Agent(
    name="RAG Chatbot",
    client=gemini_client,
    model="gemini-2.5-flash",
    tools=[
        Tool(name="search_qdrant", func=search_qdrant),
        Tool(name="load_skill", func=load_skill),
        Tool(name="cite_source", func=cite_source)
    ],
    instructions="Answer questions using only book content. If query mentions 'SLAM', load 'ros2-mastery' skill. Always cite sources."
)
```

**Skill Detection Logic**:
- **Keyword Mapping**: `{"SLAM": "ros2-mastery", "Jetson": "edge-computing", "Isaac": "nvidia-isaac"}`
- **Backend Logs**: `print(f"Matrix Skill Loaded: {skill_name}")` (FR-009)

**Alternatives Considered**:
1. **LangChain**: Rejected due to Constitution mandate for OpenAI Agents SDK
2. **LlamaIndex**: Rejected as not specified in Constitution
3. **Google generativeai SDK**: Rejected to maintain compatibility with OpenAI Agents SDK ecosystem; using OpenAI SDK as drop-in replacement instead

---

## 8. Personalization Engine Decision

### Decision: Gemini 2.5 Flash (via OpenAI SDK) with Custom Prompt Templates

**Rationale**:
- **Drop-in Replacement**: Use Gemini 2.5 Flash via OpenAI SDK with custom `base_url` (same configuration as RAG chatbot)
- **Content Rewriting**: Gemini 2.5 Flash can rewrite code blocks from "Local Isaac Sim" to "Cloud/AWS" instructions based on `hardware_profile`
- **Context Preservation**: Prompt template includes original MDX, user hardware profile, and rewriting instructions
- **Caching**: Store rewritten content in Neon `personalized_content` table with 7-day TTL (reduces API costs to $0 - free tier)
- **Performance**: Avg 5-7s for 2000-word chapter rewrite (within SC-004: 10s requirement)

**Prompt Template**:
```
You are a technical content adapter for Physical AI robotics tutorials.

User Hardware Profile: {hardware_profile}
Original Content:
{original_mdx}

Task: Rewrite code blocks ONLY to optimize for {hardware_profile}. If 'jetson_orin_nano', reduce memory usage, use CPU-based processing. If 'rtx_4090', enable high-fidelity rendering and GPU acceleration. Preserve all prose, Mermaid diagrams, and structure.
```

**Alternatives Considered**:
1. **Rule-Based Replacement**: Rejected as too brittle for complex code transformations
2. **Claude Opus**: Rejected due to Constitution mandate for OpenAI SDK ecosystem
3. **OpenAI GPT-4**: Initially considered but replaced with Gemini 2.5 Flash for cost savings (free tier) while maintaining OpenAI SDK compatibility

---

## 9. Localization Engine Decision

### Decision: Gemini 2.5 Flash (via OpenAI SDK) with Language-Specific Prompts

**Rationale**:
- **Drop-in Replacement**: Use Gemini 2.5 Flash via OpenAI SDK with custom `base_url` (same configuration as RAG chatbot)
- **Technical Translation**: Gemini 2.5 Flash handles technical Urdu translation while preserving code blocks
- **Dual Language Support**: Supports both Roman Urdu and Formal Urdu based on user preference
- **Code Preservation**: Prompt explicitly instructs to keep code blocks, Mermaid syntax, and identifiers in English
- **Caching**: Store translations in Neon `translated_content` table with 7-day TTL (reduces costs to $0 - free tier)

**Prompt Template**:
```
You are a technical translator specializing in Physical AI and robotics education.

Target Language: {target_language} (roman_urdu or formal_urdu)
Original Content:
{original_mdx}

Task: Translate all explanatory text to {target_language}. DO NOT translate code blocks, Mermaid diagram syntax, or technical identifiers (ROS 2, URDF, Isaac Sim). Preserve formatting and structure.
```

**Alternatives Considered**:
1. **Google Translate API**: Rejected due to poor handling of technical terminology and code preservation
2. **DeepL**: Rejected as lacks Urdu support

---

## 10. Testing Strategy Decision

### Decision: Pytest (Backend) + Playwright (Frontend E2E)

**Rationale**:
- **Backend (Pytest)**: FastAPI has native Pytest integration via `TestClient` for endpoint testing
- **Frontend (Playwright)**: Constitution Article III mandates Playwright as "The Navigator" for E2E testing
- **Contract Testing**: OpenAPI schema generated by FastAPI can be validated against frontend API calls
- **Security Testing**: Pytest fixtures can inject XSS/SQL injection payloads to verify sanitization (FR-023)

**Test Coverage**:
- **Backend**: `test_chat_endpoint.py`, `test_personalize_endpoint.py`, `test_auth_flow.py`
- **Frontend**: `test_chat_widget.spec.ts`, `test_personalize_button.spec.ts`, `test_urdu_translation.spec.ts`

**Alternatives Considered**:
1. **Cypress**: Rejected due to Constitution mandate for Playwright
2. **Jest (Frontend Unit Tests)**: Complementary, not alternative; can be used alongside Playwright

---

## 11. Deployment Strategy Decision

### Decision: GitHub Pages (Docs) + Render (API)

**Rationale**:
- **Docs Deployment**: Constitution Article VI mandates GitHub Pages via GitHub Actions
- **API Deployment**: Render supports Python runtime with free tier suitable for hackathon evaluation
- **CI/CD**: GitHub Actions workflow builds Docusaurus site and triggers Render deployment on push to `main`
- **Environment Secrets**: GitHub Secrets store `OPENAI_API_KEY`, `QDRANT_API_KEY`, `NEON_CONNECTION_STRING`

**Alternative for API Deployment**:
- **Vercel**: Also viable (supports Python serverless functions), can be swapped post-hackathon

---

## 12. Agentic Infrastructure Decision (Matrix Protocol)

### Decision: Specialized Agents (`.claude/agents/`) + Reusable Skills (`.claude/skills/`)

**Rationale**:
- **Role-Based Context**: Each agent (author.md, coder.md, architect.md, qa.md, translator.md) provides specialized context for AI interactions
- **Knowledge Modularity**: Skills (ros2-mastery.md, docusaurus-guru.md, matrix-loader.md, better-auth.md) are reusable across agents
- **Dynamic Loading**: Matrix Protocol enables runtime skill loading based on query context (FR-027)
- **Version Control**: All agent definitions and skills are version-controlled alongside code

**Agent Definitions**:
1. **author.md**: Physics & Robotics Professor persona for content creation
2. **coder.md**: Full-Stack Engineer persona for RAG/FastAPI/React implementation
3. **architect.md**: System Designer persona for folder structure and ADR creation
4. **qa.md**: SOC Analyst persona for security testing and sanitization validation
5. **translator.md**: Urdu Localization Expert persona for translation feature

**Skill Definitions**:
1. **ros2-mastery.md**: Deep knowledge of ROS 2 Nodes, Topics, Services for content accuracy
2. **docusaurus-guru.md**: Mastery of Docusaurus Admonitions, Tabs, Swizzling for frontend implementation
3. **matrix-loader.md**: Logic for dynamic skill loading and backend logging
4. **better-auth.md**: Implementation guide for Better-Auth integration (bonus feature)

---

## 13. MCP Tool Usage Strategy (Constitution Article III)

### Decision: Prioritize MCP Servers Over Raw SDKs

**Rationale**:
- **Context7 (The Librarian)**: MUST fetch latest Docusaurus v3, FastAPI, and Neon documentation to avoid hallucinated configuration
- **GitHub MCP (The Operator)**: MUST use for PR creation, issue tracking, GitHub Pages deployment workflow triggers
- **Playwright (The Navigator)**: Use for verifying deployed Docusaurus pages, taking screenshots for documentation

**Implementation**:
- **Research Phase**: Use Context7 to fetch Docusaurus plugin documentation before implementing Mermaid/Tabs
- **Deployment Phase**: Use GitHub MCP to create deployment workflows instead of manual `.github/workflows/` creation
- **Testing Phase**: Use Playwright MCP to verify deployed site loads correctly post-deployment

---

## Architectural Decision Summary

| **Component** | **Technology** | **Rationale** |
|---------------|----------------|---------------|
| Frontend | Docusaurus v3 (React/TypeScript) | Constitution mandate + MDX/Tabs/Mermaid support |
| Backend API | FastAPI (Python 3.11+) + Uvicorn | Constitution mandate + async + OpenAPI auto-gen |
| Database | Neon (Serverless PostgreSQL) | Constitution mandate + auto-scaling + MCP integration |
| Vector Store | Qdrant Cloud | Constitution mandate + semantic search + free tier |
| Auth | Better-Auth (Email/GitHub OAuth) | Constitution mandate + onboarding flow + security |
| AI Orchestration | OpenAI Agents SDK + ChatKit | Constitution mandate + dynamic skill loading |
| Testing | Pytest (Backend) + Playwright (E2E) | Constitution mandate + contract testing |
| Deployment | GitHub Pages (Docs) + Render (API) | Constitution mandate + free tier + CI/CD |
| Agentic Infra | `.claude/agents/` + `.claude/skills/` | Matrix Protocol + role-based context + knowledge modularity |

---

## Next Steps

1. **Phase 1**: Create `data-model.md` (entity schemas from research.md section 4)
2. **Phase 1**: Generate API contracts in `/contracts/` (OpenAPI schema for endpoints)
3. **Phase 1**: Create `quickstart.md` (local setup instructions)
4. **Phase 1**: Run `.specify/scripts/bash/update-agent-context.sh claude` to update agent context
5. **Phase 2**: Generate `tasks.md` with implementation tasks based on this research

---

## Constitution Compliance Checklist

- ✅ **Article I (SDD)**: Research completed before any implementation
- ✅ **Article II (Tech Stack)**: All technologies match authorized stack (Docusaurus, Qdrant, Neon, FastAPI, Better-Auth, OpenAI Agents SDK)
- ✅ **Article III (MCP Mandate)**: Context7, GitHub MCP, Playwright usage documented
- ✅ **Article IV (Protocols)**: Language protocol followed (English for technical docs)
- ✅ **Article V (Publishing)**: Docusaurus conventions (Tabs, Mermaid, Admonitions) researched
- ✅ **Article VI (Engineering)**: SOC Protocol (input sanitization, .env secrets) covered in auth and security sections
- ✅ **Article VII (Intelligence)**: Agentic infrastructure (.claude/) designed for knowledge preservation

---

**Research Phase Complete**: All technology decisions documented and justified. Ready to proceed to Phase 1 (Design & Contracts).
