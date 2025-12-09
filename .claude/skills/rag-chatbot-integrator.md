# rag-chatbot-integrator Skill (Bonus)

## Purpose
Integrate RAG chatbot functionality using OpenAI Agents + Qdrant + ChatKit for contextual text selection.

## Usage Context
Used by content-implementor agent when implementing Category 6.1 (RAG Chatbot) tasks.

## Agent Assignments
- Primary: content-implementor
- Secondary: super-orchestrator (deployment)

## Key Patterns

### Content Ingestion
```python
# Parse Docusaurus MDX files
# Chunk into 500-word segments with 50-word overlap
# Generate 768-dim embeddings with Gemini text-embedding-004
# Upsert to Qdrant with payload (chapter_id, section_id, part_number, week_number)
```

### RAG Pipeline
```python
# Embed user query with text-embedding-004
# Search Qdrant (top_k=5, cosine similarity > 0.7)
# Use Gemini chat completion with system prompt:
# "Answer using only provided book content. Always cite chapter sources."
# Extract citations from Qdrant results
```

### ChatKit Integration
```typescript
// React ChatWidget with text selection handler
// Detect user text selection on page
// Show "Ask about this" button
// Pass selected_text to /api/chat endpoint
```

## Constitution Reference
- Article IX (Skill System) - Bonus Skill
- Article X (Bonus Features) - RAG Chatbot (+50 points)
