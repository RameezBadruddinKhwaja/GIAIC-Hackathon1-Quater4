# Skill: RAG Chatbot Architecture

## Overview
A "Matrix-style" embedded chatbot that answers questions based on the book's content.

## Components
1. **Ingestion Pipeline:**
   - Read .md files from `/docs`.
   - Chunk text (Overlap: 200 chars).
   - Generate Embeddings (text-embedding-3-small).
   - Upsert to Qdrant Cloud.

2. **Retrieval (RAG):**
   - User query -> Embedding.
   - Qdrant Search -> Top 3 chunks.
   - System Prompt: "You are an assistant for this book. Answer using only the context below."

3. **Frontend Widget:**
   - Floating chat bubble on Docusaurus pages.
   - React component using OpenAI ChatKit (or custom fetch).
