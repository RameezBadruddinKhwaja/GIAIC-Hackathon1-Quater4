import os
from typing import List, Dict, Any
from ..models.chat_models import Citation, ChatResponse
from ..db.qdrant import qdrant_manager
from .embedding_service import embedding_service
from dotenv import load_dotenv
import google.generativeai as genai

load_dotenv()

class RAGService:
    def __init__(self):
        # Set Google Gemini API key from environment variables
        gemini_api_key = os.getenv("GEMINI_API_KEY")
        genai.configure(api_key=gemini_api_key)

        # Initialize the Gemini model
        self.model = genai.GenerativeModel('gemini-1.5-flash')  # Using gemini-1.5-flash for cost efficiency

    def retrieve_context(self, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """Retrieve relevant context from the vector database based on the query"""
        # Generate embedding for the query
        query_embedding = embedding_service.generate_embedding(query)

        # Search for similar content in Qdrant
        search_results = qdrant_manager.search_similar(query_embedding, top_k=top_k)

        # Extract the relevant chunks
        context_chunks = []
        for result in search_results:
            payload = result.get("payload", {})
            text = payload.get("text", "")
            if text:
                context_chunks.append({
                    "text": text,
                    "score": result.get("score", 0),
                    "metadata": payload.get("metadata", {})
                })

        return context_chunks

    def generate_answer(self, query: str, context_chunks: List[Dict[str, Any]]) -> str:
        """Generate an answer based on the query and retrieved context"""
        try:
            # Construct the context from retrieved chunks
            context_str = "\n".join([chunk["text"] for chunk in context_chunks])

            # Construct the prompt
            prompt = f"""Answer the question based only on the following textbook excerpts. Do not use any external knowledge.

Context:
{context_str}

Question: {query}

Answer: """

            # Configure generation parameters
            generation_config = genai.GenerationConfig(
                max_output_tokens=500,
                temperature=0.3,
            )

            # Call Google Gemini API to generate the answer
            response = self.model.generate_content(
                prompt,
                generation_config=generation_config,
                safety_settings={
                    "HARM_CATEGORY_HARASSMENT": "BLOCK_NONE",
                    "HARM_CATEGORY_HATE_SPEECH": "BLOCK_NONE",
                    "HARM_CATEGORY_SEXUALLY_EXPLICIT": "BLOCK_NONE",
                    "HARM_CATEGORY_DANGEROUS_CONTENT": "BLOCK_NONE"
                }
            )

            answer = response.text.strip()
            return answer
        except Exception as e:
            print(f"Error generating answer: {e}")
            return "I encountered an error while generating the answer. Please try again."

    def check_off_topic(self, query: str, context_chunks: List[Dict[str, Any]]) -> bool:
        """Check if the query is off-topic based on similarity scores"""
        if not context_chunks:
            return True  # No context found, likely off-topic

        # If all scores are low (below 0.6), consider it off-topic
        low_score_threshold = 0.6
        all_low_scores = all(chunk["score"] < low_score_threshold for chunk in context_chunks)

        return all_low_scores

    def process_query(self, query: str, top_k: int = 5) -> ChatResponse:
        """Process a user query through the RAG pipeline"""
        # Retrieve relevant context
        context_chunks = self.retrieve_context(query, top_k=top_k)

        # Check if the query is off-topic
        if self.check_off_topic(query, context_chunks):
            answer = "I can only answer questions about the Physical AI & Humanoid Robotics textbook. Please ask a question related to the textbook content."
            return ChatResponse(
                answer=answer,
                citations=[],
                session_id=""
            )

        # Generate the answer
        answer = self.generate_answer(query, context_chunks)

        # Extract citations from context chunks
        citations = []
        for chunk in context_chunks:
            metadata = chunk.get("metadata", {})
            if "chapter" in metadata and "section" in metadata and "url" in metadata:
                citations.append(Citation(
                    chapter=metadata["chapter"],
                    section=metadata["section"],
                    url=metadata["url"]
                ))

        return ChatResponse(
            answer=answer,
            citations=citations,
            session_id=""  # This would be populated with actual session ID in real implementation
        )

# Global instance
rag_service = RAGService()