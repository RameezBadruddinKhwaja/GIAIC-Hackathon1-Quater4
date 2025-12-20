import tiktoken
from typing import List, Dict, Any, Tuple
from langchain_text_splitters import RecursiveCharacterTextSplitter

class VectorChunkingService:
    def __init__(self, chunk_size: int = 500, chunk_overlap: int = 100):
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap

        # Initialize the text splitter
        self.text_splitter = RecursiveCharacterTextSplitter(
            chunk_size=chunk_size,
            chunk_overlap=chunk_overlap,
            length_function=self._get_token_count,
            separators=["\n\n", "\n", " ", ""]
        )

        # Initialize tokenizer for OpenAI models
        self.tokenizer = tiktoken.encoding_for_model("gpt-3.5-turbo")

    def _get_token_count(self, text: str) -> int:
        """Count the number of tokens in a text using OpenAI's tokenizer"""
        return len(self.tokenizer.encode(text))

    def chunk_text(self, text: str, metadata: Dict[str, Any] = None) -> List[Dict[str, Any]]:
        """
        Split text into chunks with specified size and overlap
        Returns a list of dictionaries with text and metadata
        """
        if metadata is None:
            metadata = {}

        # Split the text into chunks
        chunks = self.text_splitter.split_text(text)

        # Create chunked data with metadata
        chunked_data = []
        for i, chunk in enumerate(chunks):
            chunk_metadata = metadata.copy()
            chunk_metadata.update({
                "chunk_index": i,
                "total_chunks": len(chunks)
            })

            chunked_data.append({
                "text": chunk,
                "metadata": chunk_metadata,
                "token_count": self._get_token_count(chunk)
            })

        return chunked_data

    def chunk_book_content(self, chapters: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Chunk entire book content from a list of chapters
        Each chapter should have: title, content, module, section, url
        """
        all_chunks = []

        for chapter in chapters:
            chapter_title = chapter.get("title", "")
            chapter_content = chapter.get("content", "")
            chapter_module = chapter.get("module", "")
            chapter_section = chapter.get("section", "")
            chapter_url = chapter.get("url", "")

            # Create metadata for this chapter
            chapter_metadata = {
                "chapter": chapter_title,
                "module": chapter_module,
                "section": chapter_section,
                "url": chapter_url
            }

            # Chunk the chapter content
            chapter_chunks = self.chunk_text(chapter_content, chapter_metadata)

            # Add source chapter info to each chunk
            for chunk in chapter_chunks:
                chunk["source_chapter"] = chapter_title
                chunk["source_module"] = chapter_module

            all_chunks.extend(chapter_chunks)

        return all_chunks

# Global instance
vector_chunking_service = VectorChunkingService()