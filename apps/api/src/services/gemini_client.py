"""
Gemini Client Configuration (Drop-in Replacement for OpenAI)

This module configures the OpenAI SDK to use Google Gemini 2.5 Flash as the backend.

IMPORTANT:
- DO NOT use the Google generativeai SDK directly
- Use OpenAI SDK with custom base_url pointing to Gemini API
- This ensures compatibility with openai-agents-sdk and provides a unified interface

Configuration:
- base_url: https://generativelanguage.googleapis.com/v1beta/openai/
- api_key: Loaded from GEMINI_API_KEY environment variable (NOT OPENAI_API_KEY)
- model: gemini-2.5-flash for chat completions
- embedding model: text-embedding-004 (768-dimensional vectors)

Example Usage:
    client = get_gemini_client()
    response = client.chat.completions.create(
        model="gemini-2.5-flash",
        messages=[{"role": "user", "content": "Explain ROS 2 Topics"}]
    )
"""

import os
from openai import OpenAI
from typing import Optional

_gemini_client: Optional[OpenAI] = None

def get_gemini_client() -> OpenAI:
    """
    Get singleton instance of OpenAI client configured with Gemini backend.

    Returns:
        OpenAI: OpenAI client instance routing to Gemini API

    Raises:
        ValueError: If GEMINI_API_KEY environment variable is not set

    Example:
        >>> client = get_gemini_client()
        >>> response = client.chat.completions.create(
        ...     model="gemini-2.5-flash",
        ...     messages=[{"role": "user", "content": "Hello"}]
        ... )
    """
    global _gemini_client

    if _gemini_client is None:
        api_key = os.getenv("GEMINI_API_KEY")
        if not api_key:
            raise ValueError(
                "GEMINI_API_KEY environment variable not set. "
                "Get your API key from https://makersuite.google.com/app/apikey"
            )

        _gemini_client = OpenAI(
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
            api_key=api_key
        )

    return _gemini_client

# Model constants
CHAT_MODEL = "gemini-2.5-flash"
EMBEDDING_MODEL = "text-embedding-004"
EMBEDDING_DIMENSIONS = 768
