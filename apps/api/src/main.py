"""
FastAPI Main Application

Physical AI & Humanoid Robotics Textbook Platform - Backend API
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv

load_dotenv()
import os

# Initialize FastAPI app
app = FastAPI(
    title="Physical AI Textbook Platform API",
    description="Backend API for AI-Native textbook platform",
    version="1.0.0"
)

# CORS middleware configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # Docusaurus frontend
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
async def root():
    """Root endpoint - API status"""
    return {
        "message": "Physical AI Textbook Platform API",
        "status": "running",
        "version": "1.0.0"
    }

@app.get("/api/health")
async def health_check():
    """Health check endpoint"""
    # Check environment variables
    neon_configured = bool(os.getenv("NEON_CONNECTION_STRING"))
    qdrant_configured = bool(os.getenv("QDRANT_URL") and os.getenv("QDRANT_API_KEY"))
    gemini_configured = bool(os.getenv("GEMINI_API_KEY"))

    return {
        "status": "healthy",
        "database": {
            "neon": "configured" if neon_configured else "not configured",
            "qdrant": "configured" if qdrant_configured else "not configured"
        },
        "ai": {
            "gemini": "configured" if gemini_configured else "not configured"
        }
    }

# Include routers
from src.routers.chat import router as chat_router
from src.routers.auth import router as auth_router
from src.routers.personalize import router as personalize_router
from src.routers.translate import router as translate_router

app.include_router(chat_router)
app.include_router(auth_router)
app.include_router(personalize_router)
app.include_router(translate_router)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
