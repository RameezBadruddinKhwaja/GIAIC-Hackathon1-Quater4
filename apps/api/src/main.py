"""
FastAPI Main Application

Physical AI & Humanoid Robotics Textbook Platform - Backend API
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv

load_dotenv()
import os

# Note: Database initialization removed from lifespan for Vercel compatibility
# Tables should be created via Alembic migrations: `alembic upgrade head`

# Initialize FastAPI app
app = FastAPI(
    title="Physical AI Textbook Platform API",
    description="Backend API for AI-Native textbook platform",
    version="1.0.0"
)

# CORS middleware configuration
# Allow all origins for hackathon/development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins
    allow_credentials=False,  # Must be False when using allow_origins=["*"]
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allow_headers=["*"],
    expose_headers=["*"],
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
from src.routers.codegen import router as codegen_router

app.include_router(codegen_router)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
