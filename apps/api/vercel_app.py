"""
Vercel-compatible FastAPI Application
This file serves as the entry point for Vercel deployments
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import os

# Only load env if not in Vercel environment
if not os.getenv("VERCEL"):
    from dotenv import load_dotenv
    load_dotenv()

app = FastAPI(
    title="Physical AI & Humanoid Robotics Textbook API",
    description="API for the Physical AI & Humanoid Robotics textbook platform",
    version="0.1.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
async def root():
    return {"message": "Physical AI & Humanoid Robotics Textbook API"}

@app.get("/health")
async def health_check():
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

# Import and include the chatbot router
try:
    # Add the current directory to the Python path
    import sys
    import os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

    from api.chatbot import router as chatbot_router
    app.include_router(chatbot_router, prefix="/api")
except ImportError as e:
    print(f"Error importing chatbot router: {e}")
    # Fallback - add a basic API route
    @app.get("/api/test")
    async def test_endpoint():
        return {"status": "API endpoint working", "error": str(e)}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)