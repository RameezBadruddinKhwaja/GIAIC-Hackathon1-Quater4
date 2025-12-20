from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import os
from dotenv import load_dotenv

# Only load env if not in Vercel environment
if not os.getenv("VERCEL"):
    load_dotenv()

# Import API routes after imports to avoid circular dependencies
from .api import chatbot

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
    return {"status": "healthy"}

# Include API routes
app.include_router(chatbot.router, prefix="/api")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)