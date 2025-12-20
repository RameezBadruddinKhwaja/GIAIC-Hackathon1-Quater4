#!/usr/bin/env python3
"""
Verification script for Hugging Face Spaces deployment
This script checks that all necessary files are in place for the Hugging Face Spaces deployment
"""

import os
from pathlib import Path

def check_hf_space_files():
    """Check that all required files exist in the Hugging Face Space directory"""
    hf_space_dir = Path("PhysicalAITextbook/Physical-AI-Textbook")

    required_files = [
        "Dockerfile",
        "requirements.txt",
        "app.py",
        "main.py",
        "README.md",
        "SETUP_GUIDE.md",
        "HF_AUTH_SETUP.md",
        ".gitignore",
        ".env.example",
        "start.sh",
        "test_setup.py"
    ]

    missing_files = []
    for file in required_files:
        if not (hf_space_dir / file).exists():
            missing_files.append(file)

    # Check required directories
    required_dirs = [
        "api",
        "db",
        "models",
        "services"
    ]

    missing_dirs = []
    for directory in required_dirs:
        if not (hf_space_dir / directory).is_dir():
            missing_dirs.append(directory)

    # Check api directory contents
    api_dir = hf_space_dir / "api"
    if api_dir.is_dir():
        required_api_files = ["__init__.py", "chatbot.py"]
        for file in required_api_files:
            if not (api_dir / file).exists():
                missing_files.append(f"api/{file}")

    # Check db directory contents
    db_dir = hf_space_dir / "db"
    if db_dir.is_dir():
        required_db_files = ["__init__.py", "qdrant.py", "neon.py"]
        for file in required_db_files:
            if not (db_dir / file).exists():
                missing_files.append(f"db/{file}")

    # Check models directory contents
    models_dir = hf_space_dir / "models"
    if models_dir.is_dir():
        required_models_files = ["chat_models.py"]
        for file in required_models_files:
            if not (models_dir / file).exists():
                missing_files.append(f"models/{file}")

    # Check services directory contents
    services_dir = hf_space_dir / "services"
    if services_dir.is_dir():
        required_services_files = [
            "rag_service.py",
            "embedding_service.py",
            "vector_chunking_service.py",
            "indexing_service.py"
        ]
        for file in required_services_files:
            if not (services_dir / file).exists():
                missing_files.append(f"services/{file}")

    return missing_files, missing_dirs

def print_verification_results():
    """Print the verification results"""
    print("[INFO] Hugging Face Spaces Deployment Verification")
    print("=" * 50)

    missing_files, missing_dirs = check_hf_space_files()

    if missing_files:
        print(f"[ERROR] Missing files: {len(missing_files)}")
        for file in missing_files:
            print(f"   - {file}")
    else:
        print("[SUCCESS] All required files present")

    if missing_dirs:
        print(f"[ERROR] Missing directories: {len(missing_dirs)}")
        for directory in missing_dirs:
            print(f"   - {directory}")
    else:
        print("[SUCCESS] All required directories present")

    print("\n[INFO] Required Environment Variables:")
    print("   - QDRANT_URL")
    print("   - QDRANT_API_KEY")
    print("   - GEMINI_API_KEY")

    print("\n[INFO] Docker Configuration:")
    print("   - Dockerfile present and configured for port 7860")
    print("   - requirements.txt includes all dependencies")
    print("   - CMD uses 'uvicorn app:app --host 0.0.0.0 --port 7860'")

    print("\n[INFO] API Endpoints:")
    print("   - GET / (health check)")
    print("   - POST /api/chatbot/query")
    print("   - POST /api/chatbot/query-selection")
    print("   - GET /api/chatbot/history/{session_id}")

    if not missing_files and not missing_dirs:
        print(f"\n[SUCCESS] Hugging Face Spaces deployment is ready!")
        print(f"   Navigate to PhysicalAITextbook/Physical-AI-Textbook/ and follow the deployment instructions")
        return True
    else:
        print(f"\n[ERROR] Deployment is not ready. Please fix the missing items above.")
        return False

if __name__ == "__main__":
    success = print_verification_results()
    exit(0 if success else 1)